/*
i960SxChipset
Copyright (c) 2020-2021, Joshua Scoggins
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR 
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/// i960Sx chipset mcu, based on atmega1284p with fuses set for:
/// - 20Mhz crystal
/// - D1 acts as CLKO
/// Language options:
/// - C++17
/// Board Platform: MightyCore
#include <SPI.h>
#include <SdFat.h>
#include "Pinout.h"

#include "ProcessorSerializer.h"
#include "MemoryThing.h"
#include "MemoryMappedFileThing.h"
#include "CoreChipsetFeatures.h"
#include "PSRAMChip.h"


/**
 * @brief Describes a single cache line which associates an address with 32 bytes of storage
 */
class CacheEntry final {
public:
    static constexpr size_t NumBytesCached = TargetBoard::cacheLineSize();
    static constexpr size_t NumWordsCached = NumBytesCached / sizeof(SplitWord16);
    static constexpr size_t LowestBitCount = 4;
    static constexpr size_t TagIndexSize = 8;
    static constexpr size_t UpperBitCount = 32 - (LowestBitCount + TagIndexSize);
    static_assert((LowestBitCount + TagIndexSize + UpperBitCount) == 32, "TaggedAddress must map exactly to a 32-bit address");
    static constexpr byte TagMask = static_cast<byte>(0xFF << LowestBitCount); // exploit shift beyond
    static constexpr byte OffsetMask = static_cast<byte>(~TagMask) >> 1;  // remember that this 16-bit aligned
    // sanity checks for optimization purposes
    static_assert(LowestBitCount <= 8, "Offset must fit within a byte!");
    static_assert(TagIndexSize <= 8, "Tag size index is too large for a single byte");

    union TaggedAddress {
        constexpr explicit TaggedAddress(Address value = 0) noexcept : base(value) { }
        void clear() noexcept { base = 0; }
        [[nodiscard]] constexpr auto getTagIndex() const noexcept { return tagIndex; }
        [[nodiscard]] constexpr auto getAddress() const noexcept { return base; }
        [[nodiscard]] constexpr auto getLowest() const noexcept { return lowest; }
        [[nodiscard]] constexpr auto getRest() const noexcept { return rest; }
        [[nodiscard]] TaggedAddress aligned() const noexcept {
            TaggedAddress result(base);
            result.lowest = 0;
            return result;
        }
        [[nodiscard]] bool restEqual(const TaggedAddress& other) const noexcept { return getRest() == other.getRest(); }
    private:
        Address base;
        struct {
            byte lowest : LowestBitCount;
            byte tagIndex : TagIndexSize;
            Address rest : UpperBitCount;
        };
    };
public:
    void reset(const TaggedAddress& newTag) noexcept {
        // no match so pull the data in from main memory
        if (needsFlushing()) {
            // just do the write out to disk to save time
            // still an expensive operation
            OnboardPSRAMBlock::write(tag.getAddress(), reinterpret_cast<byte*>(data), sizeof(data));
        }
        flags_ = IsClean | IsValid;
        // since we have called reset, now align the new address internally
        tag = newTag.aligned();
        // this is a _very_ expensive operation
        OnboardPSRAMBlock::read(tag.getAddress(), reinterpret_cast<byte*>(data), sizeof (data));
    }
    /**
     * @brief Clear the entry without saving what was previously in it, necessary if the memory was reused for a different purpose
     */
    void clear() noexcept {
        // clear all flags
        flags_ = IsClean | IsInvalid;
        tag.clear();
        for (auto& a : data) {
            a.wholeValue_ = 0;
        }
    }
    [[nodiscard]] constexpr bool matches(const TaggedAddress& addr) const noexcept { return isValid() && (tag.restEqual(addr)); }
    [[nodiscard]] constexpr auto get(byte offset) const noexcept { return data[offset].getWholeValue(); }
    template<bool terminateEarlyOnMatch = true>
    void set(byte offset, LoadStoreStyle style, SplitWord16 value) noexcept {
        // while unsafe, assume it is correct because we only get this from the ProcessorSerializer, perhaps directly grab it?
        auto& target = data[offset];
        auto oldValue = target.getWholeValue();
        if constexpr (terminateEarlyOnMatch) {
            if (oldValue == value.getWholeValue()) {
                // Check and see if the oldValue equals the new value and return early if it is true
                // this does add some overhead overall but if we aren't actually making a change then there is no point in writing
                // to the cache line
                return;
            }
        }
        switch (style) {
            case LoadStoreStyle::Full16:
                target.bytes[0] = value.bytes[0];
                target.bytes[1] = value.bytes[1];
                break;
            case LoadStoreStyle::Lower8:
                target.bytes[0] = value.bytes[0];
                break;
            case LoadStoreStyle::Upper8:
                target.bytes[1] = value.bytes[1];
                break;
            default:
                signalHaltState(F("BAD LOAD STORE STYLE FOR SETTING A CACHE LINE"));
        }
        // do a comparison at the end to see if we actually changed anything
        // the idea is that if the values don't change don't mark the cache line as dirty again
        // it may already be dirty but don't force the matter on any write
        // we can get here if it is a lower or upper 8 bit write so oldValue != value.getWholeValue()
        if (oldValue != target.getWholeValue()) {
            // consumes more flash to do it this way but we only update ram when we have something to change
            flags_ |= IsDirty;
        }
    }
    [[nodiscard]] constexpr bool isValid() const noexcept { return flags_ & IsValid ; }
    [[nodiscard]] constexpr bool isDirty() const noexcept { return flags_ & IsDirty; }
    [[nodiscard]] constexpr bool needsFlushing() const noexcept {
        return isValid() && isDirty();
    }
private:
    static constexpr byte IsDirty = 0b10;
    static constexpr byte IsValid = 0b01;
    static constexpr byte IsClean = 0;
    static constexpr byte IsInvalid = 0;
    SplitWord16 data[NumWordsCached]; // 32 bytes
    TaggedAddress tag { 0}; // 4 bytes
    byte flags_ = 0;
};
class CacheWay {
public:
    using TaggedAddress = CacheEntry::TaggedAddress;
    static constexpr auto NumberOfWays = 2;
public:
    CacheEntry& getLine(const TaggedAddress& theAddress) noexcept __attribute__((noinline));
    void clear() noexcept {
        for (auto& way : ways_) {
            way.clear();
        }
        mostRecentlyUsed_ = false;
    }
private:
    CacheEntry ways_[NumberOfWays];
    bool mostRecentlyUsed_ = false;
};
CacheEntry&
CacheWay::getLine(const TaggedAddress& theAddress) noexcept {
    static constexpr bool Way0MostRecentlyUsed = false;
    static constexpr bool Way1MostRecentlyUsed = true;
    if (ways_[0].matches(theAddress)) {
        mostRecentlyUsed_ = Way0MostRecentlyUsed; // way0 was the last used
        return ways_[0];
    } else if (ways_[1].matches(theAddress)) {
        mostRecentlyUsed_ = Way1MostRecentlyUsed; // way1 was the last used
        return ways_[1];
    } else if (!ways_[0].isValid()) {
        ways_[0].reset(theAddress);
        mostRecentlyUsed_ = Way0MostRecentlyUsed;
        return ways_[0];
    } else if (!ways_[1].isValid()) {
        ways_[1].reset(theAddress);
        mostRecentlyUsed_ = Way1MostRecentlyUsed;
        return ways_[1];
    } else if (!mostRecentlyUsed_) {
        // way1 needs to be reset
        ways_[1].reset(theAddress);
        mostRecentlyUsed_ = Way1MostRecentlyUsed;
        return ways_[1];
    } else {
        // way0 was the
        ways_[0].reset(theAddress);
        mostRecentlyUsed_ = Way0MostRecentlyUsed;
        return ways_[0];
    }
}

CacheWay entries[TargetBoard::numberOfCacheLines()];
// inlining actually causes a large amount of overhead
auto& getLine() noexcept {
    // only align if we need to reset the chip
    CacheEntry::TaggedAddress theAddress(ProcessorInterface::getAddress());
    return entries[theAddress.getTagIndex()].getLine(theAddress);
}

[[nodiscard]] bool informCPU() noexcept {
    // you must scan the BLAST_ pin before pulsing ready, the cpu will change blast for the next transaction
    auto isBurstLast = DigitalPin<i960Pinout::BLAST_>::isAsserted();
    DigitalPin<i960Pinout::Ready>::pulse();
    return isBurstLast;
}
constexpr auto IncrementAddress = true;
constexpr auto LeaveAddressAlone = false;
constexpr byte MaximumNumberOfWordsTransferrableInASingleTransaction = 8;
inline void fallbackBody() noexcept {
    // fallback, be consistent to make sure we don't run faster than the i960
    if (DigitalPin<i960Pinout::W_R_>::isAsserted()) {
        ProcessorInterface::setupDataLinesForRead();
        for (byte i = ProcessorInterface::getCacheOffsetEntry(); i < MaximumNumberOfWordsTransferrableInASingleTransaction; ++i) {
            // need to introduce some delay
            ProcessorInterface::setDataBits(0);
            if (informCPU()) {
                break;
            }
            ProcessorInterface::burstNext<LeaveAddressAlone>();
        }
    } else {
        ProcessorInterface::setupDataLinesForWrite();
        for (byte i = ProcessorInterface::getCacheOffsetEntry(); i < MaximumNumberOfWordsTransferrableInASingleTransaction; ++i) {
            // put four cycles worth of delay into this to make damn sure we are ready with the i960
            asm volatile ("nop");
            asm volatile ("nop");
            asm volatile ("nop");
            asm volatile ("nop");
            // need to introduce some delay
            if (informCPU()) {
                break;
            }
            ProcessorInterface::burstNext<LeaveAddressAlone>();
        }
    }
}
inline void invocationBody() noexcept {
    // wait until AS goes from low to high
    // then wait until the DEN state is asserted
    while (DigitalPin<i960Pinout::DEN_>::isDeasserted());
    // keep processing data requests until we
    // when we do the transition, record the information we need
    // there are only two parts to this code, either we map into ram or chipset functions
    // we can just check if we are in ram, otherwise it is considered to be chipset. This means that everything not ram is chipset
    // and so we are actually continually mirroring the mapping for the sake of simplicity
    if (auto targetSection = ProcessorInterface::newDataCycle(); OnboardPSRAMBlock::respondsTo(targetSection)) {
        // okay we are dealing with the psram chips
        // now take the time to compute the cache offset entries
        if (auto& theEntry = getLine(); DigitalPin<i960Pinout::W_R_>::isAsserted()) {
            ProcessorInterface::setupDataLinesForRead();
            // when dealing with read operations, we can actually easily unroll the do while by starting at the cache offset entry and walking
            // forward until we either hit the end of the cache line or blast is asserted first (both are valid states)
            for (byte i = ProcessorInterface::getCacheOffsetEntry(); i < MaximumNumberOfWordsTransferrableInASingleTransaction; ++i) {
                ProcessorInterface::setDataBits(theEntry.get(i));
                if (informCPU()) {
                    break;
                }
                // so if I don't increment the address, I think we run too fast xD based on some experimentation
                ProcessorInterface::burstNext<LeaveAddressAlone>();
            }
        } else {
            ProcessorInterface::setupDataLinesForWrite();
            // when dealing with writes to the cache line we are safe in just looping through from the start to at most 8 because that is as
            // far as we can go with how the Sx works!

            // Also the manual states that the processor cannot burst across 16-byte boundaries so :D.
            for (byte i = ProcessorInterface::getCacheOffsetEntry(); i < MaximumNumberOfWordsTransferrableInASingleTransaction; ++i) {
                theEntry.set(i, ProcessorInterface::getStyle(), SplitWord16{ProcessorInterface::getDataBits()});
                if (informCPU()) {
                    break;
                }
                // the manual doesn't state that the burst transaction will always have BE0 and BE1 pulled low and this is very true, you must
                // check the pins because it will do unaligned burst transactions but even that will never span multiple 16-byte entries
                // so if I don't increment the address, I think we run too fast xD based on some experimentation
                ProcessorInterface::burstNext<LeaveAddressAlone>();
            }
        }
    } else if (CoreChipsetFeatures::respondsTo(targetSection)) {
        // generally we shouldn't see burst operations here but who knows!
        // don't read lss when dealing with the chipset interface since all should be aligned to 16-bits
        if (DigitalPin<i960Pinout::W_R_>::isAsserted()) {
            ProcessorInterface::setupDataLinesForRead();
            for (byte i = ProcessorInterface::getCacheOffsetEntry(); i < MaximumNumberOfWordsTransferrableInASingleTransaction; ++i) {
                ProcessorInterface::setDataBits(CoreChipsetFeatures::read(i, ProcessorInterface::getStyle()));
                if (informCPU()) {
                    break;
                }
                // be careful of querying i960 state at this point because the chipset runs at twice the frequency of the i960
                // so you may still be reading the previous i960 cycle state!
                ProcessorInterface::burstNext<LeaveAddressAlone>();
            }
        } else {
            ProcessorInterface::setupDataLinesForWrite();
            for (byte i = ProcessorInterface::getCacheOffsetEntry(); i < MaximumNumberOfWordsTransferrableInASingleTransaction; ++i) {
                CoreChipsetFeatures::write(i, ProcessorInterface::getStyle(), ProcessorInterface::getDataBits());
                if (informCPU()) {
                    break;
                }
                // be careful of querying i960 state at this point because the chipset runs at twice the frequency of the i960
                // so you may still be reading the previous i960 cycle state!
                ProcessorInterface::burstNext<LeaveAddressAlone>();
            }
        }
    } else {
        fallbackBody();
    }
}

// the setup routine runs once when you press reset:
void setup() {
    Serial.begin(250'000);
    while(!Serial) {
        delay(10);
    }
    // before we do anything else, configure as many pins as possible and then
    // pull the i960 into a reset state, it will remain this for the entire
    // duration of the setup function
    setupPins(OUTPUT,
              i960Pinout::PSRAM_EN,
              i960Pinout::SD_EN,
              i960Pinout::Reset960,
              i960Pinout::Ready,
              i960Pinout::GPIOSelect,
              i960Pinout::SPI_OFFSET0,
              i960Pinout::SPI_OFFSET1,
              i960Pinout::SPI_OFFSET2);
    if constexpr (!attachedToIOExpander_v<i960Pinout::Reset960>) {
        pinMode(i960Pinout::Reset960, OUTPUT);
        digitalWrite<i960Pinout::Reset960, HIGH>();
    }
    if constexpr (!attachedToIOExpander_v<i960Pinout::Int0_>) {
        pinMode(i960Pinout::Int0_, OUTPUT);
        digitalWrite<i960Pinout::Int0_, HIGH>();
    }
    {
        PinAsserter<i960Pinout::Reset960> holdi960InReset;
        // all of these pins need to be pulled high
        digitalWrite<i960Pinout::PSRAM_EN, HIGH>();
        digitalWrite<i960Pinout::SD_EN, HIGH>();
        digitalWrite<i960Pinout::Ready, HIGH>();
        digitalWrite<i960Pinout::GPIOSelect, HIGH>();
        digitalWrite<i960Pinout::SPI_OFFSET0, LOW>();
        digitalWrite<i960Pinout::SPI_OFFSET1, LOW>();
        digitalWrite<i960Pinout::SPI_OFFSET2, LOW>();
        // setup the pins that could be attached to an io expander separately
        if constexpr (!attachedToIOExpander_v<i960Pinout::BA1>) { pinMode(i960Pinout::BA1, INPUT); }
        if constexpr (!attachedToIOExpander_v<i960Pinout::BA2>) { pinMode(i960Pinout::BA2, INPUT); }
        if constexpr (!attachedToIOExpander_v<i960Pinout::BA3>) { pinMode(i960Pinout::BA3, INPUT); }
        if constexpr (!attachedToIOExpander_v<i960Pinout::BE0>) { pinMode(i960Pinout::BE0, INPUT); }
        if constexpr (!attachedToIOExpander_v<i960Pinout::BE1>) { pinMode(i960Pinout::BE1, INPUT); }
        setupPins(INPUT,
                  i960Pinout::BLAST_,
                  i960Pinout::W_R_,
                  i960Pinout::DEN_,
                  i960Pinout::FAIL);
        //pinMode(i960Pinout::MISO, INPUT_PULLUP);
        SPI.begin();
        Serial.println(F("i960Sx chipset bringup"));
        // purge the cache pages
        while (!SD.begin(static_cast<int>(i960Pinout::SD_EN))) {
            Serial.println(F("SD CARD INIT FAILED...WILL RETRY SOON"));
            delay(1000);
        }
        Serial.println(F("SD CARD UP!"));
        CoreChipsetFeatures::begin();
        ProcessorInterface::begin();
        OnboardPSRAMBlock::begin();
        // okay now we need to actually open boot.system and copy it into the ramBlock
        if (!SD.exists(const_cast<char*>("boot.sys"))) {
            // delete the file and start a new
            signalHaltState(F("Could not find file \"boot.sys\"!"));
        }
        if (auto theFile = SD.open("boot.sys", FILE_READ); !theFile) {
            signalHaltState(F("Could not open \"boot.sys\"! SD CARD may be corrupt?")) ;
        } else {
            // okay we were successful in opening the file, now copy the image into psram
            Address size = theFile.size();
            static constexpr auto CacheSize = sizeof(entries);
            //static_assert(CacheSize >= (TargetBoard::cacheLineSize() * TargetBoard::numberOfCacheLines()), "The entry cache set is smaller than the requested cache size");
            // use the cache as a buffer since it won't be in use at this point in time
            auto* storage = reinterpret_cast<byte*>(entries);
            Serial.println(F("TRANSFERRING BOOT.SYS TO PSRAM"));
            for (Address addr = 0; addr < size; addr += CacheSize) {
                // do a linear read from the start to the end of storage
                // wait around to make sure we don't run afoul of the sdcard itself
                while (theFile.isBusy());
                auto numRead = theFile.read(storage, CacheSize);
                if (numRead < 0) {
                    // something wen't wrong so halt at this point
                    SD.errorHalt();
                }
                (void)OnboardPSRAMBlock::write(addr, storage, numRead);
                //Serial.print(F("."));
            }
            Serial.println(F("Transfer complete!"));
            // make sure we close the file before destruction
            theFile.close();
            for (auto& way : entries) {
                way.clear();
            }
        }
        delay(100);
        Serial.println(F("i960Sx chipset brought up fully!"));

    }
    // at this point we have started execution of the i960
    // wait until we enter self test state
    while (DigitalPin<i960Pinout::FAIL>::isDeasserted()) {
        if (DigitalPin<i960Pinout::DEN_>::isAsserted()) {
            break;
        }
    }

    // now wait until we leave self test state
    while (DigitalPin<i960Pinout::FAIL>::isAsserted()) {
        if (DigitalPin<i960Pinout::DEN_>::isAsserted()) {
            break;
        }
    }
    // at this point we are in idle so we are safe to loaf around a bit
    // at this point, the i960 will request 32-bytes to perform a boot check sum on.
    // If the checksum is successful then execution will continue as normal
    // first set of 16-byte request from memory
    invocationBody();
    invocationBody();
    if (DigitalPin<i960Pinout::FAIL>::isAsserted()) {
        signalHaltState(F("CHECKSUM FAILURE!"));
    }
    Serial.println(F("SYSTEM BOOT SUCCESSFUL!"));
}
// ----------------------------------------------------------------
// state machine
// ----------------------------------------------------------------
// The bootup process has a separate set of states
// TStart - Where we start
// TSystemTest - Processor performs self test
//
// TStart -> TSystemTest via FAIL being asserted
// TSystemTest -> Ti via FAIL being deasserted
//
// State machine will stay here for the duration
// State diagram based off of i960SA/SB Reference manual
// Basic Bus States
// Ti - Idle State
// Ta - Address State
// Td - Data State
// Tw - Wait State

// READY - ~READY asserted
// NOT READY - ~READY not asserted
// BURST - ~BLAST not asserted
// NO BURST - ~BLAST asserted
// NEW REQUEST - ~AS asserted
// NO REQUEST - ~AS not asserted when in

// Ti -> Ti via no request
// Ti -> Ta via new request
// on enter of Ta, set address state to false
// on enter of Td, burst is sampled
// Ta -> Td
// Td -> Ti after signaling ready and no burst (blast low)
// Td -> Td after signaling ready and burst (blast high)
// Ti -> TChecksumFailure if FAIL is asserted
// NOTE: Tw may turn out to be synthetic

void loop() {
    invocationBody();
}

[[noreturn]]
void
signalHaltState(const __FlashStringHelper* haltMsg) {
    Serial.println(haltMsg);
    while(true) {
        delay(1000);
    }
}



SdFat SD;
