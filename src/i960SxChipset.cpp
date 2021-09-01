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

ProcessorInterface& processorInterface = ProcessorInterface::getInterface();
// the core devices
CoreChipsetFeatures chipsetFunctions(0);
OnboardPSRAMBlock ramBlock(0);
FallbackMemoryThing& fallback = FallbackMemoryThing::getFallback();

/**
 * @brief Describes a single cache line which associates an address with 32 bytes of storage
 */
class CacheEntry {
public:
    static constexpr size_t NumBytesCached = TargetBoard::cacheLineSize();
    static constexpr size_t NumWordsCached = NumBytesCached / sizeof(SplitWord16);
    static constexpr size_t LowestBitCount = 5;
    static constexpr size_t TagIndexSize = 8;
    static constexpr size_t UpperBitCount = 32 - (LowestBitCount + TagIndexSize);
    static_assert((LowestBitCount + TagIndexSize + UpperBitCount) == 32, "TaggedAddress must map exactly to a 32-bit address");
    static constexpr byte TagMask = static_cast<byte>(0xFF << LowestBitCount); // exploit shift beyond
    static constexpr byte OffsetMask = static_cast<byte>(~TagMask) >> 1;  // remember that this 16-bit aligned

    union TaggedAddress {
        constexpr explicit TaggedAddress(Address value = 0)  noexcept : base(value) { }
        [[nodiscard]] constexpr auto getTagIndex() const noexcept { return tagIndex; }
        [[nodiscard]] constexpr auto getAddress() const noexcept { return base; }
        [[nodiscard]] constexpr auto getLowest() const noexcept { return lowest; }
        [[nodiscard]] constexpr auto getRest() const noexcept { return rest; }
    private:
        Address base;
        struct {
            Address lowest : LowestBitCount;
            Address tagIndex : TagIndexSize;
            Address rest : UpperBitCount;
        };
    };
    static constexpr auto computeTagIndex(Address address) noexcept {
        return TaggedAddress(address).getTagIndex();
    }
public:
    void reset(Address newTag, MemoryThing& thing) noexcept {
        // no match so pull the data in from main memory
        if (valid_ && dirty_) {
            // just do the write out to disk to save time
            // still an expensive operation
            backingThing->write(tag, reinterpret_cast<byte*>(data), sizeof(data));
        }
        valid_ = true; // always set this
        dirty_ = false;
        tag = newTag;
        backingThing = &thing;
        // this is a _very_ expensive operation
        thing.read(tag, reinterpret_cast<byte*>(data), sizeof (data));
    }
    /**
     * @brief Clear the entry without saving what was previously in it, necessary if the memory was reused for a different purpose
     */
    void clear() noexcept {
        // clear all flags
        flagsRaw_ = 0;
        tag = 0;
        backingThing = nullptr;
        for (auto& a : data) {
            a.wholeValue_ = 0;
        }
    }
    [[nodiscard]] constexpr bool matches(Address addr) const noexcept { return valid_ && (tag == addr); }
    [[nodiscard]] const SplitWord16& get(byte offset) const noexcept { return data[offset & OffsetMask]; }
    void set(byte offset, LoadStoreStyle style, SplitWord16 value) noexcept {
        dirty_ = true;
        switch (auto& target = data[offset & OffsetMask]; style) {
            case LoadStoreStyle::Full16:
                target.wholeValue_ = value.wholeValue_;
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
    }
private:
    SplitWord16 data[NumWordsCached]; // 32 bytes
    Address tag; // 4 bytes
    MemoryThing* backingThing; // 2 bytes
    union {
        // save precious sram by smashing flags into a single byte
        byte flagsRaw_ = 0;
        bool valid_ : 1;
        bool dirty_ : 1;
        // remaining 6 bits unused
    };
};

CacheEntry entries[TargetBoard::numberOfCacheLines()]; // we actually are holding more bytes in the cache than before
// we have a second level cache of 1 megabyte in sram over spi
auto& getLine(MemoryThing& theThing) noexcept {
    auto address = processorInterface.getAlignedAddress();
    auto& theEntry = entries[CacheEntry::computeTagIndex(address)];
    if (!theEntry.matches(address)) {
        theEntry.reset(address, theThing);
    }
    return theEntry;
}

inline bool informCPU() noexcept {
    // you must scan the BLAST_ pin before pulsing ready, the cpu will change blast for the next transaction
    auto isBurstLast = DigitalPin<i960Pinout::BLAST_>::isAsserted();
    DigitalPin<i960Pinout::Ready>::pulse();
    return isBurstLast;
}
inline void invocationBody() noexcept {
    // wait until AS goes from low to high
    // then wait until the DEN state is asserted
    while (DigitalPin<i960Pinout::DEN_>::isDeasserted());
    // keep processing data requests until we
    // when we do the transition, record the information we need
    auto isReadOperation = DigitalPin<i960Pinout::W_R_>::isAsserted();
    if (auto targetDevice = processorInterface.newDataCycle(); targetDevice < 4) {
        // okay we are dealing with the psram chips
        auto& theEntry = getLine(ramBlock);
        do {
            if (isReadOperation) {
                processorInterface.setDataBits(theEntry.get(processorInterface.getCacheOffsetEntry()).getWholeValue());
            } else {
                theEntry.set(processorInterface.getCacheOffsetEntry(),
                             processorInterface.getStyle(),
                             SplitWord16{processorInterface.getDataBits()});
            }
            if (informCPU()) {
                break;
            }
            processorInterface.burstNext();
        } while (true);
    } else if (targetDevice == 0xFE) {
        // generally we shouldn't see burst operations here but who knows!
        do {
            if (isReadOperation) {
                processorInterface.setDataBits(chipsetFunctions.read(processorInterface.getAddress(),
                                                                     processorInterface.getStyle()));
            } else {
                chipsetFunctions.write(processorInterface.getAddress(), processorInterface.getDataBits(),
                                       processorInterface.getStyle());
            }
            if (informCPU()) {
                break;
            }
            processorInterface.burstNext();
        } while (true);
    } else {
        // fallback case
        // do it once at the beginning and never again
        if (isReadOperation) {
            processorInterface.setDataBits(0);
        }
        do {
            // on writes we do nothing but throw the value on
            // on reads we just keep the latch at 0 so we don't change a thing
            // after the first operation
            if (informCPU()) {
                break;
            }
            processorInterface.burstNext();
        } while (true);
    }
}

// the setup routine runs once when you press reset:
void setup() {
    Serial.begin(115200);
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
    if constexpr (!attachedToIOExpander(i960Pinout::Reset960)) {
        pinMode(i960Pinout::Reset960, OUTPUT);
        digitalWrite<i960Pinout::Reset960, HIGH>();
    }
    if constexpr (!attachedToIOExpander(i960Pinout::Int0_)) {
        pinMode(i960Pinout::Int0_, OUTPUT);
        digitalWrite<i960Pinout::Int0_, HIGH>();
    }
    {
        PinAsserter<i960Pinout::Reset960> holdi960InReset;
        // all of these pins need to be pulled high
        digitalWriteBlock(HIGH,
                          i960Pinout::PSRAM_EN,
                          i960Pinout::SD_EN,
                          i960Pinout::Ready,
                          i960Pinout::GPIOSelect);
        digitalWriteBlock(LOW,
                          i960Pinout::SPI_OFFSET0,
                          i960Pinout::SPI_OFFSET1,
                          i960Pinout::SPI_OFFSET2);
        // setup the pins that could be attached to an io expander separately
        if constexpr (!attachedToIOExpander(i960Pinout::BA1)) { pinMode(i960Pinout::BA1, INPUT); }
        if constexpr (!attachedToIOExpander(i960Pinout::BA2)) { pinMode(i960Pinout::BA2, INPUT); }
        if constexpr (!attachedToIOExpander(i960Pinout::BA3)) { pinMode(i960Pinout::BA3, INPUT); }
        if constexpr (!attachedToIOExpander(i960Pinout::BE0)) { pinMode(i960Pinout::BE0, INPUT); }
        if constexpr (!attachedToIOExpander(i960Pinout::BE1)) { pinMode(i960Pinout::BE1, INPUT); }
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
        chipsetFunctions.begin();
        processorInterface.begin();
        ramBlock.begin();
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
                (void)ramBlock.write(addr, storage, numRead);
                //Serial.print(F("."));
            }
            Serial.println(F("Transfer complete!"));
            // make sure we close the file before destruction
            theFile.close();
            Serial.println(F("CLEARING CACHE"));
            for (auto& entry : entries) {
                // the cache will be filled with transfer garbage so just clear it out without caring what it's contents are
                entry.clear();
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

[[noreturn]]
void
signalHaltState(const char* haltMsg) {
    Serial.println(haltMsg);
    while(true) {
        delay(1000);
    }

}


SdFat SD;
