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
#include "SDCardFileSystemInterface.h"
#include "CoreChipsetFeatures.h"
#include "TFTShieldThing.h"
#include "ClockGeneration.h"
#include "PSRAMChip.h"
#ifdef ARDUINO_ARCH_RP2040
#include "hitagimon.h"
#include "OnChipMemoryThing.h"
#endif
constexpr bool EnableDebuggingCompileTime = TargetBoard::onRaspberryPiPico();

bool displayReady = false;
/**
 * @brief Describes a single cache line which associates an address with 16 bytes of storage
 */
ProcessorInterface& processorInterface = ProcessorInterface::getInterface();
// ----------------------------------------------------------------
// Load/Store routines
// ----------------------------------------------------------------




class ROMTextSection : public MemoryMappedFile {
public:
    static constexpr Address Size = 512_MB;
    static constexpr Address Mask = Size - 1;
    using Parent = MemoryMappedFile;
public:
    explicit ROMTextSection(Address base) noexcept : Parent(base, base + Size, Size, "boot.rom", FILE_READ){ }
    ~ROMTextSection() override = default;
    Address makeAddressRelative(Address input) const noexcept override {
        SplitWord32 newAddr(input);
        newAddr.bytes[3] &= 0b00011111; // We just need to factor out the offset
        return newAddr.wholeValue_;
    }
    bool
    respondsTo(Address address) const noexcept override {
        // we just need to make sure that the upper most byte has the following pattern set
        // if the upper most byte is less than 0b0010'0000 then we have a match
        return SplitWord32(address).bytes[3] < 0b0010'0000;
    }
};

/// @todo add support for the boot data section that needs to be copied into ram by the i960 on bootup
class ROMDataSection : public MemoryMappedFile {
public:
    // two clusters are held onto at a time
    static constexpr Address Size = 512_MB;
    static constexpr auto Mask = Size - 1;
    using Parent = MemoryMappedFile;
public:
    explicit ROMDataSection(Address base) noexcept : Parent(base, base + Size, Size, "boot.dat", FILE_READ) { }
    ~ROMDataSection() override = default;
    Address makeAddressRelative(Address input) const noexcept override {
        /// @todo test the assumption that we can just return the input address since this will only fire if we invoke the proper mapped thing
        SplitWord32 newAddr(input);
        newAddr.bytes[3] &= 0b00011111; // We just need to factor out the offset
        return newAddr.wholeValue_;
    }
    bool
    respondsTo(Address address) const noexcept override {
        auto targetByte = (SplitWord32(address).bytes[3]);
        return (targetByte < 0b0100'0000) && (targetByte >= 0b0010'0000);
    }
};

CoreChipsetFeatures chipsetFunctions(0);
DisplayThing displayCommandSet(0x200);
//constexpr Address RAMStart = 0x8000'0000;
constexpr auto PerformPSRAMSanityCheck = !TargetBoard::onAtmega1284p();
// the boot.sys image must be 64 megs in size
constexpr auto PerformClearOnBootup = false; // clear it out first and then later on we will copy into it
using OnboardMemoryBlock = OnboardPSRAMBlock<PerformClearOnBootup, PerformPSRAMSanityCheck>;
OnboardMemoryBlock ramBlock(0);
FallbackMemoryThing& fallback = FallbackMemoryThing::getFallback();
SDCardFilesystemInterface fs(0x300);




// ----------------------------------------------------------------
// setup routines
// ----------------------------------------------------------------

// we only have a single 128kb cache chip
class CacheEntry {
public:
    static constexpr size_t NumBytesCached = TargetBoard::cacheLineSize();
    static constexpr size_t NumWordsCached = NumBytesCached / sizeof(SplitWord16);
    static constexpr size_t LowestBitCount = 5;
    static constexpr size_t TagIndexSize = 8;
    static constexpr size_t UpperBitCount = 32 - (LowestBitCount + TagIndexSize);
    static_assert((LowestBitCount + TagIndexSize + UpperBitCount) == 32, "TaggedAddress must map exactly to a 32-bit address");
    static constexpr size_t ActualCacheEntrySize = NumBytesCached + (2*sizeof(bool)) + sizeof(void*) + sizeof(Address);
    static constexpr auto SramCacheSize = 128_KB;
    static constexpr auto SramCacheEntrySize = NumBytesCached * 2; // we need to waste a bunch of space in this design but it will help with locality
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
    static constexpr Address computeL2TagIndex(Address address) noexcept {
        // we don't care about the upper most bit because the SRAM cache isn't large enough
        SplitWord32 tmp(address);
        tmp.bytes[0] &= TagMask;
        return tmp.wholeValue_ << 1;
    }
public:
    CacheEntry() noexcept { };
    [[nodiscard]] constexpr bool valid() const noexcept { return valid_; }
    [[nodiscard]] constexpr bool isDirty() const noexcept { return dirty_; }
    static void invalidateAllEntries() noexcept {
        if constexpr (CacheActive_v) {
            for (uint32_t i = 0; i < SramCacheSize; i += SramCacheEntrySize) {
                CacheEntry target(i); // load from the cache and purge the hell out of it
                target.invalidate(); // try and invalidate it as well
            }
        }
    }
private:
    /**
     * @brief Construct a cache entry from the SRAM cache
     * @param tag The address to use to pull from the SRAM cache
     */
    explicit CacheEntry(Address tag) noexcept {
        absorbEntryFromSRAMCache(tag);
    }
    void absorbEntryFromSRAMCache(Address newTag) noexcept {
        if constexpr (CacheActive_v) {
            SplitWord32 translation(computeL2TagIndex(newTag));
            digitalWrite<i960Pinout::CACHE_EN_, LOW>();
            SPI.transfer(0x03);
            SPI.transfer(translation.bytes[2]);
            SPI.transfer(translation.bytes[1]);
            SPI.transfer(translation.bytes[0]); // aligned to 32-byte boundaries
            SPI.transfer(backingStorage, ActualCacheEntrySize);
            digitalWrite<i960Pinout::CACHE_EN_, HIGH>();
            // and we are done!
        }
    }
private:
    void commitToSRAM() noexcept {
        if constexpr (CacheActive_v) {
            SplitWord32 translation(computeL2TagIndex(tag));
            digitalWrite<i960Pinout::CACHE_EN_, LOW>();
            SPI.transfer(0x02);
            SPI.transfer(translation.bytes[2]);
            SPI.transfer(translation.bytes[1]);
            SPI.transfer(translation.bytes[0]); // aligned to 32-byte boundaries
            SPI.transfer(backingStorage, ActualCacheEntrySize); // this will garbage out things by design
            digitalWrite<i960Pinout::CACHE_EN_, HIGH>();
        }
    }
public:
    void reset(Address newTag, MemoryThing& thing) noexcept {
        if constexpr (CacheActive_v) {
            // commit what we currently have in this object to sram cache (this could be invalid but it is important!)
            // at no point can the cache ever be manipulated outside of this method
            commitToSRAM();
            // pull the new tag's address out of sram and do some work with it
            absorbEntryFromSRAMCache(newTag);
            if (matches(newTag)) {
                // we got a match to just return
                return;
            }
        }
        // no match so pull the data in from main memory
        if (valid() && isDirty()) {
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
    void invalidate() noexcept {
        if (valid() && isDirty()) {
            backingThing->write(tag, reinterpret_cast<byte*>(data), sizeof(data));
            valid_ = false;
            dirty_ = false;
            tag = 0;
            backingThing = nullptr;
            //unused = 0; // make sure that this is correctly purged
        }
    }
    [[nodiscard]] constexpr bool matches(Address addr) const noexcept { return valid() && (tag == addr); }
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
                break;
        }
    }
private:
    union {
        // align to 40-bytes, this does waste some space on l2 cache
        // (we have to make the entries 64-bytes in size for simple alignment)
        byte backingStorage[ActualCacheEntrySize] = { 0 };
        struct {
            bool valid_;
            bool dirty_;
            MemoryThing* backingThing; // 2 bytes
            Address tag; // 4 bytes
            SplitWord16 data[NumWordsCached]; // 32 bytes
        } PACKED_ATTRIBUTE ;
    };
};
static_assert(sizeof(CacheEntry) == CacheEntry::ActualCacheEntrySize);
CacheEntry entries[TargetBoard::numberOfCacheLines()]; // we actually are holding more bytes in the cache than before
// we have a second level cache of 1 megabyte in sram over spi
void invalidateGlobalCache() noexcept {
    // commit all entries back
    // walk through the SRAMCache and commit any entries which are empty
    for (auto& entry : entries) {
        entry.invalidate();
    }
    CacheEntry::invalidateAllEntries();
}
auto& getLine(MemoryThing& theThing) noexcept {
    if constexpr (EnableDebuggingCompileTime) {
        Serial.println(F("getLine() {"));
    }
    auto address = processorInterface.getAlignedAddress();
    auto tagIndex = CacheEntry::computeTagIndex(address);
    if constexpr (EnableDebuggingCompileTime) {
        Serial.print(F("ADDRESS: 0x"));
        Serial.println(address, HEX);
        Serial.print(F("TAG INDEX: 0x"));
        Serial.println(tagIndex, HEX);
    }
    auto& theEntry = entries[tagIndex];
    if (!theEntry.matches(address)) {
        theEntry.reset(address, theThing);
    }
    if constexpr (EnableDebuggingCompileTime) {
        Serial.println(F("}"));
    }
    return theEntry;
}

/**
 * @brief Just in case, purge the sram of data
 */
void purgeSRAMCache() noexcept {
    if constexpr (CacheActive_v) {
        // 23lc1024s are in sequential by default :)
        digitalWrite<i960Pinout::CACHE_EN_, LOW>();
        SPI.transfer(0xFF);
        digitalWrite<i960Pinout::CACHE_EN_, HIGH>();
        constexpr uint32_t max = 128_KB; // only one SRAM chip on board
        Serial.println(F("CHECKING SRAM IS PROPERLY WRITABLE"));
        for (uint32_t i = 0; i < max; i += 32) {
            SplitWord32 translation(i);
            byte pagePurgeInstruction[36]{
                    0x02,
                    translation.bytes[2],
                    translation.bytes[1],
                    translation.bytes[0],
                    0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
                    0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
                    0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
                    0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
            };
            byte pageReadInstruction[36]{
                    0x03,
                    translation.bytes[2],
                    translation.bytes[1],
                    translation.bytes[0],
                    0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0,
            };
            digitalWrite<i960Pinout::CACHE_EN_, LOW>();
            SPI.transfer(pagePurgeInstruction, 36);
            digitalWrite<i960Pinout::CACHE_EN_, HIGH>();
            digitalWrite<i960Pinout::CACHE_EN_, LOW>();
            SPI.transfer(pageReadInstruction, 36);
            digitalWrite<i960Pinout::CACHE_EN_, HIGH>();
            for (int x = 4; x < 36; ++x) {
                if (pageReadInstruction[x] != 0x55) {
                    Serial.print(F("MISMATCH EXPECTED 0x55 BUT GOT 0x"));
                    Serial.println(pageReadInstruction[x], HEX);
                    signalHaltState(F("SRAM CACHE MISMATCH!!!"));
                }
            }
        }
        Serial.println(F("SUCCESSFULLY CHECKED SRAM CACHE!"));
        Serial.println(F("PURGING SRAM CACHE!"));
        for (uint32_t i = 0; i < max; i+= 32) {
            SplitWord32 translation(i);
            byte pagePurgeInstruction[36] {
                    0x02,
                    translation.bytes[2],
                    translation.bytes[1],
                    translation.bytes[0],
                    0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0,
            };
            byte pageReadInstruction[36]{
                    0x03,
                    translation.bytes[2],
                    translation.bytes[1],
                    translation.bytes[0],
                    0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0,
            };
            digitalWrite<i960Pinout::CACHE_EN_, LOW>();
            SPI.transfer(pagePurgeInstruction, 36);
            digitalWrite<i960Pinout::CACHE_EN_, HIGH>();
            digitalWrite<i960Pinout::CACHE_EN_, LOW>();
            SPI.transfer(pageReadInstruction, 36);
            digitalWrite<i960Pinout::CACHE_EN_, HIGH>();
            for (int x = 4; x < 36; ++x) {
                if (pageReadInstruction[x] != 0) {
                    signalHaltState(F("SRAM CACHE FAILURE!!!"));
                }
            }
        }
        Serial.println(F("DONE PURGING SRAM CACHE!"));
    }
}
// the setup routine runs once when you press reset:
void setup() {
    setupClockSource();
    Serial.begin(115200);
    while(!Serial) {
        delay(10);
    }
    if constexpr (TargetBoard::onRaspberryPiPico()) {
        delay(2000);
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
    if constexpr (DisplayActive_v) {
        pinMode(i960Pinout::DISPLAY_EN, OUTPUT);
        digitalWrite<i960Pinout::DISPLAY_EN, HIGH>();
    }
    if constexpr (!attachedToIOExpander(i960Pinout::Reset960)) {
        pinMode(i960Pinout::Reset960, OUTPUT);
        digitalWrite<i960Pinout::Reset960, HIGH>();
    }
    if constexpr (!attachedToIOExpander(i960Pinout::Int0_)) {
        pinMode(i960Pinout::Int0_, OUTPUT);
        digitalWrite<i960Pinout::Int0_, HIGH>();
    }
    if constexpr (CacheActive_v) {
        pinMode(i960Pinout::CACHE_EN_, OUTPUT);
        digitalWrite<i960Pinout::CACHE_EN_, HIGH>();
    }
    {
        PinAsserter<i960Pinout::Reset960> holdi960InReset;
        // all of these pins need to be pulled high
        digitalWriteBlock(HIGH,
                          i960Pinout::PSRAM_EN,
                          i960Pinout::SD_EN,
                          i960Pinout::DISPLAY_EN,
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
        purgeSRAMCache();
        Serial.println(F("i960Sx chipset bringup"));
        // purge the cache pages
        fs.begin();
        chipsetFunctions.begin();
        processorInterface.begin();
        displayCommandSet.begin();
        displayReady = true;
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
            static constexpr auto CacheSize = 1024;
            byte storage[CacheSize] = { 0 };
            Serial.println(F("TRANSFERRING BOOT.SYS TO PSRAM"));
            for (Address addr = 0; addr < size; addr += CacheSize) {
                // do a linear read from the start to the end of storage
                auto numRead = theFile.read(storage, CacheSize);
                (void)ramBlock.write(addr, storage, numRead);
                // wait around to make sure we don't run afoul of the sdcard itself
                while (theFile.isBusy());
            }
            Serial.println(F("Transfer complete!"));
            // make sure we close the file before destruction
            theFile.close();
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
    while (DigitalPin<i960Pinout::DEN_>::isDeasserted());

    auto rom = processorInterface.newDataCycle();
    // we know that this will always get mapped to rom so no need to look this up constantly
    if (rom->bypassesCache()) {
        // we also know that this will be a read operation at this point
        do {
            processorInterface.setDataBits(rom->read(processorInterface.getAddress(), processorInterface.getStyle()));
            auto isBurstLast = DigitalPin<i960Pinout::BLAST_>::isAsserted();
            DigitalPin<i960Pinout::Ready>::pulse();
            if (isBurstLast) {
                break;
            }
            processorInterface.burstNext();
        } while (true);
    } else {
        auto& theEntry = getLine(*rom);
        do {
            processorInterface.setDataBits(theEntry.get(processorInterface.getCacheOffsetEntry()).getWholeValue());
            auto isBurstLast = DigitalPin<i960Pinout::BLAST_>::isAsserted();
            DigitalPin<i960Pinout::Ready>::pulse();
            if (isBurstLast) {
                break;
            }
            processorInterface.burstNext();
        } while (true);
    }
    // then do the second 16-byte request
    while (DigitalPin<i960Pinout::DEN_>::isDeasserted());
    rom = processorInterface.newDataCycle();
    // we know that this will always get mapped to rom so no need to look this up constantly
    if (rom->bypassesCache()) {
        // we also know that this will be a read operation at this point
        do {
            processorInterface.setDataBits(rom->read(processorInterface.getAddress(), processorInterface.getStyle()));
            auto isBurstLast = DigitalPin<i960Pinout::BLAST_>::isAsserted();
            DigitalPin<i960Pinout::Ready>::pulse();
            if (isBurstLast) {
                break;
            }
            processorInterface.burstNext();
        } while (true);
    } else {
        auto& theEntry = getLine(*rom);
        do {
            processorInterface.setDataBits(theEntry.get(processorInterface.getCacheOffsetEntry()).getWholeValue());
            auto isBurstLast = DigitalPin<i960Pinout::BLAST_>::isAsserted();
            DigitalPin<i960Pinout::Ready>::pulse();
            if (isBurstLast) {
                break;
            }
            processorInterface.burstNext();
        } while (true);
    }
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
    // wait until AS goes from low to high
    // then wait until the DEN state is asserted
    while (DigitalPin<i960Pinout::DEN_>::isDeasserted());
    // keep processing data requests until we
    // when we do the transition, record the information we need
    if (auto theThing = processorInterface.newDataCycle(); theThing->bypassesCache()) {
        if (DigitalPin<i960Pinout::W_R_>::isAsserted()) {
            do {
                processorInterface.setDataBits(theThing->read(processorInterface.getAddress(), processorInterface.getStyle()));
                auto isBurstLast = DigitalPin<i960Pinout::BLAST_>::isAsserted();
                DigitalPin<i960Pinout::Ready>::pulse();
                if (isBurstLast) {
                    break;
                }
                processorInterface.burstNext();
            } while (true);
        } else {
            do {
                theThing->write(processorInterface.getAddress(), processorInterface.getDataBits(), processorInterface.getStyle());
                auto isBurstLast = DigitalPin<i960Pinout::BLAST_>::isAsserted();
                DigitalPin<i960Pinout::Ready>::pulse();
                if (isBurstLast) {
                    break;
                }
                processorInterface.burstNext();
            } while (true);
        }
    } else {
        if (auto& theEntry = getLine(*theThing); DigitalPin<i960Pinout::W_R_>::isAsserted()) {
            do {
                processorInterface.setDataBits(theEntry.get(processorInterface.getCacheOffsetEntry()).getWholeValue());
                auto isBurstLast = DigitalPin<i960Pinout::BLAST_>::isAsserted();
                DigitalPin<i960Pinout::Ready>::pulse();
                if (isBurstLast) {
                    break;
                }
                processorInterface.burstNext();
            } while (true);
        } else {
            do {
                theEntry.set(processorInterface.getCacheOffsetEntry(),
                             processorInterface.getStyle(),
                             SplitWord16{processorInterface.getDataBits()});
                auto isBurstLast = DigitalPin<i960Pinout::BLAST_>::isAsserted();
                DigitalPin<i960Pinout::Ready>::pulse();
                if (isBurstLast) {
                    break;
                }
                processorInterface.burstNext();
            } while (true);
        }
    }
}

[[noreturn]]
void
signalHaltState(const __FlashStringHelper* haltMsg) {
    if (displayReady) {
        displayCommandSet.clearScreen();
        displayCommandSet.setCursor(0, 0);
        displayCommandSet.setTextSize(2);
        displayCommandSet.println(haltMsg);
    }
    Serial.println(haltMsg);
    while(true) {
        delay(1000);
    }
}

[[noreturn]]
void
signalHaltState(const char* haltMsg) {
    if (displayReady) {
        displayCommandSet.clearScreen();
        displayCommandSet.setCursor(0, 0);
        displayCommandSet.setTextSize(2);
        displayCommandSet.println(haltMsg);
    }
    Serial.println(haltMsg);
    while(true) {
        delay(1000);
    }

}
MemoryThing* memoryMapping[256] {
    // 512 megabytes (map the first 64 megabytes and nothing else until we get to the upper most area)
   &ramBlock, &ramBlock, &ramBlock, &ramBlock, &fallback, &fallback, &fallback, &fallback,
   &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback,
   &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback,
   &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback,
  // 512 megs
   &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback,
   &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback,
   &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback,
   &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback,
        // 512 megs
   &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback,
   &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback,
   &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback,
   &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback,
        // 512 megs
   &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback,
   &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback,
   &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback,
   &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback,
   // 512 megabytes
   &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback,
   &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback,
   &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback,
   &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback,
        // 512 megabytes
   &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback,
   &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback,
   &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback,
   &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback,
        // 512 megabytes
   &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback,
   &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback,
   &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback,
   &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback,
        // 512 megabytes
   &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback,
   &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback,
   &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback,
   &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, nullptr, &fallback,
};
MemoryThing* ioSpaceSimpleMapping[256] {
        &chipsetFunctions, &fallback, &displayCommandSet, &fs, &fallback, &fallback, &fallback, &fallback,
        &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback,
        &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback,
        &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback,
        &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback,
        &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback,
        &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback,
        &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback,
        &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback,
        &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback,
        &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback,
        &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback,
        &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback,
        &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback,
        &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback,
        &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback,
        &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback,
        &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback,
        &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback,
        &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback,
        &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback,
        &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback,
        &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback,
        &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback,
        &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback,
        &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback,
        &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback,
        &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback,
        &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback,
        &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback,
        &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback,
        &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback, &fallback,
    };

inline MemoryThing*
getThingFromMSB(byte index) noexcept {
    return memoryMapping[index];
}

inline MemoryThing*
getIOSpaceDevice(byte index) noexcept {
    return ioSpaceSimpleMapping[index];
}
MemoryThing*
getThing(Address address) noexcept {
    SplitWord32 decomposedAddress(address);
    if (auto mapping = getThingFromMSB(decomposedAddress.bytes[3]); mapping == nullptr) {
        return getIOSpaceDevice(decomposedAddress.bytes[1]);
    } else {
        return mapping;
    }
}
SdFat SD;
