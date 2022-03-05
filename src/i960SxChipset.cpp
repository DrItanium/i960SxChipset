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

#include "CacheEntry.h"
#include "DirectMappedCacheWay.h"
#include "TwoWayLRUCacheEntry.h"
#include "FourWayPseudoLRUEntry.h"
#include "EightWayPseudoLRUEntry.h"
#include "SixteenWayPseudoLRUEntry.h"
#include "EightWayRandPLRUEntry.h"
#include "EightWayTreePLRUEntry.h"
#include "SinglePoolCache.h"

#include "ProcessorSerializer.h"
#include "DisplayInterface.h"
#include "CoreChipsetFeatures.h"
#include "PSRAMChip.h"
#include "SDCardAsRam.h"
#include "TaggedCacheAddress.h"
#include "RTCInterface.h"
#include "i960SxChipset.h"
#include "type_traits.h"
#include "23LC1024.h"
#include "SRAMDataContainer.h"

constexpr auto RTCBaseAddress = 0xFA00'0000;
constexpr auto Serial0BaseAddress = 0xFB00'0000;
constexpr auto DisplayBaseAddress = 0xFC00'0000;
constexpr auto SDBaseAddress = 0xFD00'0000;
constexpr auto MaximumNumberOfOpenFiles = 16;
constexpr auto CompileInAddressDebuggingSupport = false;
constexpr auto AddressDebuggingEnabledOnStartup = false;
constexpr auto ValidateTransferDuringInstall = false;
/**
 * @brief When set to true, the interrupt lines the mcp23s17 provides are used to determine which bytes to read
 */
constexpr auto UseIOExpanderAddressLineInterrupts = true;
using TheDisplayInterface = DisplayInterface<DisplayBaseAddress>;
using TheSDInterface = SDCardInterface<MaximumNumberOfOpenFiles, SDBaseAddress>;
using TheConsoleInterface = Serial0Interface<Serial0BaseAddress, CompileInAddressDebuggingSupport, AddressDebuggingEnabledOnStartup>;
using TheRTCInterface = RTCInterface<RTCBaseAddress>;
using ConfigurationSpace = CoreChipsetFeatures<TheConsoleInterface,
        TheSDInterface,
        TheDisplayInterface,
        TheRTCInterface>;
// define the backing memory storage classes via template specialization
// at this point in time, if no specialization is performed, use SDCard as ram backend
using FallbackMemory = SDCardAsRam<TheSDInterface >;
template<TargetMCU mcu> struct BackingMemoryStorage final {
    using Type = FallbackMemory;
};
template<> struct BackingMemoryStorage<TargetMCU::ATmega1284p_Type1> final {
    //using ActualType = OnboardPSRAMBlock;
    //using Type = SRAMDataContainer<ActualType>;
    using Type = OnboardPSRAMBlock;
};

using BackingMemoryStorage_t = BackingMemoryStorage<TargetBoard::getMCUTarget()>::Type;
constexpr auto NumAddressBitsForPSRAMCache = 26;
constexpr auto NumAddressBits = NumAddressBitsForPSRAMCache;
constexpr auto CacheLineSize = 6;
constexpr auto CacheSize = 8192;
constexpr auto UseSpecificTypesForDifferentAddressComponents = true;

using L1Cache = CacheInstance_t<EightWayRandPLRUCacheSet, CacheSize, NumAddressBits, CacheLineSize, BackingMemoryStorage_t, UseSpecificTypesForDifferentAddressComponents>;
L1Cache theCache;

template<bool inDebugMode, bool useInterrupts>
inline void invocationBody() noexcept {
    // wait until AS goes from low to high
    // then wait until the DEN state is asserted
    while (DigitalPin<i960Pinout::DEN_>::isDeasserted());
    // keep processing data requests until we
    // when we do the transition, record the information we need
    // there are only two parts to this code, either we map into ram or chipset functions
    // we can just check if we are in ram, otherwise it is considered to be chipset. This means that everything not ram is chipset
    // and so we are actually continually mirroring the mapping for the sake of simplicity
    ProcessorInterface::newDataCycle<inDebugMode, decltype(theCache)::CacheEntry , useInterrupts>();
}
template<bool allowAddressDebuggingCodePath, bool useInterrupts>
[[gnu::always_inline]] inline void doInvocationBody() noexcept {
    if constexpr (allowAddressDebuggingCodePath) {
        if (TheConsoleInterface::addressDebuggingEnabled())  {
            invocationBody<true, useInterrupts>();
        } else {
            invocationBody<false, useInterrupts>();
        }
    } else {
        invocationBody<false, useInterrupts>();
    }
}
template<bool testFlashChips = false>
void installBootImage() noexcept {
    if constexpr (!testFlashChips) {
        // okay now we need to actually open boot.system and copy it into the ramBlock
        if (!SD.exists(const_cast<char *>("boot.sys"))) {
            // delete the file and start a new
            signalHaltState(F("Could not find file \"boot.sys\"!"));
        }
        if (auto theFile = SD.open("boot.sys", FILE_READ); !theFile) {
            signalHaltState(F("Could not open \"boot.sys\"! SD CARD may be corrupt?"));
        } else {
            // okay we were successful in opening the file, now copy the image into psram
            Address size = theFile.size();
            Serial.println(F("TRANSFERRING BOOT.SYS TO RAM"));
            static constexpr auto CacheSize = theCache.getCacheSize();
            //static constexpr auto CacheSize = ::CacheSize;
            auto *storage = theCache.viewAsStorage();
            if constexpr (ValidateTransferDuringInstall) {
                static constexpr auto RealCacheSize = CacheSize / 2;
                byte *storage0 = storage;
                byte *storage1 = storage + (RealCacheSize);
                for (Address addr = 0; addr < size; addr += RealCacheSize) {
                    // do a linear read from the start to the end of storage
                    // wait around to make sure we don't run afoul of the sdcard itself
                    while (theFile.isBusy());
                    auto numRead = theFile.read(storage0, RealCacheSize);
                    if (numRead < 0) {
                        // something wen't wrong so halt at this point
                        SD.errorHalt();
                    }
                    (void) BackingMemoryStorage_t::write(addr, storage0, numRead);
                    (void) BackingMemoryStorage_t::read(addr, storage1, numRead);
                    // now read back the contents into the second buffer
                    for (auto i = 0; i < numRead; ++i) {
                        auto a = storage0[i];
                        auto b = storage1[i];
                        if (a != b) {
                            Serial.print(F("MISMATCH WANTED 0x"));
                            Serial.print(a, HEX);
                            Serial.print(F(" BUT GOT 0x"));
                            Serial.println(b, HEX);
                        }
                    }

                    Serial.print(F("."));
                }
            } else {
                // use the cache as a buffer since it won't be in use at this point in time
                for (Address addr = 0; addr < size; addr += CacheSize) {
                    // do a linear read from the start to the end of storage
                    // wait around to make sure we don't run afoul of the sdcard itself
                    while (theFile.isBusy());
                    auto numRead = theFile.read(storage, CacheSize);
                    if (numRead < 0) {
                        // something wen't wrong so halt at this point
                        SD.errorHalt();
                    }
                    (void) BackingMemoryStorage_t::write(addr, storage, numRead);
                    // now read back the contents into the upper half
                    Serial.print(F("."));
                }
            }
            Serial.println();
            Serial.println(F("Transfer complete!"));
            // make sure we close the file before destruction
            theFile.close();
        }
    } else {
        // dump the first four megabytes of spi flash out
        digitalWrite(i960Pinout::MEMBLK0_A0, LOW);
        digitalWrite(i960Pinout::MEMBLK0_A1, LOW);
        /// @todo reimplement to copy data over to PSRAM for testing purposes
        SPI.beginTransaction(SPISettings(TargetBoard::runFlashAt(), MSBFIRST, SPI_MODE0));
        SplitWord32 container[16/sizeof(SplitWord32)];
        for (Address addr = 0; addr < 4_MB; addr += 16) {
            SplitWord32 currentAddress{addr};
            digitalWrite(i960Pinout::MEMBLK0_, LOW);
            SPI.transfer(0x03);
            SPI.transfer(currentAddress.bytes[2]);
            SPI.transfer(currentAddress.bytes[1]);
            SPI.transfer(currentAddress.bytes[0]);
            SPI.transfer(container, 16);
            digitalWrite(i960Pinout::MEMBLK0_, HIGH);
            Serial.printf(F("0x%08lX: %08lX %08lX %08lX %08lX\n"), addr, container[0], container[1], container[2], container[3]);
        }
        SPI.endTransaction();
    }
    // clear both caches to be on the safe side
    theCache.clear();
}

void setupChipsetType1() noexcept {
}

void setupChipsetVersionSpecificPins() noexcept {
    if constexpr (TargetBoard::onAtmega1284p_Type1()) {
        setupChipsetType1();
    } else {
        // do nothing
    }
}
void setupDispatchTable() noexcept;
// the setup routine runs once when you press reset:
void setup() {
    // startup SPI as soon as possible in this design
    setupPins(OUTPUT,
              i960Pinout::SPI_OFFSET0,
              i960Pinout::SPI_OFFSET1,
              i960Pinout::SPI_OFFSET2,
              i960Pinout::MEMBLK0_A0,
              i960Pinout::MEMBLK0_A1,
              i960Pinout::MEMBLK0_,
              i960Pinout::PSRAM_EN,
              i960Pinout::TFT_CS,
              i960Pinout::TFT_DC,
              i960Pinout::SD_EN,
              i960Pinout::Ready,
              i960Pinout::GPIOSelect);
    // all of these pins need to be pulled high
    digitalWrite<i960Pinout::SPI_OFFSET0, HIGH>();
    digitalWrite<i960Pinout::SPI_OFFSET1, HIGH>();
    digitalWrite<i960Pinout::SPI_OFFSET2, HIGH>();
    digitalWrite<i960Pinout::MEMBLK0_A0, HIGH>();
    digitalWrite<i960Pinout::MEMBLK0_A1, HIGH>();
    digitalWrite<i960Pinout::PSRAM_EN, HIGH>();
    digitalWrite<i960Pinout::SD_EN, HIGH>();
    digitalWrite<i960Pinout::Ready, HIGH>();
    digitalWrite<i960Pinout::GPIOSelect, HIGH>();
    digitalWrite<i960Pinout::MEMBLK0_, HIGH>();
    digitalWrite<i960Pinout::TFT_CS, HIGH>();
    // setup the pins that could be attached to an io expander separately
    setupPins(INPUT,
              i960Pinout::RAM_SPACE_,
              i960Pinout::IO_SPACE_,
              i960Pinout::BE0,
              i960Pinout::BE1,
              i960Pinout::BLAST_,
              i960Pinout::W_R_,
              i960Pinout::DEN_,
              i960Pinout::FAIL,
              i960Pinout::IOEXP_INT0,
              i960Pinout::IOEXP_INT1,
              i960Pinout::IOEXP_INT2,
              i960Pinout::IOEXP_INT3);
    SPI.begin();
    ProcessorInterface::begin();

    // always do this first to make sure that we put the i960 into reset regardless of target
    ProcessorInterface::putCPUIntoReset();
    Serial.begin(115200);
    while (!Serial) {
        delay(10);
    }
    // seed random on startup to be on the safe side from analog pin A0, A1, A2, and A3
    randomSeed(analogRead(A0) + analogRead(A1) + analogRead(A2) + analogRead(A3));
    // before we do anything else, configure as many pins as possible and then
    // pull the i960 into a reset state, it will remain this for the entire
    // duration of the setup function
    // get SPI setup ahead of time
    /// @todo pull the i960 into reset at this point
    //pinMode(i960Pinout::MISO, INPUT_PULLUP);
    theCache.begin();
    // purge the cache pages
    ConfigurationSpace::begin();
    Serial.println(F("i960Sx chipset bringup"));
    BackingMemoryStorage_t::begin();
    setupDispatchTable();
    installBootImage();
    delay(100);
    Serial.println(F("i960Sx chipset brought up fully!"));
    ProcessorInterface::pullCPUOutOfReset();
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

    // on bootup we need to ignore the interrupt lines for now
    doInvocationBody<CompileInAddressDebuggingSupport, false>();
    doInvocationBody<CompileInAddressDebuggingSupport, false>();
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
    // by default, arduino's main has a boolean check inside of the loop for the serial event run function check
    // this is expensive and something I don't use. The infinite loop is for this purpose. It shaves off around 0.1usec in the worst case
    // and doesn't seem to impact performance in burst transactions

    for (;;) {
        doInvocationBody<CompileInAddressDebuggingSupport, UseIOExpanderAddressLineInterrupts>();
    }
}

[[noreturn]]
void
signalHaltState(const __FlashStringHelper* haltMsg) {
    Serial.print(F("CHIPSET HALT: "));
    Serial.println(haltMsg);
    while(true) {
        delay(1000);
    }
}

using DispatchTable = BodyFunction[256];
extern DispatchTable lookupTableRead_Debug;
extern DispatchTable lookupTableWrite_Debug;
extern DispatchTable lookupTableRead;
extern DispatchTable lookupTableWrite;
template<typename T>
void
registerExternalDeviceWithLookupTable() noexcept {
    lookupTableRead[T::SectionID] = ProcessorInterface::performExternalDeviceRead<T, false>;
    lookupTableWrite[T::SectionID] = ProcessorInterface::performExternalDeviceWrite<T, false>;
    if constexpr (CompileInAddressDebuggingSupport) {
        lookupTableRead_Debug[T::SectionID] = ProcessorInterface::performExternalDeviceRead<T, true>;
        lookupTableWrite_Debug[T::SectionID] = ProcessorInterface::performExternalDeviceWrite<T, true>;
    }
}

template<bool debug>
void
readCacheLine() noexcept {
    ProcessorInterface::performCacheRead<typename L1Cache::CacheEntry, debug>(theCache.getLine());
}

template<bool debug>
void
writeCacheLine() noexcept {
    ProcessorInterface::performCacheWrite<typename L1Cache::CacheEntry, debug>(theCache.getLine());
}
void setupDispatchTable() noexcept {
    Serial.println(F("Setting up the initial lookup table"));
    for (int i = 0; i < 256; ++i) {
        lookupTableRead[i] = ProcessorInterface::performFallbackRead;
        lookupTableWrite[i] = ProcessorInterface::performFallbackWrite;
        if constexpr (CompileInAddressDebuggingSupport) {
            lookupTableRead_Debug[i] = ProcessorInterface::performFallbackRead;
            lookupTableWrite_Debug[i] = ProcessorInterface::performFallbackWrite;
        }
    }
    for (int i = 0;i < 4; ++i) {
        lookupTableRead[i] = readCacheLine<false>;
        lookupTableWrite[i] = writeCacheLine<false>;
        if constexpr (CompileInAddressDebuggingSupport) {
            lookupTableRead_Debug[i] = readCacheLine<true>;
            lookupTableWrite_Debug[i] = writeCacheLine<true>;
        }
    }
    registerExternalDeviceWithLookupTable<TheRTCInterface>();
    registerExternalDeviceWithLookupTable<TheDisplayInterface>();
    registerExternalDeviceWithLookupTable<TheSDInterface >();
    registerExternalDeviceWithLookupTable<TheConsoleInterface>();
    registerExternalDeviceWithLookupTable<ConfigurationSpace>();
    // now tell the ProcessorInterface to pull the appropriate functions
    ProcessorInterface::setupMostRecentDispatchFunctions();
}
BodyFunction
getDebugReadBody(byte index) noexcept {
    if constexpr (CompileInAddressDebuggingSupport) {
        return lookupTableRead_Debug[index];
    } else {
        return ProcessorInterface::performFallbackRead;
    }
}
BodyFunction
getDebugWriteBody(byte index) noexcept {
    if constexpr (CompileInAddressDebuggingSupport) {
        return lookupTableWrite_Debug[index];
    } else {
        return ProcessorInterface::performFallbackWrite;
    }
}
BodyFunction
getNonDebugReadBody(byte index) noexcept {
    return lookupTableRead[index];
}
BodyFunction getNonDebugWriteBody(byte index) noexcept {
    return lookupTableWrite[index];
}

SdFat SD;
DispatchTable lookupTableRead;
DispatchTable lookupTableRead_Debug;
DispatchTable lookupTableWrite;
DispatchTable lookupTableWrite_Debug;
