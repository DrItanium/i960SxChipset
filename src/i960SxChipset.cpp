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
#include "MultiCache.h"

#include "ProcessorSerializer.h"
#include "DisplayInterface.h"
#include "CoreChipsetFeatures.h"
#include "PSRAMChip.h"
#include "SDCardAsRam.h"
#include "TaggedCacheAddress.h"
#include "RTCInterface.h"
#include "i960SxChipset.h"
#include "type_traits.h"

constexpr auto RTCBaseAddress = 0xFA00'0000;
constexpr auto Serial0BaseAddress = 0xFB00'0000;
constexpr auto DisplayBaseAddress = 0xFC00'0000;
constexpr auto SDBaseAddress = 0xFD00'0000;
constexpr auto MaximumNumberOfOpenFiles = 16;
constexpr auto CompileInAddressDebuggingSupport = false;
constexpr auto AddressDebuggingEnabledOnStartup = false;
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
template<TargetMCU mcu> struct BackingMemoryStorage final { using Type = SDCardAsRam<TheSDInterface>; };
template<> struct BackingMemoryStorage<TargetMCU::ATmega1284p_Type1> final { using Type = OnboardPSRAMBlock; };

using BackingMemoryStorage_t = BackingMemoryStorage<TargetBoard::getMCUTarget()>::Type;

//using OnboardPSRAMBlock = ::
constexpr auto NumAddressBitsForPSRAMCache = 26;
constexpr auto NumAddressBits = NumAddressBitsForPSRAMCache;
constexpr auto CacheLineSize = 6;
constexpr auto CacheSize = 8192;
//using L1Cache = CacheInstance_t<EightWayTreePLRUCacheSet, CacheSize, NumAddressBits, CacheLineSize, BackingMemoryStorage_t>;
//using L1Cache = CacheInstance_t<EightWayLRUCacheWay, CacheSize, NumAddressBits, CacheLineSize, BackingMemoryStorage_t>;
using L1Cache = CacheInstance_t<EightWayRandPLRUCacheSet, CacheSize, NumAddressBits, CacheLineSize, BackingMemoryStorage_t>;
L1Cache theCache;

//template<template<auto, auto, auto, typename> typename L,
//        byte NumberOfCaches,
//        uint16_t IndividualCacheSize,
//        byte CacheLineSize>
//using L1Cache = MultiCache<L, NumberOfCaches, IndividualCacheSize, NumAddressBits, CacheLineSize, BackingMemoryStorage_t>;
//L1Cache<DirectMappedCacheWay, 11, 1024, 6> theCache;



[[nodiscard]] inline bool informCPU() noexcept {
    // you must scan the BLAST_ pin before pulsing ready, the cpu will change blast for the next transaction
    auto isBurstLast = DigitalPin<i960Pinout::BLAST_>::isAsserted();
    pulse<i960Pinout::Ready>();
    return isBurstLast;
}
constexpr auto IncrementAddress = true;
constexpr auto LeaveAddressAlone = false;
// while the i960 does not allow going beyond 8 words, we can use the number of words cached in all cases to be safe
constexpr byte MaximumNumberOfWordsTransferrableInASingleTransaction = decltype(theCache)::NumWordsCached;
inline void displayRequestedAddress() noexcept {
    auto address = ProcessorInterface::getAddress();
    Serial.print(F("ADDRESS: 0x"));
    Serial.println(address, HEX);
}

template<bool inDebugMode>
inline void fallbackBody() noexcept {
    if constexpr (inDebugMode) {
        displayRequestedAddress();
    }
    // fallback, be consistent to make sure we don't run faster than the i960
    if (ProcessorInterface::isReadOperation()) {
        ProcessorInterface::setupDataLinesForRead();
        for (;;) {
            // need to introduce some delay
            ProcessorInterface::setDataBits(0);
            if (informCPU()) {
                break;
            }
            ProcessorInterface::burstNext<LeaveAddressAlone>();
        }
    } else {
        ProcessorInterface::setupDataLinesForWrite();
        for (;;) {
            // put four cycles worth of delay into this to make damn sure we are ready with the i960
            __builtin_avr_nops(4);
            // need to introduce some delay
            if (informCPU()) {
                break;
            }
            ProcessorInterface::burstNext<LeaveAddressAlone>();
        }
    }
}

template<bool inDebugMode>
inline void handleMemoryInterface() noexcept {
    if constexpr (inDebugMode) {
        displayRequestedAddress();
    }
    // okay we are dealing with the psram chips
    // now take the time to compute the cache offset entries
    if (auto& theEntry = theCache.getLine(); ProcessorInterface::isReadOperation()) {
        ProcessorInterface::setupDataLinesForRead();
        // when dealing with read operations, we can actually easily unroll the do while by starting at the cache offset entry and walking
        // forward until we either hit the end of the cache line or blast is asserted first (both are valid states)
        for (byte i = ProcessorInterface::getCacheOffsetEntry(); i < MaximumNumberOfWordsTransferrableInASingleTransaction; ++i) {
            auto outcome = theEntry.get(i);
            if constexpr (inDebugMode) {
                Serial.print(F("\tOffset: 0x")) ;
                Serial.println(i, HEX);
                Serial.print(F("\tRead the value: 0x"));
                Serial.println(outcome, HEX);
            }
            // Only pay for what we need even if it is slower
            ProcessorInterface::setDataBits(outcome);
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
            auto bits = ProcessorInterface::getDataBits();
            if constexpr (inDebugMode) {
                Serial.print(F("\tOffset: 0x")) ;
                Serial.println(i, HEX);
                Serial.print(F("\tWriting the value: 0x"));
                Serial.println(bits.getWholeValue(), HEX);
            }
            theEntry.set(i, ProcessorInterface::getStyle(), bits);
            if (informCPU()) {
                break;
            }
            // the manual doesn't state that the burst transaction will always have BE0 and BE1 pulled low and this is very true, you must
            // check the pins because it will do unaligned burst transactions but even that will never span multiple 16-byte entries
            // so if I don't increment the address, I think we run too fast xD based on some experimentation
            ProcessorInterface::burstNext<LeaveAddressAlone>();
        }
    }
}

template<bool inDebugMode, typename T>
inline void handleExternalDeviceRequest() noexcept {
    if constexpr (inDebugMode) {
        displayRequestedAddress();
    }
    // with burst transactions in the core chipset, we do not have access to a cache line to write into.
    // instead we need to do the old style infinite iteration design
    if (ProcessorInterface::isReadOperation()) {
        ProcessorInterface::setupDataLinesForRead();
        for(;;) {
            auto result = T::read(ProcessorInterface::getPageIndex(),
                                  ProcessorInterface::getPageOffset(),
                                  ProcessorInterface::getStyle());
            if constexpr (inDebugMode) {
                Serial.print(F("\tPage Index: 0x")) ;
                Serial.println(ProcessorInterface::getPageIndex(), HEX);
                Serial.print(F("\tPage Offset: 0x")) ;
                Serial.println(ProcessorInterface::getPageOffset(), HEX);
                Serial.print(F("\tRead Value: 0x"));
                Serial.println(result, HEX);
            }
            ProcessorInterface::setDataBits(result);
            if (informCPU()) {
                break;
            }
            ProcessorInterface::burstNext<IncrementAddress>();
        }
    } else {
        ProcessorInterface::setupDataLinesForWrite();
        for (;;) {
            auto dataBits = ProcessorInterface::getDataBits();
            if constexpr (inDebugMode) {
                Serial.print(F("\tPage Index: 0x")) ;
                Serial.println(ProcessorInterface::getPageIndex(), HEX);
                Serial.print(F("\tPage Offset: 0x")) ;
                Serial.println(ProcessorInterface::getPageOffset(), HEX);
                Serial.print(F("\tData To Write: 0x"));
                Serial.println(dataBits.getWholeValue(), HEX);
            }
            T::write(ProcessorInterface::getPageIndex(),
                     ProcessorInterface::getPageOffset(),
                     ProcessorInterface::getStyle(),
                     dataBits);
            if (informCPU()) {
                break;
            }
            // be careful of querying i960 state at this point because the chipset runs at twice the frequency of the i960
            // so you may still be reading the previous i960 cycle state!
            ProcessorInterface::burstNext<IncrementAddress>();
        }
    }
}


template<bool inDebugMode, bool useInterrupts>
[[gnu::always_inline]]
inline void invocationBody() noexcept {
    // wait until AS goes from low to high
    // then wait until the DEN state is asserted
    while (DigitalPin<i960Pinout::DEN_>::isDeasserted());
    // keep processing data requests until we
    // when we do the transition, record the information we need
    // there are only two parts to this code, either we map into ram or chipset functions
    // we can just check if we are in ram, otherwise it is considered to be chipset. This means that everything not ram is chipset
    // and so we are actually continually mirroring the mapping for the sake of simplicity
    ProcessorInterface::newDataCycle<inDebugMode, decltype(theCache)::CacheEntryMask, useInterrupts>();
}
template<bool allowAddressDebuggingCodePath, bool useInterrupts>
[[gnu::always_inline]]
inline void doInvocationBody() noexcept {
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
void installBootImage() noexcept {

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
        static constexpr auto CacheSize = theCache.getCacheSize();
        //static_assert(CacheSize >= (TargetBoard::cacheLineSize() * TargetBoard::numberOfCacheLines()), "The entry cache set is smaller than the requested cache size");
        // use the cache as a buffer since it won't be in use at this point in time
        auto *storage = theCache.viewAsStorage();
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
            (void) BackingMemoryStorage_t::write(addr, storage, numRead);

            Serial.print(F("."));
        }
        Serial.println();
        Serial.println(F("Transfer complete!"));
        // make sure we close the file before destruction
        theFile.close();
        // clear both caches to be on the safe side
        theCache.clear();
    }
}

using DispatchTable = BodyFunction[256];
extern DispatchTable lookupTable;
extern DispatchTable lookupTable_Debug;
// the setup routine runs once when you press reset:
void setup() {
    // seed random on startup to be on the safe side from analog pin A0, A1, A2, and A3
    randomSeed(analogRead(A0) + analogRead(A1) + analogRead(A2) + analogRead(A3));
    // before we do anything else, configure as many pins as possible and then
    // pull the i960 into a reset state, it will remain this for the entire
    // duration of the setup function
    setupPins(OUTPUT,
              i960Pinout::PSRAM_EN,
              i960Pinout::SD_EN,
              i960Pinout::Reset960,
              i960Pinout::Ready,
              i960Pinout::GPIOSelect,
#ifdef CHIPSET_TYPE1
              i960Pinout::SPI_OFFSET0,
              i960Pinout::SPI_OFFSET1,
              i960Pinout::SPI_OFFSET2,
#endif
              i960Pinout::Reset960,
              i960Pinout::Int0_);
    digitalWrite<i960Pinout::Reset960, HIGH>();
    digitalWrite<i960Pinout::Int0_, HIGH>();
    {
        PinAsserter<i960Pinout::Reset960> holdi960InReset;
        // all of these pins need to be pulled high
        digitalWrite<i960Pinout::PSRAM_EN, HIGH>();
        digitalWrite<i960Pinout::SD_EN, HIGH>();
        digitalWrite<i960Pinout::Ready, HIGH>();
        digitalWrite<i960Pinout::GPIOSelect, HIGH>();
#ifdef CHIPSET_TYPE1
        digitalWrite<i960Pinout::SPI_OFFSET0, LOW>();
        digitalWrite<i960Pinout::SPI_OFFSET1, LOW>();
        digitalWrite<i960Pinout::SPI_OFFSET2, LOW>();
#endif
        // setup the pins that could be attached to an io expander separately
        setupPins(INPUT,
#ifdef CHIPSET_TYPE1
                  i960Pinout::BA1,
                  i960Pinout::BA2,
                  i960Pinout::BA3,
#endif
                  i960Pinout::BE0,
                  i960Pinout::BE1,
                  i960Pinout::BLAST_,
                  i960Pinout::W_R_,
                  i960Pinout::DEN_,
                  i960Pinout::FAIL,
                  i960Pinout::INT_EN0,
                  i960Pinout::INT_EN1);
        //pinMode(i960Pinout::MISO, INPUT_PULLUP);
        theCache.begin();
        SPI.begin();
        // purge the cache pages
        ConfigurationSpace::begin();
        Serial.println(F("i960Sx chipset bringup"));
        ProcessorInterface::begin();
        BackingMemoryStorage_t::begin();

        installBootImage();
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
        doInvocationBody<CompileInAddressDebuggingSupport, true>();
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
template<bool debug>
BodyFunction getExecBody(byte index) noexcept {
    switch (index) {
        case 0:
        case 1:
        case 2:
        case 3:
            return handleMemoryInterface<debug>;
        case TheRTCInterface::SectionID:
            return handleExternalDeviceRequest<debug, TheRTCInterface>;
        case TheDisplayInterface::SectionID:
            return handleExternalDeviceRequest<debug, TheDisplayInterface>;
        case TheSDInterface::SectionID:
            return handleExternalDeviceRequest<debug, TheSDInterface>;
        case TheConsoleInterface::SectionID:
            return handleExternalDeviceRequest<debug, TheConsoleInterface>;
        case ConfigurationSpace::SectionID:
            return handleExternalDeviceRequest<debug, ConfigurationSpace>;
        default: break;
    }
    return fallbackBody<debug>;
}
BodyFunction getNonDebugBody(byte index) noexcept {
    return getExecBody<false>(index);
}
BodyFunction getDebugBody(byte index) noexcept {
    if constexpr (CompileInAddressDebuggingSupport) {
        return getExecBody<true>(index);
    } else {
        return fallbackBody<true>;
    }
}

SdFat SD;
