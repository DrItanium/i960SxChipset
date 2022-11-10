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
#include "CacheDescription.h"
#include "i960SxChipset.h"
extern SdFat SD;

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
    ProcessorInterface::newDataCycle<inDebugMode, useInterrupts>();
}
template<bool useInterrupts>
[[gnu::always_inline]] inline void doInvocationBody() noexcept {
    invocationBody<CompileInAddressDebuggingSupport, useInterrupts>();
}
void installBootImage() noexcept {
    static constexpr auto CacheSize = theCache.getCacheSize();
    auto *storage = theCache.viewAsStorage();
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
        Serial.println(F("TRANSFERRING BOOT.SYS TO RAM FROM SDCARD"));
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
        // make sure we close the file before destruction
        theFile.close();
    }
    Serial.println();
    Serial.println(F("Transfer complete!"));
    // clear both caches to be on the safe side
    theCache.clear();
    // seed the cache with data to simplify getting lines
    theCache.precache();
}
void
setupPins() noexcept {

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
#ifdef CHIPSET_TYPE1
    digitalWrite<i960Pinout::SPI_OFFSET0, HIGH>();
    digitalWrite<i960Pinout::SPI_OFFSET1, HIGH>();
    digitalWrite<i960Pinout::SPI_OFFSET2, HIGH>();
    digitalWrite<i960Pinout::MEMBLK0_A0, LOW>();
    digitalWrite<i960Pinout::MEMBLK0_A1, LOW>();
    digitalWrite<i960Pinout::PSRAM_EN, HIGH>();
    digitalWrite<i960Pinout::MEMBLK0_, HIGH>();
    digitalWrite<i960Pinout::TFT_CS, HIGH>();
#endif
    digitalWrite<i960Pinout::SD_EN, HIGH>();
    digitalWrite<i960Pinout::Ready, HIGH>();
    digitalWrite<i960Pinout::GPIOSelect, HIGH>();
    // setup the pins that could be attached to an io expander separately
    setupPins(INPUT,
              i960Pinout::RAM_SPACE_,
              i960Pinout::IO_SPACE_,
              i960Pinout::BE0,
              i960Pinout::BE1,
              i960Pinout::BLAST_,
              i960Pinout::W_R_,
              i960Pinout::FAIL,
              i960Pinout::DEN_,
              i960Pinout::IOEXP_INT0,
              i960Pinout::IOEXP_INT1,
              i960Pinout::IOEXP_INT2,
              i960Pinout::IOEXP_INT3);
    SPI.beginTransaction(SPISettings(TargetBoard::runIOExpanderSPIInterfaceAt(), MSBFIRST, SPI_MODE0));
    // at bootup, the IOExpanders all respond to 0b000 because IOCON.HAEN is
    // disabled. We can send out a single IOCON.HAEN enable message and all
    // should receive it.
    // so do a begin operation on all chips (0b000)
    // set IOCON.HAEN on all chips
    // mirror the interrupts for the upper 16-bits, for some reason, the upper most 8-bits are never marked as changed
    // mirror the interrupts on the lower 16-bits to free up two pins at the cost of update accuracy
    // we want two separate pins free for the data lines io expander
    // we want mirroring on the data lines interrupts
    // disable banking and activate byte mode (this should enable the special byte increment wrap around mode)
    static constexpr uint8_t initialIOCONValue_ = 0b0010'1000;
    //static constexpr uint16_t currentGPIO4Status_ = 0b00000000'10010010;
    //static constexpr uint16_t currentGPIO4Direction_ = 0b00000000'00100000;
    ProcessorInterface::write8<ProcessorInterface::DataLines, ProcessorInterface::MCP23x17Registers::IOCON, false>(initialIOCONValue_);
    ProcessorInterface::write8<ProcessorInterface::Upper16Lines, ProcessorInterface::MCP23x17Registers::IOCON, false>(0b0100'1000);
    ProcessorInterface::write8<ProcessorInterface::Lower16Lines, ProcessorInterface::MCP23x17Registers::IOCON, false>(0b0100'1000);
    ProcessorInterface::write8<ProcessorInterface::MemoryCommitExtras, ProcessorInterface::MCP23x17Registers::IOCON, false>(initialIOCONValue_);

    //ProcessorInterface::writeDirection<ProcessorInterface::MemoryCommitExtras, false>(currentGPIO4Direction_);
    //ProcessorInterface::writeGPIO16<ProcessorInterface::MemoryCommitExtras, false>(currentGPIO4Status_);
    // immediately pull the i960 into reset as soon as possible
    DigitalPin<i960Pinout::RESET960>::setup();
    DigitalPin<i960Pinout::INT960_0_>::setup();
    DigitalPin<i960Pinout::INT960_1>::setup();
    DigitalPin<i960Pinout::INT960_2>::setup();
    DigitalPin<i960Pinout::INT960_3_>::setup();
    DigitalPin<i960Pinout::HOLD>::setup();
    DigitalPin<i960Pinout::HLDA>::setup();
    DigitalPin<i960Pinout::LOCK_>::setup();
    DigitalPin<i960Pinout::LOCK_>::deassertPin();
    DigitalPin<i960Pinout::INT960_0_>::deassertPin();
    DigitalPin<i960Pinout::INT960_1>::deassertPin();
    DigitalPin<i960Pinout::INT960_2>::deassertPin();
    DigitalPin<i960Pinout::INT960_3_>::deassertPin();
    DigitalPin<i960Pinout::HOLD>::deassertPin();
    SPI.endTransaction();
}
void
startupSerial() noexcept {
    Serial.begin(115200);
    while (!Serial) {
        delay(10);
    }
}
void
setupChipset() noexcept {
    // always do this first to make sure that we put the i960 into reset regardless of target
    DigitalPin<i960Pinout::RESET960>::assertPin();
    // before we do anything else, configure as many pins as possible and then
    // pull the i960 into a reset state, it will remain this for the entire
    // duration of the setup function
    // get SPI setup ahead of time
    theCache.begin();
    while (!SD.begin(static_cast<int>(i960Pinout::SD_EN))) {
        Serial.println(F("SD CARD INIT FAILED...WILL RETRY SOON"));
        delay(1000);
    }
    // purge the cache pages
    //ConfigurationSpace::begin();
    if constexpr (DisplayBootupInformation) {
        Serial.println(F("i960Sx chipset bringup"));
    }
    BackingMemoryStorage_t::begin();
    installBootImage();
    delay(100);
    if constexpr (DisplayBootupInformation) {
        Serial.println(F("i960Sx chipset brought up fully!"));
    }
    DigitalPin<i960Pinout::RESET960>::deassertPin();
}
// the setup routine runs once when you press reset:
void
setup() {
    // seed random on startup to be on the safe side from analog pin A0, A1, A2, and A3
    randomSeed(analogRead(A0) + analogRead(A1) + analogRead(A2) + analogRead(A3));
    SPI.begin();
    setupPins();
    ProcessorInterface::begin();
    startupSerial();
    setupChipset();
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
    doInvocationBody<false>();
    doInvocationBody<false>();
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

        doInvocationBody<UseIOExpanderAddressLineInterrupts>();
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

SdFat SD;
L1Cache theCache;
