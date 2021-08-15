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
//
// Created by jwscoggins on 6/21/21.
//
#include "CoreChipsetFeatures.h"

void
CoreChipsetFeatures::writeLed(uint8_t value) noexcept {
    // disabled for the current board since the led is bound to the ready pin
    //digitalWrite(i960Pinout::CACHE_A0, value > 0 ? HIGH : LOW);
}
uint8_t
CoreChipsetFeatures::readLed() noexcept {
    // disabled for the current board since the led is bound to the ready pin
    return 0;
    //return static_cast<uint8_t>(digitalRead(i960Pinout::CACHE_A0));
}
void
CoreChipsetFeatures::setDisplayMemoryReadsAndWrites(bool value) noexcept {
    if constexpr (AllowDebuggingStatements) {
        displayMemoryReadsAndWrites_ = value;
    }
}
void
CoreChipsetFeatures::setDisplayCacheLineUpdates(bool value) noexcept {
    if constexpr (AllowDebuggingStatements) {
        displayCacheLineUpdates_ = value;
    }
}
CoreChipsetFeatures::CoreChipsetFeatures(Address offsetFromIOBase) : IOSpaceThing(offsetFromIOBase, offsetFromIOBase + 0x100) {
}
uint8_t
CoreChipsetFeatures::read8(Address address) noexcept {
    switch (static_cast<Registers>(address)) {
        case Registers::DisplayMemoryReadsAndWrites: return displayMemoryReadsAndWrites();
        case Registers::DisplayCacheLineUpdates: return displayCacheLineUpdates();
        default:
            return 0;
    }
}
uint16_t
CoreChipsetFeatures::read16(Address address) noexcept {
    switch (static_cast<Registers>(address)) {
        case Registers::ConsoleIO: return Serial.read();
        case Registers::ConsoleAvailable: return Serial.available();
        case Registers::ConsoleAvailableForWrite: return Serial.availableForWrite();
        default: return 0;
    }
}
void
CoreChipsetFeatures::write16(Address address, uint16_t value) noexcept {
    switch (static_cast<Registers>(address)) {
        case Registers::ConsoleFlush:
            Serial.flush();
            break;
        case Registers::ConsoleIO:
            Serial.write(static_cast<char>(value));
            break;
        default:
            break;
    }
}

void
CoreChipsetFeatures::write8(Address address, uint8_t value) noexcept {
    switch (static_cast<Registers>(address)) {
        case Registers::DisplayMemoryReadsAndWrites:
            setDisplayMemoryReadsAndWrites(value != 0);
            break;
        case Registers::DisplayCacheLineUpdates:
            setDisplayCacheLineUpdates(value != 0);
            break;
        default:
            break;
    }
}


void
CoreChipsetFeatures::begin() noexcept {
    if constexpr (false) {
        Serial.print(F("ADDRESS OF LED: 0x"));
        Serial.println(static_cast<uint32_t>(Registers::Led) + 0xFE00'0000, HEX);
    }
}
Address
CoreChipsetFeatures::makeAddressRelative(Address input) const noexcept {
    // we already know that this address is exactly one byte wide so just return the lowest byte as it's own thing
    return SplitWord32(input).bytes[0];
}
bool
CoreChipsetFeatures::respondsTo(Address address) const noexcept {
    SplitWord32 theAddr(address);
    SplitWord32 theBase(getBaseAddress());
    return (theAddr.bytes[3] == theBase.bytes[3]) && // okay we are in io space
           (theAddr.bytes[2] == theBase.bytes[2]) && // okay we are in the proper sub section of io space
           (theAddr.bytes[1] == theBase.bytes[1]);   // and the proper sub subsection of io space
}
