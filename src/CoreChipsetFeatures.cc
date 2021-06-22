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
    digitalWrite(i960Pinout::Led, value > 0 ? HIGH : LOW);
}
uint8_t
CoreChipsetFeatures::readLed() noexcept {
    return static_cast<uint8_t>(digitalRead(i960Pinout::Led));
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
    consoleBufferBaseAddress_.full = 0;
}
uint8_t
CoreChipsetFeatures::readFromStream(Stream& aStream, Address baseAddress, uint8_t length) noexcept {
    uint8_t count = 0;
    if (auto thing = getThing(baseAddress, LoadStoreStyle::Lower8); thing) {
        // force the i960sx to wait
        count = aStream.readBytes(buffer_, length);
        thing->write(thing->makeAddressRelative(baseAddress), buffer_, length);
    }
    return count;
}
uint8_t
CoreChipsetFeatures::writeToStream(Stream& aStream, Address baseAddress, uint8_t length) noexcept {
    uint8_t count = 0;
    if (auto thing = getThing(baseAddress, LoadStoreStyle::Lower8); thing) {
        thing->read(thing->makeAddressRelative(baseAddress), buffer_, length);
        count = aStream.write(buffer_, length);
    }
    return count;
}
uint8_t
CoreChipsetFeatures::read8(Address address) noexcept {
    switch (static_cast<Registers>(address)) {
        case Registers::Led:
            return readLed();
        case Registers::DisplayMemoryReadsAndWrites:
            return displayMemoryReadsAndWrites();
        case Registers::DisplayCacheLineUpdates:
            return displayCacheLineUpdates();
        case Registers::PortZGPIO:
            return ProcessorInterface::getInterface().readPortZGPIORegister();
        case Registers::PortZGPIODirection:
            return ProcessorInterface::getInterface().getPortZDirectionRegister();
        case Registers::PortZGPIOPolarity:
            return ProcessorInterface::getInterface().getPortZPolarityRegister();
        case Registers::PortZGPIOPullup:
            return ProcessorInterface::getInterface().getPortZPullupResistorRegister();
        case Registers::ConsoleBufferLength:
            return consoleBufferLength_;
        case Registers::ConsoleBufferDoorbell:
            return readFromStream(Serial, consoleBufferBaseAddress_.full, consoleBufferLength_);
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
        case Registers::ConsoleBufferAddressLowerHalf: return consoleBufferBaseAddress_.halves[0];
        case Registers::ConsoleBufferAddressUpperHalf: return consoleBufferBaseAddress_.halves[1];
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
        case Registers::ConsoleBufferAddressLowerHalf:
            consoleBufferBaseAddress_.halves[0] = value;
            break;
        case Registers::ConsoleBufferAddressUpperHalf:
            consoleBufferBaseAddress_.halves[1] = value;
            break;
        default:
            break;
    }
}

void
CoreChipsetFeatures::write8(Address address, uint8_t value) noexcept {
    switch (static_cast<Registers>(address)) {
        case Registers::Led:
            writeLed(value);
            break;
        case Registers::DisplayMemoryReadsAndWrites:
            setDisplayMemoryReadsAndWrites(value != 0);
            break;
        case Registers::DisplayCacheLineUpdates:
            setDisplayCacheLineUpdates(value != 0);
            break;
        case Registers::PortZGPIO:
            ProcessorInterface::getInterface().writePortZGPIORegister(value);
            break;
        case Registers::PortZGPIOPullup:
            ProcessorInterface::getInterface().setPortZPullupResistorRegister(value);
            break;
        case Registers::PortZGPIOPolarity:
            ProcessorInterface::getInterface().setPortZPolarityRegister(value);
            break;
        case Registers::PortZGPIODirection:
            ProcessorInterface::getInterface().setPortZDirectionRegister(value);
            break;
        case Registers::ConsoleBufferLength:
            consoleBufferLength_ = value;
            break;
        case Registers::ConsoleBufferDoorbell:
            (void)writeToStream(Serial, consoleBufferBaseAddress_.full, consoleBufferLength_);
            break;
        default:
            break;
    }
}
