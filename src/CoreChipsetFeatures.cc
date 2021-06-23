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
        case Registers::PatternEngine_ActualPattern000: return pattern_.shorts[0];
        case Registers::PatternEngine_ActualPattern001: return pattern_.shorts[1];
        case Registers::PatternEngine_ActualPattern010: return pattern_.shorts[2];
        case Registers::PatternEngine_ActualPattern011: return pattern_.shorts[3];
        case Registers::PatternEngine_ActualPattern100: return pattern_.shorts[4];
        case Registers::PatternEngine_ActualPattern101: return pattern_.shorts[5];
        case Registers::PatternEngine_ActualPattern110: return pattern_.shorts[6];
        case Registers::PatternEngine_ActualPattern111: return pattern_.shorts[7];
        case Registers::PatternEngine_LengthLower: return patternLength_.halves[0]; break;
        case Registers::PatternEngine_LengthUpper: return patternLength_.halves[1]; break;
        case Registers::PatternEngine_StartAddressLower: return patternAddress_.halves[0]; break;
        case Registers::PatternEngine_StartAddressUpper: return patternAddress_.halves[1]; break;
        case Registers::PatternEngine_Doorbell: return invokePatternEngine();
        case Registers::CopyEngine_DestinationAddressLower: return copyEngineDestinationAddress_.halves[0]; break;
        case Registers::CopyEngine_DestinationAddressUpper: return copyEngineDestinationAddress_.halves[1] ; break;
        case Registers::CopyEngine_SourceAddressLower: return copyEngineSourceAddress_.halves[0] ; break;
        case Registers::CopyEngine_SourceAddressUpper: return copyEngineSourceAddress_.halves[1] ; break;
        case Registers::CopyEngine_LengthLower: return copyEngineLength_.halves[0] ; break;
        case Registers::CopyEngine_LengthUpper: return copyEngineLength_.halves[1] ; break;
        case Registers::CopyEngine_Doorbell: return invokeCopyEngine(); break;
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
        case Registers::PatternEngine_ActualPattern000: pattern_.shorts[0] = value; break;
        case Registers::PatternEngine_ActualPattern001: pattern_.shorts[1] = value; break;
        case Registers::PatternEngine_ActualPattern010: pattern_.shorts[2] = value; break;
        case Registers::PatternEngine_ActualPattern011: pattern_.shorts[3] = value; break;
        case Registers::PatternEngine_ActualPattern100: pattern_.shorts[4] = value; break;
        case Registers::PatternEngine_ActualPattern101: pattern_.shorts[5] = value; break;
        case Registers::PatternEngine_ActualPattern110: pattern_.shorts[6] = value; break;
        case Registers::PatternEngine_ActualPattern111: pattern_.shorts[7] = value; break;
        case Registers::PatternEngine_LengthLower: patternLength_.halves[0] = value; break;
        case Registers::PatternEngine_LengthUpper: patternLength_.halves[1] = value; break;
        case Registers::PatternEngine_StartAddressLower: patternAddress_.halves[0] = value; break;
        case Registers::PatternEngine_StartAddressUpper: patternAddress_.halves[1] = value; break;
        case Registers::PatternEngine_Doorbell: (void)invokePatternEngine(); break;
        case Registers::CopyEngine_DestinationAddressLower: copyEngineDestinationAddress_.halves[0] = value; break;
        case Registers::CopyEngine_DestinationAddressUpper: copyEngineDestinationAddress_.halves[1] = value; break;
        case Registers::CopyEngine_SourceAddressLower: copyEngineSourceAddress_.halves[0] = value; break;
        case Registers::CopyEngine_SourceAddressUpper: copyEngineSourceAddress_.halves[1] = value; break;
        case Registers::CopyEngine_LengthLower: copyEngineLength_.halves[0] = value; break;
        case Registers::CopyEngine_LengthUpper: copyEngineLength_.halves[1] = value; break;
        case Registers::CopyEngine_Doorbell: (void)invokeCopyEngine(); break;
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
        default:
            break;
    }
}

uint16_t
CoreChipsetFeatures::invokePatternEngine() noexcept {
    if (auto* thing = getThing(patternAddress_.wholeValue_, LoadStoreStyle::Lower8); thing) {
        TemporarilyDisableThingCache cacheOff(thing);
        auto fullCopies = patternLength_.wholeValue_ / 16;
        auto slop = patternLength_.wholeValue_ % 16;
        Address addr = patternAddress_.wholeValue_;
        for (uint32_t i = 0; i < fullCopies; ++i, addr+=16) {
            thing->write(addr, pattern_.bytes, 16);
        }
        if (slop > 0) {
            thing->write(addr, pattern_.bytes, slop);
        }
        return 0;
    } else {
        return -1;
    }
}

uint16_t
CoreChipsetFeatures::invokeCopyEngine() noexcept {
    auto srcAddress = copyEngineSourceAddress_.wholeValue_;
    auto destAddress = copyEngineDestinationAddress_.wholeValue_;
    if (auto src = getThing(srcAddress, LoadStoreStyle::Lower8),
             dest = getThing(destAddress, LoadStoreStyle::Lower8);
            src && dest) {
        TemporarilyDisableThingCache srcCacheOff(src);
        TemporarilyDisableThingCache destCacheOff(dest);
        auto fullCopies = copyEngineLength_.wholeValue_ / CopyEngineCacheSize;
        auto slop = copyEngineLength_.wholeValue_ % CopyEngineCacheSize;
        Address srcAddrPtr = copyEngineSourceAddress_.wholeValue_;
        Address destAddrPtr = copyEngineDestinationAddress_.wholeValue_;
        Serial.print(F("COPYING FROM 0x"));
        Serial.print(srcAddrPtr, HEX);
        Serial.print(F(" TO 0x"));
        Serial.print(destAddrPtr, HEX);
        Serial.print(F(" length: 0x"));
        Serial.println(copyEngineLength_.wholeValue_, HEX);
        Serial.print(F("Full Copies, Slop: [0x"));
        Serial.print(fullCopies, HEX);
        Serial.print(F(", 0x"));
        Serial.print(slop, HEX);
        Serial.println(F("]"));
        for (uint32_t i = 0; i < fullCopies; ++i, srcAddrPtr += CopyEngineCacheSize, destAddrPtr += CopyEngineCacheSize) {
            Serial.print(F("COPYING FROM 0x"));
            Serial.print(srcAddrPtr, HEX);
            Serial.print(F(" TO 0x"));
            Serial.println(destAddrPtr, HEX);
            src->read(srcAddrPtr, copyEngineBuffer_, CopyEngineCacheSize);
            dest->write(destAddrPtr, copyEngineBuffer_, CopyEngineCacheSize);
        }
        if (slop > 0) {
            Serial.print(F("COPYING FROM 0x"));
            Serial.print(srcAddrPtr, HEX);
            Serial.print(F(" TO 0x"));
            Serial.println(destAddrPtr, HEX);
            src->read(srcAddrPtr, copyEngineBuffer_, slop);
            dest->write(destAddrPtr, copyEngineBuffer_, slop);
        }
        return 0;
    } else {
        return -1;
    }
}

void
CoreChipsetFeatures::begin() noexcept {
    if constexpr (true) {
        Serial.print(F("ADDRESS OF LED: 0x"));
        Serial.println(static_cast<uint32_t>(Registers::Led) + 0xFE00'0000, HEX);
        Serial.print(F("BASE ADDRESS OF PATTERN: 0x"));
        Serial.println(static_cast<uint32_t>(Registers::PatternEngine_ActualPattern000) + 0xFE00'0000, HEX);
        Serial.print(F("BASE ADDRESS OF PATTERN LENGTH: 0x"));
        Serial.println(static_cast<uint32_t>(Registers::PatternEngine_LengthLower) + 0xFE00'0000, HEX);
        Serial.print(F("BASE ADDRESS OF PATTERN ADDRESS: 0x"));
        Serial.println(static_cast<uint32_t>(Registers::PatternEngine_StartAddressLower) + 0xFE00'0000, HEX);
        Serial.print(F("BASE ADDRESS OF PATTERN ENGINE DOORBELL: 0x"));
        Serial.println(static_cast<uint32_t>(Registers::PatternEngine_Doorbell) + 0xFE00'0000, HEX);
        Serial.print(F("BASE ADDRESS OF COPY ENGINE DOORBELL: 0x"));
        Serial.println(static_cast<uint32_t>(Registers::CopyEngine_Doorbell) + 0xFE00'0000, HEX);
        Serial.print(F("BASE ADDRESS OF COPY ENGINE LENGTH: 0x"));
        Serial.println(static_cast<uint32_t>(Registers::CopyEngine_LengthLower) + 0xFE00'0000, HEX);
        Serial.print(F("BASE ADDRESS OF COPY ENGINE SOURCE ADDRESS: 0x"));
        Serial.println(static_cast<uint32_t>(Registers::CopyEngine_SourceAddressLower) + 0xFE00'0000, HEX);
        Serial.print(F("BASE ADDRESS OF COPY ENGINE DESTINATION ADDRESS: 0x"));
        Serial.println(static_cast<uint32_t>(Registers::CopyEngine_DestinationAddressLower) + 0xFE00'0000, HEX);
    }
}