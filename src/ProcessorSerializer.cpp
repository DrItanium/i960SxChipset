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

#include "ProcessorSerializer.h"

namespace
{
    SPISettings theSettings(TargetBoard::onFeatherBoard() ? 8'000'000 : 10'000'000, MSBFIRST, SPI_MODE0);
    constexpr byte generateReadOpcode(ProcessorInterface::IOExpanderAddress address) noexcept {
        return 0b0100'0000 | ((static_cast<uint8_t>(address) & 0b111) << 1) | 1;
    }
    constexpr byte generateWriteOpcode(ProcessorInterface::IOExpanderAddress address) noexcept {
        return 0b0100'0000 | ((static_cast<byte>(address) & 0b111) << 1);
    }
    constexpr byte IODirBaseAddress = 0x00;
    constexpr byte GPIOBaseAddress = 0x12;
    constexpr byte IOConAddress = 0x0A;
    uint16_t read16(ProcessorInterface::IOExpanderAddress addr, byte opcode) {
        uint8_t buffer[4] = {
                generateReadOpcode(addr),
                opcode,
                0x00,
                0x00,
        };

        digitalWrite(i960Pinout::GPIOSelect, LOW);
        SPI.transfer(buffer, 4);
        digitalWrite(i960Pinout::GPIOSelect, HIGH);
        auto lower = static_cast<uint16_t>(buffer[2]);
        auto lowest = static_cast<uint16_t>(buffer[3]) << 8;
        return lower | lowest;
    }
    uint8_t read8(ProcessorInterface::IOExpanderAddress addr, byte opcode) {
        uint8_t buffer[3] = {
                generateReadOpcode(addr),
                opcode,
                0x00,
        };

        digitalWrite(i960Pinout::GPIOSelect, LOW);
        SPI.transfer(buffer, 3);
        digitalWrite(i960Pinout::GPIOSelect, HIGH);
        return buffer[2];
    }
    uint16_t readGPIO16(ProcessorInterface::IOExpanderAddress addr) {
        return read16(addr, GPIOBaseAddress);
    }
    void write16(ProcessorInterface::IOExpanderAddress addr, byte opcode, uint16_t value) {
        uint8_t buffer[4] = {
                generateWriteOpcode(addr),
                opcode,
                static_cast<uint8_t>(value),
                static_cast<uint8_t>(value >> 8),
        };
        digitalWrite(i960Pinout::GPIOSelect, LOW);
        SPI.transfer(buffer, 4);
        digitalWrite(i960Pinout::GPIOSelect, HIGH);
    }
    void write8(ProcessorInterface::IOExpanderAddress addr, byte opcode, uint8_t value) {
        uint8_t buffer[3] = {
                generateWriteOpcode(addr),
                opcode,
                value
        };
        digitalWrite(i960Pinout::GPIOSelect, LOW);
        SPI.transfer(buffer, 3);
        digitalWrite(i960Pinout::GPIOSelect, HIGH);
    }
    void writeGPIO16(ProcessorInterface::IOExpanderAddress addr, uint16_t value) {
        write16(addr, GPIOBaseAddress, value);
    }
    void writeDirection(ProcessorInterface::IOExpanderAddress addr, uint16_t value) {
        write16(addr, IODirBaseAddress, value);
    }
}
uint16_t
ProcessorInterface::getDataBits() noexcept {
    SPI.beginTransaction(theSettings);
    if (dataLinesDirection_ != 0xFFFF) {
        dataLinesDirection_ = 0xFFFF;
        writeDirection(ProcessorInterface::IOExpanderAddress::DataLines, dataLinesDirection_);
    }
    auto result = readGPIO16(ProcessorInterface::IOExpanderAddress::DataLines);
    SPI.endTransaction();
    return result;
}

void
ProcessorInterface::setDataBits(uint16_t value) noexcept {
    SPI.beginTransaction(theSettings);
    if (dataLinesDirection_ != 0) {
        dataLinesDirection_ = 0;
        writeDirection(ProcessorInterface::IOExpanderAddress::DataLines, dataLinesDirection_);
    }
    writeGPIO16(ProcessorInterface::IOExpanderAddress::DataLines, value);
    SPI.endTransaction();
}





void
ProcessorInterface::setHOLDPin(decltype(LOW) value) noexcept {
    digitalWrite(static_cast<int>(ExtraGPIOExpanderPinout::HOLD), value, extra_);
}
void
ProcessorInterface::setLOCKPin(decltype(LOW) value) noexcept {
    digitalWrite(static_cast<int>(ExtraGPIOExpanderPinout::LOCK_), value, extra_);
}

void
ProcessorInterface::begin() noexcept {
    if (!initialized_) {
        initialized_ = true;
        pinMode(i960Pinout::GPIOSelect, OUTPUT);
        pinMode(i960Pinout::GPIOSelect, HIGH);
        // at bootup, the IOExpanders all respond to 0b000 because IOCON.HAEN is
        // disabled. We can send out a single IOCON.HAEN enable message and all
        // should receive it.
        // so do a begin operation on all chips (0b000)
        // set IOCON.HAEN on all chips
        auto iocon = read8(ProcessorInterface::IOExpanderAddress::DataLines, IOConAddress);
        write8(ProcessorInterface::IOExpanderAddress::DataLines, IOConAddress, iocon & 0b0111'1110);
        // now we have to refresh our on mcu flags for each io expander
        extra_.refreshIOCon();
        // now all devices tied to this ~CS pin have separate addresses
        // make each of these inputs
        writeDirection(IOExpanderAddress::Lower16Lines, 0xFFFF);
        writeDirection(IOExpanderAddress::Upper16Lines, 0xFFFF);
        writeDirection(IOExpanderAddress::DataLines, dataLinesDirection_);
        writeDirection(IOExpanderAddress::MemoryCommitExtras, 0x005F);
        setHOLDPin(LOW);
        setLOCKPin(HIGH);
    }
}
void
ProcessorInterface::setPortZDirectionRegister(byte value) noexcept {
    extra_.setPortBDirection(value);
}
byte
ProcessorInterface::getPortZDirectionRegister() noexcept {
    return extra_.getPortBDirection();
}
void ProcessorInterface::setPortZPolarityRegister(byte value) noexcept {
    extra_.setPortBPolarity(value);

}
byte ProcessorInterface::getPortZPolarityRegister() noexcept {
    return extra_.getPortBPolarity();
}
void ProcessorInterface::setPortZPullupResistorRegister(byte value) noexcept {
    extra_.setPortBPullupResistorRegister(value);
}
byte ProcessorInterface::getPortZPullupResistorRegister() noexcept {
    return extra_.getPortBPullupResistorRegister();
}
byte ProcessorInterface::readPortZGPIORegister() noexcept {
    return extra_.readPortB();
}
void ProcessorInterface::writePortZGPIORegister(byte value) noexcept {
    extra_.writePortB(value);
}

void ProcessorInterface::newDataCycle() noexcept {
    clearDENTrigger();
    SPI.beginTransaction(theSettings);
    auto lower16Addr = static_cast<Address>(readGPIO16(ProcessorInterface::IOExpanderAddress::Lower16Lines));
    auto upper16Addr = static_cast<Address>(readGPIO16(ProcessorInterface::IOExpanderAddress::Upper16Lines)) << 16;
    SPI.endTransaction();
    auto currentBaseAddress_ = lower16Addr | upper16Addr;
    upperMaskedAddress_ = 0xFFFFFFF0 & currentBaseAddress_;
    address_ = currentBaseAddress_;
    isReadOperation_ = DigitalPin<i960Pinout::W_R_>::isAsserted();
}
void ProcessorInterface::updateDataCycle() noexcept {
    auto bits = readGPIO16(IOExpanderAddress::MemoryCommitExtras);
    auto burstAddressBits = static_cast<byte>((bits & 0b111) << 1);
    auto byteEnableBits = static_cast<byte>((bits & 0b11000) >> 3);
    lss_ = static_cast<LoadStoreStyle>(byteEnableBits);
    address_ = upperMaskedAddress_ | burstAddressBits;
}
