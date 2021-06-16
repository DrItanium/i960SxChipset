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
    uint16_t read16(ProcessorInterface::IOExpanderAddress addr, byte opcode) {
        digitalWrite(i960Pinout::GPIOSelect, LOW);
        SPI.transfer(generateReadOpcode(addr));
        SPI.transfer(opcode);
        auto lower = static_cast<uint16_t>(SPI.transfer(0x00));
        auto lowest = static_cast<uint16_t>(SPI.transfer(0x00)) << 8;
        digitalWrite(i960Pinout::GPIOSelect, HIGH);
        return lower | lowest;
    }
    uint16_t readGPIO16(ProcessorInterface::IOExpanderAddress addr) {
        return read16(addr, GPIOBaseAddress);
    }
    void write16(ProcessorInterface::IOExpanderAddress addr, byte opcode, uint16_t value) {
        digitalWrite(i960Pinout::GPIOSelect, LOW);
        SPI.transfer(generateWriteOpcode(addr));
        SPI.transfer(opcode);
        SPI.transfer(static_cast<uint8_t>(value));
        SPI.transfer(static_cast<uint8_t>(value >> 8));
        digitalWrite(i960Pinout::GPIOSelect, HIGH);
    }
    void writeGPIO16(ProcessorInterface::IOExpanderAddress addr, uint16_t value) {
        write16(addr, GPIOBaseAddress, value);
    }
    void writeDirection(ProcessorInterface::IOExpanderAddress addr, uint16_t value) {
        write16(addr, IODirBaseAddress, value);
    }

}
Address
ProcessorInterface::getAddress() noexcept {
    SPI.beginTransaction(theSettings);
    auto lower16Addr = static_cast<Address>(readGPIO16(ProcessorInterface::IOExpanderAddress::Lower16Lines));
    auto upper16Addr = static_cast<Address>(readGPIO16(ProcessorInterface::IOExpanderAddress::Upper16Lines)) << 16;
    SPI.endTransaction();
    return lower16Addr | upper16Addr;
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
    //return static_cast<uint16_t>(dataLines_.readGPIOs());
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
    //dataLines_.writeGPIOs(value);
}



uint8_t
ProcessorInterface::getByteEnableBits() noexcept {
    return (extra_.readGPIOs() & 0b11000) >> 3;
}

decltype(LOW)
ProcessorInterface::getByteEnable0() noexcept {
    return (getByteEnableBits() & 1) == 0 ? LOW : HIGH;
}
decltype(LOW)
ProcessorInterface::getByteEnable1() noexcept {
    return (getByteEnableBits() & 0b10) == 0 ? LOW : HIGH;
}

uint8_t
ProcessorInterface::getBurstAddressBits() noexcept {
    auto gpios = extra_.readGPIOs();
    return (gpios & 0b111) << 1;
}

Address
ProcessorInterface::getBurstAddress(Address base) noexcept {
    return getBurstAddress(base, static_cast<Address>(getBurstAddressBits()));
}
Address
ProcessorInterface::getBurstAddress() noexcept {
    return getBurstAddress(getAddress());
}

bool
ProcessorInterface::isReadOperation() const noexcept {
    return DigitalPin<i960Pinout::W_R_>::isAsserted();
}
bool
ProcessorInterface::isWriteOperation() const noexcept {
    return DigitalPin<i960Pinout::W_R_>::isDeasserted();
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
        // at bootup, the IOExpanders all respond to 0b000 because IOCON.HAEN is
        // disabled. We can send out a single IOCON.HAEN enable message and all
        // should receive it.
        // so do a begin operation on all chips (0b000)
        dataLines_.begin();
        // set IOCON.HAEN on all chips
        dataLines_.enableHardwareAddressPins();
        // now we have to refresh our on mcu flags for each io expander
        lower16_.refreshIOCon();
        upper16_.refreshIOCon();
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
LoadStoreStyle
ProcessorInterface::getStyle() noexcept {
    return static_cast<LoadStoreStyle>(getByteEnableBits());
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
