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
#include <SPI.h>
namespace
{
    enum class MCP23x17Registers : byte {
        IODIRA = 0,
        IODIRB,
        IPOLA,
        IPOLB,
        GPINTENA,
        GPINTENB,
        DEFVALA,
        DEFVALB,
        INTCONA,
        INTCONB,
        _IOCONA,
        _IOCONB,
        GPPUA,
        GPPUB,
        INTFA,
        INTFB,
        INTCAPA,
        INTCAPB,
        GPIOA,
        GPIOB,
        OLATA,
        OLATB,
        OLAT = OLATA,
        GPIO = GPIOA,
        IOCON = _IOCONA,
        IODIR = IODIRA,
        INTCAP = INTCAPA,
        INTF = INTFA,
        GPPU = GPPUA,
        INTCON = INTCONA,
        DEFVAL = DEFVALA,
        GPINTEN = GPINTENA,
        IPOL = IPOLA,
    };
    constexpr byte generateReadOpcode(ProcessorInterface::IOExpanderAddress address) noexcept {
        return 0b0100'0000 | ((static_cast<uint8_t>(address) & 0b111) << 1) | 1;
    }
    constexpr byte generateWriteOpcode(ProcessorInterface::IOExpanderAddress address) noexcept {
        return 0b0100'0000 | ((static_cast<byte>(address) & 0b111) << 1);
    }
    SPISettings ioexpander(8_MHz, MSBFIRST, SPI_MODE0);
    inline void doSPI(uint8_t* buffer, size_t count) {
        SPI.beginTransaction(ioexpander);
        DigitalPin<i960Pinout::GPIOSelect>::togglePin();
        SPI.transfer(buffer, count);
        DigitalPin<i960Pinout::GPIOSelect>::togglePin();
        SPI.endTransaction();
    }
    uint16_t read16(ProcessorInterface::IOExpanderAddress addr, MCP23x17Registers opcode) {
        uint8_t buffer[4] = {
                generateReadOpcode(addr),
                static_cast<byte>(opcode),
                0x00,
                0x00,
        };
        doSPI(buffer, 4);
        auto lower = static_cast<uint16_t>(buffer[2]);
        auto lowest = static_cast<uint16_t>(buffer[3]) << 8;
        return lower | lowest;
    }
    uint8_t read8(ProcessorInterface::IOExpanderAddress addr, MCP23x17Registers opcode) {
        uint8_t buffer[3] = {
                generateReadOpcode(addr),
                static_cast<byte>(opcode),
                0x00,
        };

        doSPI(buffer, 3);
        return buffer[2];
    }
    uint16_t readGPIO16(ProcessorInterface::IOExpanderAddress addr) {
        return read16(addr, MCP23x17Registers::GPIO);
    }

    void write16(ProcessorInterface::IOExpanderAddress addr, MCP23x17Registers opcode, uint16_t value) {
        uint8_t buffer[4] = {
                generateWriteOpcode(addr),
                static_cast<byte>(opcode),
                static_cast<uint8_t>(value),
                static_cast<uint8_t>(value >> 8),
        };
        doSPI(buffer, 4);
    }
    void write8(ProcessorInterface::IOExpanderAddress addr, MCP23x17Registers opcode, uint8_t value) {
        uint8_t buffer[3] = {
                generateWriteOpcode(addr),
                static_cast<byte>(opcode),
                value
        };
        doSPI(buffer, 3);
    }
    void writeGPIO16(ProcessorInterface::IOExpanderAddress addr, uint16_t value) {
        write16(addr, MCP23x17Registers::GPIO, value);
    }
    void writeDirection(ProcessorInterface::IOExpanderAddress addr, uint16_t value) {
        write16(addr, MCP23x17Registers::IODIR, value);
    }
}
uint16_t
ProcessorInterface::getDataBits() noexcept {
    if (dataLinesDirection_ != 0xFFFF) {
        dataLinesDirection_ = 0xFFFF;
        writeDirection(ProcessorInterface::IOExpanderAddress::DataLines, dataLinesDirection_);
    }
    auto result = readGPIO16(ProcessorInterface::IOExpanderAddress::DataLines);
    return result;
}

void
ProcessorInterface::setDataBits(uint16_t value) noexcept {
    if (dataLinesDirection_ != 0) {
        dataLinesDirection_ = 0;
        writeDirection(ProcessorInterface::IOExpanderAddress::DataLines, dataLinesDirection_);
    }
    writeGPIO16(ProcessorInterface::IOExpanderAddress::DataLines, value);
}




void
ProcessorInterface::updateOutputLatch() noexcept {
    // construct the bit pattern as needed
    byte latchValue = 0;
    if (holdValue_ && lockValue_) {
        latchValue = 0b1010'0000;
    } else if (holdValue_ && !lockValue_) {
        latchValue = 0b0010'0000;
    } else if (!holdValue_ && lockValue_) {
       latchValue = 0b1000'0000;
    }
    write8(IOExpanderAddress::MemoryCommitExtras, MCP23x17Registers::OLATA, latchValue);
}
void
ProcessorInterface::setHOLDPin(bool value) noexcept {
    if (value != holdValue_) {
        holdValue_ = value;
        updateOutputLatch();
    }
}
void
ProcessorInterface::setLOCKPin(bool value) noexcept {
    if (value != lockValue_) {
        lockValue_ = value;
        updateOutputLatch();
    }
}

void
ProcessorInterface::begin() noexcept {
    if (!initialized_) {
        initialized_ = true;
        pinMode(i960Pinout::GPIOSelect, OUTPUT);
        digitalWrite(i960Pinout::GPIOSelect, HIGH);
        // at bootup, the IOExpanders all respond to 0b000 because IOCON.HAEN is
        // disabled. We can send out a single IOCON.HAEN enable message and all
        // should receive it.
        // so do a begin operation on all chips (0b000)
        // set IOCON.HAEN on all chips
        auto iocon = read8(ProcessorInterface::IOExpanderAddress::DataLines, MCP23x17Registers::IOCON);
        write8(ProcessorInterface::IOExpanderAddress::DataLines, MCP23x17Registers::IOCON, iocon | 0b0000'1000);
        // now all devices tied to this ~CS pin have separate addresses
        // make each of these inputs
        writeDirection(IOExpanderAddress::Lower16Lines, 0xFFFF);
        writeDirection(IOExpanderAddress::Upper16Lines, 0xFFFF);
        writeDirection(IOExpanderAddress::DataLines, dataLinesDirection_);
        writeDirection(IOExpanderAddress::MemoryCommitExtras, 0x005F);
        // we can just set the pins up in a single write operation to the olat, since only the pins configured as outputs will be affected
        write8(IOExpanderAddress::MemoryCommitExtras, MCP23x17Registers::OLATA, 0b1000'0000);

    }
}
void
ProcessorInterface::setPortZDirectionRegister(byte value) noexcept {
    write8(IOExpanderAddress::MemoryCommitExtras, MCP23x17Registers::IODIRB, value);
}
byte
ProcessorInterface::getPortZDirectionRegister() noexcept {
    return read8(IOExpanderAddress::MemoryCommitExtras, MCP23x17Registers::IODIRB);
}
void ProcessorInterface::setPortZPolarityRegister(byte value) noexcept {
    write8(IOExpanderAddress::MemoryCommitExtras, MCP23x17Registers::IPOLB, value);
}
byte ProcessorInterface::getPortZPolarityRegister() noexcept {
    return read8(IOExpanderAddress::MemoryCommitExtras, MCP23x17Registers::IPOLB);
}
void ProcessorInterface::setPortZPullupResistorRegister(byte value) noexcept {
    write8(IOExpanderAddress::MemoryCommitExtras, MCP23x17Registers::GPPUB, value);
}
byte ProcessorInterface::getPortZPullupResistorRegister() noexcept {
    return read8(IOExpanderAddress::MemoryCommitExtras, MCP23x17Registers::GPPUB);
}
byte ProcessorInterface::readPortZGPIORegister() noexcept {
    return read8(IOExpanderAddress::MemoryCommitExtras, MCP23x17Registers::GPIOB);
}
void ProcessorInterface::writePortZGPIORegister(byte value) noexcept {
    write8(IOExpanderAddress::MemoryCommitExtras, MCP23x17Registers::GPIOB, value);
}

void ProcessorInterface::newDataCycle() noexcept {
    auto lower16Addr = static_cast<Address>(readGPIO16(ProcessorInterface::IOExpanderAddress::Lower16Lines));
    auto upper16Addr = static_cast<Address>(readGPIO16(ProcessorInterface::IOExpanderAddress::Upper16Lines)) << 16;
    address_ = lower16Addr | upper16Addr;
    upperMaskedAddress_ = 0xFFFF'FFF0 & address_;
    blastAsserted_ = DigitalPin<i960Pinout::BLAST_>::isAsserted();
}
void ProcessorInterface::updateDataCycle() noexcept {
#ifdef ARDUINO_AVR_ATmega1284
    auto bits = PINA;
    auto byteEnableBits = static_cast<byte>(bits & 0b1110);
    burstAddressBits_ = byteEnableBits >> 1;
    lss_ = static_cast<LoadStoreStyle>((bits & 0b110000) >> 4);
    address_ = upperMaskedAddress_ | byteEnableBits;
    blastAsserted_ = (bits & 0b0100'0000) == 0;
#else
#warning "USING IO EXPANDER TO READ CONTROL BITS!!!"
    // leave this around for targets with fewer pins
    auto bits = read16(IOExpanderAddress::MemoryCommitExtras, MCP23x17Registers::GPIOA);
    lss_ = static_cast<LoadStoreStyle>(static_cast<byte>((bits & 0b11000) >> 3));
    address_ = upperMaskedAddress_ | static_cast<byte>((bits & 0b111) << 1);
    burstAddressBits_ = static_cast<byte>(bits & 0b111);
    blastAsserted_ = DigitalPin<i960Pinout::BLAST_>::isAsserted();
#endif
}
