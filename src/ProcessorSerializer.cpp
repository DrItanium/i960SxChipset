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
        return 0b0100'0001 | static_cast<uint8_t>(address);
    }
    constexpr byte generateWriteOpcode(ProcessorInterface::IOExpanderAddress address) noexcept {
        return 0b0100'0000 | static_cast<uint8_t>(address);
    }
    template<ProcessorInterface::IOExpanderAddress addr, MCP23x17Registers opcode, bool standalone = true>
    uint16_t read16() noexcept {
        if constexpr (standalone) {
            SPI.beginTransaction(SPISettings(TargetBoard::runIOExpanderSPIInterfaceAt(), MSBFIRST, SPI_MODE0));
        }
        digitalWrite<i960Pinout::GPIOSelect, LOW>();
        SPI.transfer(generateReadOpcode(addr));
        SPI.transfer(static_cast<byte>(opcode));
        auto lower = SPI.transfer(0);
        auto upper = SPI.transfer(0);
        digitalWrite<i960Pinout::GPIOSelect, HIGH>();
        if constexpr (standalone) {
            SPI.endTransaction();
        }
        return SplitWord16(lower, upper).wholeValue_;
    }
    template<ProcessorInterface::IOExpanderAddress addr, MCP23x17Registers opcode, bool standalone = true>
    uint8_t read8() {
        if constexpr (standalone) {
            SPI.beginTransaction(SPISettings(TargetBoard::runIOExpanderSPIInterfaceAt(), MSBFIRST, SPI_MODE0));
        }
        digitalWrite<i960Pinout::GPIOSelect, LOW>();
        SPI.transfer(generateReadOpcode(addr));
        SPI.transfer(static_cast<byte>(opcode));
        auto lower = SPI.transfer(0);
        digitalWrite<i960Pinout::GPIOSelect, HIGH>();
        if constexpr (standalone) {
            SPI.endTransaction();
        }
        return lower;
    }

    template<ProcessorInterface::IOExpanderAddress addr, MCP23x17Registers opcode, bool standalone = true>
    void write16(uint16_t value) {
        SplitWord16 valueDiv(value);
        if constexpr (standalone) {
            SPI.beginTransaction(SPISettings(TargetBoard::runIOExpanderSPIInterfaceAt(), MSBFIRST, SPI_MODE0));
        }
        digitalWrite<i960Pinout::GPIOSelect, LOW>();
        SPI.transfer(generateWriteOpcode(addr));
        SPI.transfer(static_cast<byte>(opcode));
        SPI.transfer(valueDiv.bytes[0]);
        SPI.transfer(valueDiv.bytes[1]);
        digitalWrite<i960Pinout::GPIOSelect, HIGH>();
        if constexpr (standalone) {
            SPI.endTransaction();
        }
    }
    template<ProcessorInterface::IOExpanderAddress addr, MCP23x17Registers opcode, bool standalone = true>
    void write8(uint8_t value) {
        if constexpr (standalone) {
            SPI.beginTransaction(SPISettings(TargetBoard::runIOExpanderSPIInterfaceAt(), MSBFIRST, SPI_MODE0));
        }
        digitalWrite<i960Pinout::GPIOSelect, LOW>();
        SPI.transfer(generateWriteOpcode(addr));
        SPI.transfer(static_cast<byte>(opcode));
        SPI.transfer(value);
        digitalWrite<i960Pinout::GPIOSelect, HIGH>();
        if constexpr (standalone) {
            SPI.endTransaction();
        }
    }
    template<ProcessorInterface::IOExpanderAddress addr, bool standalone = true>
    inline uint16_t readGPIO16() {
        return read16<addr, MCP23x17Registers::GPIO, standalone>();
    }
    template<ProcessorInterface::IOExpanderAddress addr, bool standalone = true>
    inline void writeGPIO16(uint16_t value) {
        write16<addr, MCP23x17Registers::GPIO, standalone>(value);
    }
    template<ProcessorInterface::IOExpanderAddress addr, bool standalone = true>
    inline void writeDirection(uint16_t value) {
        write16<addr, MCP23x17Registers::IODIR, standalone>(value);
    }
}
uint16_t
ProcessorInterface::getDataBits() noexcept {
    if (dataLinesDirection_ != 0xFFFF) {
        dataLinesDirection_ = 0xFFFF;
        writeDirection<ProcessorInterface::IOExpanderAddress::DataLines>(dataLinesDirection_);
    }
    return readGPIO16<ProcessorInterface::IOExpanderAddress::DataLines>();
}

void
ProcessorInterface::setDataBits(uint16_t value) noexcept {
    if (dataLinesDirection_ != 0) {
        dataLinesDirection_ = 0;
        writeDirection<ProcessorInterface::IOExpanderAddress::DataLines>(dataLinesDirection_);
    }
    // the latch is preserved in between data line changes
    // okay we are still pointing as output values
    // check the latch and see if the output value is the same as what is latched
    if (latchedDataOutput != value) {
        writeGPIO16<ProcessorInterface::IOExpanderAddress::DataLines>(value);
        latchedDataOutput = value;
    }
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
    write8<IOExpanderAddress::MemoryCommitExtras, MCP23x17Registers::OLATA>(latchValue);
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
    static bool initialized_ = false;
    if (!initialized_) {
        initialized_ = true;
        SPI.beginTransaction(SPISettings(TargetBoard::runIOExpanderSPIInterfaceAt(), MSBFIRST, SPI_MODE0));
        pinMode(i960Pinout::GPIOSelect, OUTPUT);
        digitalWrite<i960Pinout::GPIOSelect, HIGH>();
        // at bootup, the IOExpanders all respond to 0b000 because IOCON.HAEN is
        // disabled. We can send out a single IOCON.HAEN enable message and all
        // should receive it.
        // so do a begin operation on all chips (0b000)
        // set IOCON.HAEN on all chips
        auto iocon = read8<ProcessorInterface::IOExpanderAddress::DataLines, MCP23x17Registers::IOCON, false>();
        write8<ProcessorInterface::IOExpanderAddress::DataLines, MCP23x17Registers::IOCON, false>(iocon | 0b0000'1000);
        // now all devices tied to this ~CS pin have separate addresses
        // make each of these inputs
        writeDirection<IOExpanderAddress::Lower16Lines, false>(0xFFFF);
        writeDirection<IOExpanderAddress::Upper16Lines, false>(0xFFFF);
        writeDirection<IOExpanderAddress::DataLines, false>(dataLinesDirection_);
        writeDirection<IOExpanderAddress::MemoryCommitExtras, false>(0x005F);
        // we can just set the pins up in a single write operation to the olat, since only the pins configured as outputs will be affected
        write8<IOExpanderAddress::MemoryCommitExtras, MCP23x17Registers::OLATA, false>(0b1000'0000);
        // write the default value out to the latch to start with
        write16<IOExpanderAddress::DataLines, MCP23x17Registers::OLAT, false>(latchedDataOutput);
        SPI.endTransaction();
    }
}

byte
ProcessorInterface::newDataCycle() noexcept {
    constexpr auto Lower16Opcode = generateReadOpcode(ProcessorInterface::IOExpanderAddress::Lower16Lines);
    constexpr auto Upper16Opcode = generateReadOpcode(ProcessorInterface::IOExpanderAddress::Upper16Lines);
    constexpr auto GPIOOpcode = static_cast<byte>(MCP23x17Registers::GPIO);
    SPI.beginTransaction(SPISettings(TargetBoard::runIOExpanderSPIInterfaceAt(), MSBFIRST, SPI_MODE0));
    digitalWrite<i960Pinout::GPIOSelect, LOW>();
    SPI.transfer(Lower16Opcode);
    SPI.transfer(GPIOOpcode);
    address_.bytes[0] = SPI.transfer(0);
    address_.bytes[1] = SPI.transfer(0);
    DigitalPin<i960Pinout::GPIOSelect>::pulse(); // go high then low again without having to capture the interrupt state multiple times
    SPI.transfer(Upper16Opcode);
    SPI.transfer(GPIOOpcode);
    address_.bytes[2] = SPI.transfer(0);
    address_.bytes[3] = SPI.transfer(0);
    digitalWrite<i960Pinout::GPIOSelect, HIGH>();
    SPI.endTransaction();
    // no need to re-read the burst address bits
    return address_.bytes[3];
}

void
ProcessorInterface::computeInitialCacheOffset() noexcept {
    lss_ = static_cast<LoadStoreStyle>((PINA & 0b110000));
    cacheOffsetEntry_ = address_.bytes[0] >> 1; // we want to make this quick to increment
}

void
ProcessorInterface::burstNext() noexcept {
    // this is a subset of actions, we just need to read the byte enable bits continuously and advance the address by two to get to the
    // next 16-bit word
    // don't increment everything just the lowest byte since we will never actually span 16 byte segments in a single burst transaction
    address_.bytes[0] += 2;
    // make sure that we always have an up to date copy of the cache offset entry
    computeInitialCacheOffset();
}
