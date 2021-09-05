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
uint16_t
ProcessorInterface::getDataBits() noexcept {
    return readGPIO16<ProcessorInterface::IOExpanderAddress::DataLines>();
}
void
ProcessorInterface::setDataBits(uint16_t value) noexcept {
    // the latch is preserved in between data line changes
    // okay we are still pointing as output values
    // check the latch and see if the output value is the same as what is latched
    if (latchedDataOutput != value) {
        latchedDataOutput = value;
        writeGPIO16<ProcessorInterface::IOExpanderAddress::DataLines>(latchedDataOutput);
    }
}





void
ProcessorInterface::begin() noexcept {
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
    digitalWrite<i960Pinout::GPIOSelect, LOW>();
    SPI.transfer(Lower16Opcode);
    SPI.transfer(GPIOOpcode);
    address_.bytes[0] = SPI.transfer(0);
    address_.bytes[1] = SPI.transfer(0);
    DigitalPin<i960Pinout::GPIOSelect>::pulse<HIGH>(); // pulse high
    SPI.transfer(Upper16Opcode);
    SPI.transfer(GPIOOpcode);
    address_.bytes[2] = SPI.transfer(0);
    address_.bytes[3] = SPI.transfer(0);
    digitalWrite<i960Pinout::GPIOSelect, HIGH>();
    // no need to re-read the burst address bits
    return address_.bytes[3];
}
void
ProcessorInterface::setupDataLinesForWrite() noexcept {
    if (dataLinesDirection_ != 0xFFFF) {
        dataLinesDirection_ = 0xFFFF;
        writeDirection<ProcessorInterface::IOExpanderAddress::DataLines>(dataLinesDirection_);
    }
}
void
ProcessorInterface::setupDataLinesForRead() noexcept {
    if (dataLinesDirection_ != 0) {
        dataLinesDirection_ = 0;
        writeDirection<ProcessorInterface::IOExpanderAddress::DataLines>(dataLinesDirection_);
    }
}