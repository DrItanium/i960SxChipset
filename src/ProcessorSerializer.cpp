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
#include "Pinout.h"
//#include "PSRAMChip.h"
SplitWord16
ProcessorInterface::getDataBits() noexcept {
    if constexpr (TargetBoard::onAtmega1284p_Type1()) {
        return readGPIO16<ProcessorInterface::IOExpanderAddress::DataLines>();
    } else {
        // stub out
        return SplitWord16(0);
    }
}
void
ProcessorInterface::setDataBits(uint16_t value) noexcept {
    if constexpr (TargetBoard::onAtmega1284p_Type1()) {
        // the latch is preserved in between data line changes
        // okay we are still pointing as output values
        // check the latch and see if the output value is the same as what is latched
        if (latchedDataOutput != value) {
            latchedDataOutput = value;
            writeGPIO16<ProcessorInterface::IOExpanderAddress::DataLines>(latchedDataOutput);
        }
    } else {
       // do nothing
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
        if constexpr (TargetBoard::onAtmega1284p_Type1()) {
            // now all devices tied to this ~CS pin have separate addresses
            // make each of these inputs
            writeDirection<IOExpanderAddress::Lower16Lines, false>(0xFFFF);
            writeDirection<IOExpanderAddress::Upper16Lines, false>(0xFFFF);
            writeDirection<IOExpanderAddress::DataLines, false>(0xFFFF);
            writeDirection<IOExpanderAddress::MemoryCommitExtras, false>(0x005F);
            // we can just set the pins up in a single write operation to the olat, since only the pins configured as outputs will be affected
            write8<IOExpanderAddress::MemoryCommitExtras, MCP23x17Registers::OLATA, false>(0b1000'0000);
            // write the default value out to the latch to start with
            write16<IOExpanderAddress::DataLines, MCP23x17Registers::OLAT, false>(latchedDataOutput);
        } else {
            /// @todo implement this
        }
        SPI.endTransaction();
    }
}

byte
ProcessorInterface::newDataCycle() noexcept {
    if constexpr (TargetBoard::onAtmega1284p_Type1()) {
        constexpr auto Lower16Opcode = generateReadOpcode(ProcessorInterface::IOExpanderAddress::Lower16Lines);
        constexpr auto Upper16Opcode = generateReadOpcode(ProcessorInterface::IOExpanderAddress::Upper16Lines);
        constexpr auto GPIOOpcode = static_cast<byte>(MCP23x17Registers::GPIO);
        // we want to overlay actions as much as possible during spi transfers, there are blocks of waiting for a transfer to take place
        // where we can insert operations to take place that would otherwise be waiting
        digitalWrite<i960Pinout::GPIOSelect, LOW>();
        SPDR = Lower16Opcode;
        /*
         * The following NOP introduces a small delay that can prevent the wait
         * loop form iterating when running at the maximum speed. This gives
         * about 10% more speed, even if it seems counter-intuitive. At lower
         * speeds it is unnoticed.
         */
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        SPDR = GPIOOpcode;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        SPDR = 0;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        auto lowest = SPDR;
        SPDR = 0;
        asm volatile("nop");
        {
            // inside of here we have access to 12 cycles to play with, so let's actually do some operations while we wait
            // put scope ticks to force the matter
            cacheOffsetEntry_ = (lowest >> 1) & 0b0000'0111; // we want to make this quick to increment
            address_.bytes[0] = lowest;
        }
        while (!(SPSR & _BV(SPIF))); // wait
        auto lower = SPDR;
        DigitalPin<i960Pinout::GPIOSelect>::pulse<HIGH>(); // pulse high
        SPDR = Upper16Opcode;
        asm volatile("nop");
        {
            address_.bytes[1] = lower;
            // interleave this operation in, can't get more complex than this
        }
        while (!(SPSR & _BV(SPIF))); // wait
        SPDR = GPIOOpcode;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        SPDR = 0;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        auto higher = SPDR;
        SPDR = 0;
        asm volatile("nop");
        {
            address_.bytes[2] = higher;
        }
        while (!(SPSR & _BV(SPIF))); // wait
        auto highest = SPDR;
        digitalWrite<i960Pinout::GPIOSelect, HIGH>();
        address_.bytes[3] = highest;
        if (isReadOperation()) {
            setupDataLinesForRead();
        } else {
            setupDataLinesForWrite();
        }
        return highest;
    } else {
        return 0;
    }
}
void
ProcessorInterface::setupDataLinesForWrite() noexcept {
    if constexpr (TargetBoard::onAtmega1284p_Type1()) {
        if (!dataLinesDirection_) {
            dataLinesDirection_ = ~dataLinesDirection_;
            writeDirection<ProcessorInterface::IOExpanderAddress::DataLines>(0xFFFF);
        }
    } else {
        // do nothing
    }
}
void
ProcessorInterface::setupDataLinesForRead() noexcept {
    if constexpr (TargetBoard::onAtmega1284p_Type1()) {
        if (dataLinesDirection_) {
            dataLinesDirection_ = ~dataLinesDirection_;
            writeDirection<ProcessorInterface::IOExpanderAddress::DataLines>(0);
        }
    } else {
        // do nothing
    }
}

void
ProcessorInterface::triggerInt0() noexcept {
#ifdef CHIPSET_TYPE1
    pulse<i960Pinout::Int0_>();
#elif defined(CHIPSET_TYPE3)
    /// @todo implement this
#endif
}