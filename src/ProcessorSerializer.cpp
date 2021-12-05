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
            // enable HAEN and also set the mirror INTA/INTB bits
            write8<IOExpanderAddress::Lower16Lines, MCP23x17Registers::IOCON, false>(0b0100'1000) ;
            write8<IOExpanderAddress::Upper16Lines, MCP23x17Registers::IOCON, false>(0b0100'1000) ;
            write16<IOExpanderAddress::Lower16Lines, MCP23x17Registers::GPINTEN, false>(0xFFFF) ;
            write16<IOExpanderAddress::Upper16Lines, MCP23x17Registers::GPINTEN, false>(0xFFFF) ;
            write16<IOExpanderAddress::Lower16Lines, MCP23x17Registers::INTCON, false>(0x0000) ;
            write16<IOExpanderAddress::Upper16Lines, MCP23x17Registers::INTCON, false>(0x0000) ;
            writeDirection<IOExpanderAddress::DataLines, false>(0xFFFF);
            writeDirection<IOExpanderAddress::MemoryCommitExtras, false>(0x005F);
            // we can just set the pins up in a single write operation to the olat, since only the pins configured as outputs will be affected
            write8<IOExpanderAddress::MemoryCommitExtras, MCP23x17Registers::OLATA, false>(0b1000'0000);
            // write the default value out to the latch to start with
            write16<IOExpanderAddress::DataLines, MCP23x17Registers::OLAT, false>(latchedDataOutput.getWholeValue());
            //updateTargetFunctions<true>();
            //updateTargetFunctions<false>();
        } else {
            /// @todo implement this
        }
        SPI.endTransaction();
    }
}