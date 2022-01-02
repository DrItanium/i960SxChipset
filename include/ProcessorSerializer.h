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

#ifndef ARDUINO_IOEXPANDERS_H
#define ARDUINO_IOEXPANDERS_H
#include <Arduino.h>
#include <SPI.h>
#include "Pinout.h"
#include "i960SxChipset.h"

class ProcessorInterface {
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
    enum class IOExpanderAddress : byte {
        DataLines = 0b0000,
        Lower16Lines = 0b0010,
        Upper16Lines = 0b0100,
        MemoryCommitExtras = 0b0110,
        OtherDevice0 = 0b1000,
        OtherDevice1 = 0b1010,
        OtherDevice2 = 0b1100,
        OtherDevice3 = 0b1110,
    };
    static constexpr byte generateReadOpcode(ProcessorInterface::IOExpanderAddress address) noexcept {
        return 0b0100'0001 | static_cast<uint8_t>(address);
    }
    static constexpr byte generateWriteOpcode(ProcessorInterface::IOExpanderAddress address) noexcept {
        return 0b0100'0000 | static_cast<uint8_t>(address);
    }
    template<IOExpanderAddress addr, MCP23x17Registers opcode, bool standalone = true>
    static SplitWord16 read16() noexcept {
        if constexpr (standalone) {
            SPI.beginTransaction(SPISettings(TargetBoard::runIOExpanderSPIInterfaceAt(), MSBFIRST, SPI_MODE0));
        }
        SplitWord16 output(0);
        digitalWrite<i960Pinout::GPIOSelect, LOW>();
        SPDR = generateReadOpcode(addr);
        /*
         * The following NOP introduces a small delay that can prevent the wait
         * loop form iterating when running at the maximum speed. This gives
         * about 10% more speed, even if it seems counter-intuitive. At lower
         * speeds it is unnoticed.
         */
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait

        SPDR = static_cast<byte>(opcode) ;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait

        SPDR = 0;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait
        auto lower = SPDR;
        SPDR = 0;
        asm volatile("nop");
        {
            output.bytes[0] = lower;
        }
        while (!(SPSR & _BV(SPIF))) ; // wait
        output.bytes[1] = SPDR;
        digitalWrite<i960Pinout::GPIOSelect, HIGH>();
        if constexpr (standalone) {
            SPI.endTransaction();
        }
        return output;
    }
    template<IOExpanderAddress addr, MCP23x17Registers opcode, bool standalone = true>
    static uint8_t read8() noexcept {
        if constexpr (standalone) {
            SPI.beginTransaction(SPISettings(TargetBoard::runIOExpanderSPIInterfaceAt(), MSBFIRST, SPI_MODE0));
        }
        digitalWrite<i960Pinout::GPIOSelect, LOW>();
        SPDR = generateReadOpcode(addr);
        /*
         * The following NOP introduces a small delay that can prevent the wait
         * loop form iterating when running at the maximum speed. This gives
         * about 10% more speed, even if it seems counter-intuitive. At lower
         * speeds it is unnoticed.
         */
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait
        SPDR = static_cast<byte>(opcode);
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait
        SPDR = 0;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait
        digitalWrite<i960Pinout::GPIOSelect, HIGH>();
        if constexpr (standalone) {
            SPI.endTransaction();
        }
        return SPDR;
    }

    template<IOExpanderAddress addr, MCP23x17Registers opcode, bool standalone = true>
    static void write16(uint16_t value) noexcept {
        SplitWord16 valueDiv(value);
        if constexpr (standalone) {
            SPI.beginTransaction(SPISettings(TargetBoard::runIOExpanderSPIInterfaceAt(), MSBFIRST, SPI_MODE0));
        }
        digitalWrite<i960Pinout::GPIOSelect, LOW>();
        SPDR = generateWriteOpcode(addr);
        /*
         * The following NOP introduces a small delay that can prevent the wait
         * loop form iterating when running at the maximum speed. This gives
         * about 10% more speed, even if it seems counter-intuitive. At lower
         * speeds it is unnoticed.
         */
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait
        SPDR = static_cast<byte>(opcode);
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait
        SPDR = valueDiv.bytes[0];
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait
        SPDR = valueDiv.bytes[1];
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait
        digitalWrite<i960Pinout::GPIOSelect, HIGH>();
        if constexpr (standalone) {
            SPI.endTransaction();
        }
    }
    template<IOExpanderAddress addr, MCP23x17Registers opcode, bool standalone = true>
    static void write8(uint8_t value) noexcept {
        if constexpr (standalone) {
            SPI.beginTransaction(SPISettings(TargetBoard::runIOExpanderSPIInterfaceAt(), MSBFIRST, SPI_MODE0));
        }
        digitalWrite<i960Pinout::GPIOSelect, LOW>();
        SPDR = generateWriteOpcode(addr);
        /*
         * The following NOP introduces a small delay that can prevent the wait
         * loop form iterating when running at the maximum speed. This gives
         * about 10% more speed, even if it seems counter-intuitive. At lower
         * speeds it is unnoticed.
         */
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait
        SPDR = static_cast<byte>(opcode);
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait
        SPDR = value;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait
        digitalWrite<i960Pinout::GPIOSelect, HIGH>();
        if constexpr (standalone) {
            SPI.endTransaction();
        }
    }
    template<IOExpanderAddress addr, bool standalone = true>
    static inline SplitWord16 readGPIO16() noexcept {
        return read16<addr, MCP23x17Registers::GPIO, standalone>();
    }
    template<IOExpanderAddress addr, bool standalone = true>
    static inline void writeGPIO16(uint16_t value) noexcept {
        write16<addr, MCP23x17Registers::GPIO, standalone>(value);
    }
    template<IOExpanderAddress addr, bool standalone = true>
    static inline void writeDirection(uint16_t value) noexcept {
        write16<addr, MCP23x17Registers::IODIR, standalone>(value);
    }
public:
    ProcessorInterface() = delete;
    ~ProcessorInterface() = delete;
    ProcessorInterface(const ProcessorInterface&) = delete;
    ProcessorInterface(ProcessorInterface&&) = delete;
    ProcessorInterface& operator=(const ProcessorInterface&) = delete;
    ProcessorInterface& operator=(ProcessorInterface&&) = delete;
public:
    static void begin() noexcept {
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
            write16<ProcessorInterface::IOExpanderAddress::DataLines, MCP23x17Registers::IOCON, false>(0b0000'1000'0000'1000);
            write16<ProcessorInterface::IOExpanderAddress::Upper16Lines, MCP23x17Registers::IOCON, false>(0b0000'1000'0000'1000);
            write16<ProcessorInterface::IOExpanderAddress::Lower16Lines, MCP23x17Registers::IOCON, false>(0b0000'1000'0000'1000);
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
            } else if constexpr (TargetBoard::onAtmega1284p_Type2()) {
                writeDirection<IOExpanderAddress::Lower16Lines, false>(0xFFFF);
                writeDirection<IOExpanderAddress::Upper16Lines, false>(0xFFFF);
                writeDirection<IOExpanderAddress::DataLines, false>(0xFFFF);
                write16<IOExpanderAddress::Lower16Lines, MCP23x17Registers::GPINTEN, false>(0xFFFF) ;
                write16<IOExpanderAddress::Upper16Lines, MCP23x17Registers::GPINTEN, false>(0xFFFF) ;
                write16<IOExpanderAddress::Lower16Lines, MCP23x17Registers::INTCON, false>(0x0000) ;
                write16<IOExpanderAddress::Upper16Lines, MCP23x17Registers::INTCON, false>(0x0000) ;
                write16<IOExpanderAddress::DataLines, MCP23x17Registers::OLAT, false>(latchedDataOutput.getWholeValue());
                // use 16-bit versions to be on the safe side
                write16<IOExpanderAddress::Lower16Lines, MCP23x17Registers::IOCON, false>(0b0001'1000'0001'1000) ;
                // for some reason, independent interrupts for the upper 16-bits is a no go... unsure why so enable mirroring and reclaim one of the pins
                write16<IOExpanderAddress::Upper16Lines, MCP23x17Registers::IOCON, false>(0b0101'1000'0101'1000) ;
                // If I ever figure out why the upper 16 lines do not want to work with independent pins then we will activate this version
                //write16<IOExpanderAddress::Upper16Lines, MCP23x17Registers::IOCON, false>(0b0001'1000'0001'1000) ;
                // make sure we clear out any interrupt flags
            } else {
                /// @todo implement this
            }
            SPI.endTransaction();
        }
    }
    [[nodiscard]] static constexpr Address getAddress() noexcept { return address_.getWholeValue(); }
    [[nodiscard]] static SplitWord16 getDataBits() noexcept {
        if constexpr (TargetBoard::onAtmega1284p_Type1() || TargetBoard::onAtmega1284p_Type2()) {
            return readGPIO16<ProcessorInterface::IOExpanderAddress::DataLines>();
        } else {
            // stub out
            return SplitWord16(0);
        }
    }
    static void setDataBits(uint16_t value) noexcept {
        if constexpr (TargetBoard::onAtmega1284p_Type1() || TargetBoard::onAtmega1284p_Type2()) {
            // the latch is preserved in between data line changes
            // okay we are still pointing as output values
            // check the latch and see if the output value is the same as what is latched
            if (latchedDataOutput.getWholeValue() != value) {
                latchedDataOutput.wholeValue_ = value;
                writeGPIO16<ProcessorInterface::IOExpanderAddress::DataLines>(latchedDataOutput.getWholeValue());
            }
        } else {
            // do nothing
        }
    }
    [[nodiscard]] static auto getStyle() noexcept { return static_cast<LoadStoreStyle>((PINA & 0b11'0000)); }
    [[nodiscard]] static bool isReadOperation() noexcept { return DigitalPin<i960Pinout::W_R_>::isAsserted(); }
    [[nodiscard]] static auto getCacheOffsetEntry() noexcept { return cacheOffsetEntry_; }
    inline static void setupDataLinesForWrite() noexcept {
        if constexpr (TargetBoard::onAtmega1284p_Type1() || TargetBoard::onAtmega1284p_Type2()) {
            if (!dataLinesDirection_) {
                dataLinesDirection_ = ~dataLinesDirection_;
                writeDirection<ProcessorInterface::IOExpanderAddress::DataLines>(0xFFFF);
            }
        } else {
            // do nothing
        }
    }
    inline static void setupDataLinesForRead() noexcept {
        if constexpr (TargetBoard::onAtmega1284p_Type1() || TargetBoard::onAtmega1284p_Type2()) {
            if (dataLinesDirection_) {
                dataLinesDirection_ = ~dataLinesDirection_;
                writeDirection<ProcessorInterface::IOExpanderAddress::DataLines>(0);
            }
        } else {
            // do nothing
        }
    }
private:
    template<bool useInterrupts = true>
    static byte getUpdateKind() noexcept {
        if constexpr (!useInterrupts) {
            return 0;
        } else {
            if constexpr (TargetBoard::onAtmega1284p_Type1()) {
                switch (PIND & 0b1001'0000) {
                    case 0b0000'0000: return 0b0000;
                    case 0b0001'0000: return 0b0011;
                    case 0b1000'0000: return 0b1100;
                    case 0b1001'0000: return 0b1111;
                    default: return 0b0000;
                }
            } else if constexpr (TargetBoard::onAtmega1284p_Type2()) {
                // even though three of the four pins are actually in use, I want to eventually diagnose the problem itself
                // so this code is ready for that day
                return PINA & 0b0000'1111;
            } else {
                return 0;
            }
        }
    }
    template<byte offsetMask, bool inDebugMode>
    inline static void full32BitUpdate() noexcept {
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
            cacheOffsetEntry_ = (lowest >> 1) & offsetMask; // we want to make this quick to increment
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
        if (highest != address_.bytes[3]) {
            updateTargetFunctions<inDebugMode>(highest);
        }
        address_.bytes[3] = highest;
    }
    template<byte offsetMask>
    static void lower16Update() noexcept {
        constexpr auto Lower16Opcode = generateReadOpcode(ProcessorInterface::IOExpanderAddress::Lower16Lines);
        constexpr auto GPIOOpcode = static_cast<byte>(MCP23x17Registers::GPIO);
        // read only the lower half
        // we want to overlay actions as much as possible during spi transfers, there are blocks of waiting for a transfer to take place
        // where we can insert operations to take place that would otherwise be waiting
        digitalWrite<i960Pinout::GPIOSelect, LOW>();
        SPDR = Lower16Opcode;
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
            cacheOffsetEntry_ = (lowest >> 1) & offsetMask; // we want to make this quick to increment
            address_.bytes[0] = lowest;
        }
        while (!(SPSR & _BV(SPIF))); // wait
        address_.bytes[1] = SPDR;
        digitalWrite<i960Pinout::GPIOSelect, HIGH>();
    }
    template<bool inDebugMode>
    static void upper16Update() noexcept {
        constexpr auto Upper16Opcode = generateReadOpcode(ProcessorInterface::IOExpanderAddress::Upper16Lines);
        constexpr auto GPIOOpcode = static_cast<byte>(MCP23x17Registers::GPIO);
        // only read the upper 16-bits
        digitalWrite<i960Pinout::GPIOSelect, LOW>();
        SPDR = Upper16Opcode;
        asm volatile("nop");
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
        if (address_.bytes[3] != highest) {
            updateTargetFunctions<inDebugMode>(highest);
        }
        address_.bytes[3] = highest;
    }
    template<bool inDebugMode>
    static void updateHighest8() noexcept {
        constexpr auto Upper16Opcode = generateReadOpcode(ProcessorInterface::IOExpanderAddress::Upper16Lines);
        constexpr auto GPIOOpcode = static_cast<byte>(MCP23x17Registers::GPIOB);
        // only read the upper 8 bits
        digitalWrite<i960Pinout::GPIOSelect, HIGH>();
        SPDR = Upper16Opcode;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        SPDR = GPIOOpcode;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        SPDR = 0;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        auto highest = SPDR;
        digitalWrite<i960Pinout::GPIOSelect, LOW>();
        if (highest != address_.bytes[3]) {
            updateTargetFunctions<inDebugMode>(highest);
        }
        address_.bytes[3] = highest;
    }
    static void updateHigher8() noexcept {
        constexpr auto Upper16Opcode = generateReadOpcode(ProcessorInterface::IOExpanderAddress::Upper16Lines);
        constexpr auto GPIOOpcode = static_cast<byte>(MCP23x17Registers::GPIOA);
        // only read the upper 8 bits
        digitalWrite<i960Pinout::GPIOSelect, HIGH>();
        SPDR = Upper16Opcode;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        SPDR = GPIOOpcode;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        SPDR = 0;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        auto highest = SPDR;
        digitalWrite<i960Pinout::GPIOSelect, LOW>();
        address_.bytes[2] = highest;
    }
    template<byte offsetMask>
    static void updateLowest8() noexcept {
        constexpr auto Lower16Opcode = generateReadOpcode(ProcessorInterface::IOExpanderAddress::Lower16Lines);
        constexpr auto GPIOOpcode = static_cast<byte>(MCP23x17Registers::GPIOA);
        // read only the lower half
        // we want to overlay actions as much as possible during spi transfers, there are blocks of waiting for a transfer to take place
        // where we can insert operations to take place that would otherwise be waiting
        digitalWrite<i960Pinout::GPIOSelect, LOW>();
        SPDR = Lower16Opcode;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        SPDR = GPIOOpcode;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        SPDR = 0;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        auto lowest = SPDR;
        digitalWrite<i960Pinout::GPIOSelect, HIGH>();
        // inside of here we have access to 12 cycles to play with, so let's actually do some operations while we wait
        // put scope ticks to force the matter
        cacheOffsetEntry_ = (lowest >> 1) & offsetMask; // we want to make this quick to increment
        address_.bytes[0] = lowest;
    }
    static void updateLower8() noexcept {
        constexpr auto Lower16Opcode = generateReadOpcode(ProcessorInterface::IOExpanderAddress::Lower16Lines);
        constexpr auto GPIOOpcode = static_cast<byte>(MCP23x17Registers::GPIOB);
        // read only the lower half
        // we want to overlay actions as much as possible during spi transfers, there are blocks of waiting for a transfer to take place
        // where we can insert operations to take place that would otherwise be waiting
        digitalWrite<i960Pinout::GPIOSelect, LOW>();
        SPDR = Lower16Opcode;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        SPDR = GPIOOpcode;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        SPDR = 0;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        address_.bytes[1] = SPDR;
        digitalWrite<i960Pinout::GPIOSelect, HIGH>();
    }
private:
    template<bool inDebugMode>
    inline static void updateTargetFunctions(byte index) noexcept {
        if constexpr (inDebugMode) {
            lastReadDebug_ = getReadBody<inDebugMode>(index);
            lastWriteDebug_ = getWriteBody<inDebugMode>(index) ;
        }
        lastRead_ = getReadBody<inDebugMode>(index);
        lastWrite_ = getWriteBody<inDebugMode>(index) ;
    }
public:
    template<bool inDebugMode, byte offsetMask, bool useInterrupts = true>
    static void newDataCycle() noexcept {
        switch (getUpdateKind<useInterrupts>()) {
            case 0b0001:
                updateLower8();
                upper16Update<inDebugMode>();
                break;
            case 0b0010:
                updateLowest8<offsetMask>();
                upper16Update<inDebugMode>();
                break;
            case 0b0011:
                upper16Update<inDebugMode>();
                break;
            case 0b0100:
                lower16Update<offsetMask>();
                updateHighest8<inDebugMode>();
                break;
            case 0b0101:
                updateLower8();
                updateHighest8<inDebugMode>();
                break;
            case 0b0110:
                updateLowest8<offsetMask>();
                updateHighest8<inDebugMode>();
                break;
            case 0b0111:
                updateHighest8<inDebugMode>();
                break;
            case 0b1000:
                lower16Update<offsetMask>();
                updateHigher8();
                break;
            case 0b1001:
                updateHigher8();
                updateLower8();
                break;
            case 0b1010:
                updateHigher8();
                updateLowest8<offsetMask>();
                break;
            case 0b1011:
                updateHigher8();
                break;
            case 0b1100:
                lower16Update<offsetMask>();
                break;
            case 0b1101:
                updateLower8();
                break;
            case 0b1110:
                updateLowest8<offsetMask>();
                break;
            case 0b1111: break;
            default:
                full32BitUpdate<offsetMask, inDebugMode>();
                break;
        }
        if (isReadOperation()) {
            setupDataLinesForRead();
            if constexpr (inDebugMode) {
                lastReadDebug_();
            } else {
                lastRead_();
            }
        } else {
            setupDataLinesForWrite();
            if constexpr (inDebugMode) {
                lastWriteDebug_();
            } else {
                lastWrite_();
            }
        }
    }
    /**
     * @brief Return the least significant byte of the address, useful for CoreChipsetFeatures
     * @return The LSB of the address
     */
    [[nodiscard]] static auto getPageOffset() noexcept { return address_.bytes[0]; }
    [[nodiscard]] static auto getPageIndex() noexcept { return address_.bytes[1]; }

    template<typename CacheLine, bool inDebugMode>
    static inline void performCacheRead(const CacheLine& line) noexcept {
        for (auto offset = getCacheOffsetEntry(); ;++offset) {
            auto isLastRead = DigitalPin<i960Pinout::BLAST_>::isAsserted();
            setDataBits(line.get(offset));
            DigitalPin<i960Pinout::Ready>::pulse();
            if (isLastRead) {
                return;
            }
        }
    }
    template<typename CacheLine, bool inDebugMode>
    static inline void performCacheWrite(CacheLine& line) noexcept {
        for (auto offset = getCacheOffsetEntry(); ;++offset) {
            auto isLast = DigitalPin<i960Pinout::BLAST_>::isAsserted();
            line.set(offset, getStyle(), getDataBits());
            DigitalPin<i960Pinout::Ready>::pulse();
            if (isLast) {
                return;
            }
        }
    }
    template<typename T, bool inDebugMode>
    static inline void performExternalDeviceRead() noexcept {
        // this is a subset of actions, we just need to read the byte enable bits continuously and advance the address by two to get to the
        // next 16-bit word
        // don't increment everything just the lowest byte since we will never actually span 16 byte segments in a single burst transaction
        for (byte pageOffset = getPageOffset(); ;pageOffset += 2) {
            auto isLast = DigitalPin<i960Pinout::BLAST_>::isAsserted();
            auto result = T::read(getPageIndex(),
                                  pageOffset,
                                  getStyle());
            if constexpr (inDebugMode) {
                Serial.print(F("\tPage Index: 0x")) ;
                Serial.println(getPageIndex(), HEX);
                Serial.print(F("\tPage Offset: 0x")) ;
                Serial.println(pageOffset, HEX);
                Serial.print(F("\tRead Value: 0x"));
                Serial.println(result, HEX);
            }
            setDataBits(result);
            DigitalPin<i960Pinout::Ready>::pulse();
            if (isLast) {
                return;
            }
        }
    }
    template<typename T, bool inDebugMode>
    static inline void performExternalDeviceWrite() noexcept {
        // be careful of querying i960 state at this point because the chipset runs at twice the frequency of the i960
        // so you may still be reading the previous i960 cycle state!
        for (byte pageOffset = getPageOffset(); ; pageOffset += 2) {
            auto dataBits = getDataBits();
            auto isLast = DigitalPin<i960Pinout::BLAST_>::isAsserted();
            if constexpr (inDebugMode) {
                Serial.print(F("\tPage Index: 0x"));
                Serial.println(ProcessorInterface::getPageIndex(), HEX);
                Serial.print(F("\tPage Offset: 0x"));
                Serial.println(pageOffset, HEX);
                Serial.print(F("\tData To Write: 0x"));
                Serial.println(dataBits.getWholeValue(), HEX);
            }
            T::write(getPageIndex(),
                     pageOffset,
                     getStyle(),
                     dataBits);
            DigitalPin<i960Pinout::Ready>::pulse();
            if (isLast) {
                break;
            }
        }
    }
    template<bool inDebugMode>
    static inline void performFallbackRead() noexcept {
        do {
            auto isLast = DigitalPin<i960Pinout::BLAST_>::isAsserted();
            setDataBits(0);
            DigitalPin<i960Pinout::Ready>::pulse();
            // need to introduce some delay
            if (isLast) {
                break;
            }
        } while (true);
    }
    template<bool inDebugMode>
    static inline void performFallbackWrite() noexcept {
        do {
            // put four cycles worth of delay into this to make damn sure we are ready with the i960
            auto isLast = DigitalPin<i960Pinout::BLAST_>::isAsserted();
            __builtin_avr_nops(4);
            DigitalPin<i960Pinout::Ready>::pulse();
            // need to introduce some delay
            if (isLast) {
                break;
            }
        } while (true);
    }
public:
    static inline void setupMostRecentDispatchFunctions() noexcept {
        if (!initialDispatchFunctionsInitialized_) {
            initialDispatchFunctionsInitialized_ = true;
            // update all of the target functions at the same time on initial startup
            updateTargetFunctions<true>(0);
        }
    }
private:
    static inline SplitWord32 address_{0};
    static inline SplitWord16 latchedDataOutput {0};
    static inline byte dataLinesDirection_ = 0xFF;
    static inline byte cacheOffsetEntry_ = 0;
    static inline bool initialized_ = false;
    static inline bool initialDispatchFunctionsInitialized_ = false;
    static inline BodyFunction lastRead_ = nullptr;
    static inline BodyFunction lastReadDebug_ = nullptr;
    static inline BodyFunction lastWrite_ = nullptr;
    static inline BodyFunction lastWriteDebug_ = nullptr;
};
// 8 IOExpanders to a single enable line for SPI purposes
// 4 of them are reserved

#endif //ARDUINO_IOEXPANDERS_H
