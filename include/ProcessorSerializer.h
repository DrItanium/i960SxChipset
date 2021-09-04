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
    static uint16_t read16() noexcept {
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
    template<IOExpanderAddress addr, MCP23x17Registers opcode, bool standalone = true, bool disableInterrupts = true>
    static uint8_t read8() noexcept {
        if constexpr (standalone) {
            SPI.beginTransaction(SPISettings(TargetBoard::runIOExpanderSPIInterfaceAt(), MSBFIRST, SPI_MODE0));
        }
        digitalWrite<i960Pinout::GPIOSelect, LOW, disableInterrupts>();
        SPI.transfer(generateReadOpcode(addr));
        SPI.transfer(static_cast<byte>(opcode));
        auto lower = SPI.transfer(0);
        digitalWrite<i960Pinout::GPIOSelect, HIGH, disableInterrupts>();
        if constexpr (standalone) {
            SPI.endTransaction();
        }
        return lower;
    }

    template<IOExpanderAddress addr, MCP23x17Registers opcode, bool standalone = true, bool disableInterrupts = true>
    static void write16(uint16_t value) noexcept {
        SplitWord16 valueDiv(value);
        if constexpr (standalone) {
            SPI.beginTransaction(SPISettings(TargetBoard::runIOExpanderSPIInterfaceAt(), MSBFIRST, SPI_MODE0));
        }
        digitalWrite<i960Pinout::GPIOSelect, LOW, disableInterrupts>();
        SPI.transfer(generateWriteOpcode(addr));
        SPI.transfer(static_cast<byte>(opcode));
        SPI.transfer(valueDiv.bytes[0]);
        SPI.transfer(valueDiv.bytes[1]);
        digitalWrite<i960Pinout::GPIOSelect, HIGH, disableInterrupts>();
        if constexpr (standalone) {
            SPI.endTransaction();
        }
    }
    template<IOExpanderAddress addr, MCP23x17Registers opcode, bool standalone = true, bool disableInterrupts = true>
    static void write8(uint8_t value) noexcept {
        if constexpr (standalone) {
            SPI.beginTransaction(SPISettings(TargetBoard::runIOExpanderSPIInterfaceAt(), MSBFIRST, SPI_MODE0));
        }
        digitalWrite<i960Pinout::GPIOSelect, LOW, disableInterrupts>();
        SPI.transfer(generateWriteOpcode(addr));
        SPI.transfer(static_cast<byte>(opcode));
        SPI.transfer(value);
        digitalWrite<i960Pinout::GPIOSelect, HIGH, disableInterrupts>();
        if constexpr (standalone) {
            SPI.endTransaction();
        }
    }
    template<IOExpanderAddress addr, bool standalone = true, bool disableInterrupts = true>
    static inline uint16_t readGPIO16() noexcept {
        return read16<addr, MCP23x17Registers::GPIO, standalone, disableInterrupts>();
    }
    template<IOExpanderAddress addr, bool standalone = true, bool disableInterrupts = true>
    static inline void writeGPIO16(uint16_t value) noexcept {
        write16<addr, MCP23x17Registers::GPIO, standalone, disableInterrupts>(value);
    }
    template<IOExpanderAddress addr, bool standalone = true, bool disableInterrupts = true>
    static inline void writeDirection(uint16_t value) noexcept {
        write16<addr, MCP23x17Registers::IODIR, standalone, disableInterrupts>(value);
    }
public:
    ProcessorInterface() = delete;
    ~ProcessorInterface() = delete;
    ProcessorInterface(const ProcessorInterface&) = delete;
    ProcessorInterface(ProcessorInterface&&) = delete;
    ProcessorInterface& operator=(const ProcessorInterface&) = delete;
    ProcessorInterface& operator=(ProcessorInterface&&) = delete;
public:
// layout of the extra memory commit expander
// PA0 - BurstAddress1 - input
// PA1 - BurstAddress2 - input
// PA2 - BurstAddress3 - input
// PA3 - BE0_ - input
// PA4 - BE1_ - input
// PA5 - HOLD  - output
// PA6 - HLDA  - input
// PA7 - _LOCK - output
// PB0-PB7 - Unused

    enum class ExtraGPIOExpanderPinout : decltype(A0) {
        BurstAddress1,
        BurstAddress2,
        BurstAddress3,
        ByteEnable0,
        ByteEnable1,
        HOLD,
        HLDA,
        LOCK_,
        // add support for upto 256 spi devices
        Unused0,
        Unused1,
        Unused2,
        Unused3,
        Unused4,
        Unused5,
        Unused6,
        Unused7,
        Count,
    };
    static_assert(static_cast<int>(ExtraGPIOExpanderPinout::Count) == 16);
public:
    static void begin() noexcept;
    [[nodiscard]] static constexpr Address getAddress() noexcept { return address_.getWholeValue(); }
    [[nodiscard]] static uint16_t getDataBits() noexcept;
    static void setDataBits(uint16_t value) noexcept;
    [[nodiscard]] static auto getStyle() noexcept { return static_cast<LoadStoreStyle>(lss_); }
    [[nodiscard]] static auto getCacheOffsetEntry() noexcept { return cacheOffsetEntry_; }
    static void setHOLDPin(bool value) noexcept {
        if (value != holdValue_) {
            holdValue_ = value;
            updateOutputLatch();
        }
    }
    static void setLOCKPin(bool value) noexcept {
        if (value != lockValue_) {
            lockValue_ = value;
            updateOutputLatch();
        }
    }
private:
    static void updateOutputLatch() noexcept {
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
public:
    static byte newDataCycle() noexcept;
    static void burstNext() noexcept {
        // this is a subset of actions, we just need to read the byte enable bits continuously and advance the address by two to get to the
        // next 16-bit word
        // don't increment everything just the lowest byte since we will never actually span 16 byte segments in a single burst transaction
        address_.bytes[0] += 2;
        // make sure that we always have an up to date copy of the cache offset entry
        computeInitialCacheOffset();
    }
    static void computeInitialCacheOffset() noexcept {
        lss_ = static_cast<LoadStoreStyle>((PINA & 0b110000));
        cacheOffsetEntry_ = address_.bytes[0] >> 1; // we want to make this quick to increment
    }
private:
    static inline uint16_t dataLinesDirection_ = 0xFFFF;
    static inline SplitWord32 address_{0};
    static inline LoadStoreStyle lss_ = LoadStoreStyle::None;
    static inline bool lockValue_ = true;
    static inline bool holdValue_ = false;
    static inline byte cacheOffsetEntry_ = 0;
    static inline uint16_t latchedDataOutput = 0;
};
// 8 IOExpanders to a single enable line for SPI purposes
// 4 of them are reserved

#endif //ARDUINO_IOEXPANDERS_H
