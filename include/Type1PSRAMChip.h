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
// Created by jwscoggins on 12/9/21.
//

#ifndef SXCHIPSET_TYPE1PSRAMCHIP_H
#define SXCHIPSET_TYPE1PSRAMCHIP_H
#ifdef CHIPSET_TYPE1
#include <Arduino.h>
#include <SPI.h>
#include "MCUPlatform.h"
#include "Pinout.h"
#include "TaggedCacheAddress.h"
/**
 * @brief Interface to the memory connected to the chipset
 * @tparam enablePin The pin that is used to signal chip usage
 * @tparam sel0 The lowest pin used to select the target device
 * @tparam sel1 The middle pin used to select the target device
 * @tparam sel2 The upper pin used to select the target device
 */
template<i960Pinout enablePin>
class MemoryBlock {
public:
    static constexpr auto EnablePin = enablePin;
    static constexpr auto Select0 = i960Pinout::SPI_OFFSET0;
    static constexpr auto Select1 = i960Pinout::SPI_OFFSET1;
    static constexpr auto Select2 = i960Pinout::SPI_OFFSET2;
    static constexpr auto NumChips = 8;
    static_assert ((EnablePin != Select0) && (EnablePin != Select1) && (EnablePin != Select2), "The enable pin must be different from all select pins");
    static_assert ((Select0 != Select1) && (Select0 != Select2) && (Select1 != Select2), "All three select pins must point to a different physical pin");
public:
    MemoryBlock() = delete;
    ~MemoryBlock() = delete;
    union PSRAMBlockAddress {
        constexpr explicit PSRAMBlockAddress(Address value = 0) : base(value) { }
        constexpr auto getAddress() const noexcept { return base; }
        constexpr auto getOffset() const noexcept { return offset; }
        constexpr auto getIndex() const noexcept { return index; }
        Address base;
        struct {
            Address offset : 23;
            byte index : 3;
        };
        byte bytes_[4];
    };
private:
    enum class InterleavedMultibyteTransmissionStyle : byte {
        Mirrored = 0,
        SPI0_SPI1,
        SPI1_SPI0,
        Count, // must be last
    };
    template<InterleavedMultibyteTransmissionStyle style>
    static constexpr bool ValidTransmissionStyle_v = static_cast<byte>(style) < static_cast<byte>(InterleavedMultibyteTransmissionStyle::Count);
    static inline void transmitByte(byte value) noexcept {
        SPDR = value;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait
    }
    static inline byte mspimTransfer(byte value) noexcept {
        while (!(UCSR1A & (1 << UDRE1)));
        UDR1 = value;
        while (!(UCSR1A & (1 << RXC1)));
        return UDR1;
    }
    static inline void transmitByte_Bus2(byte value) noexcept {
        UDR1 = value;
        asm volatile ("nop");
        while (!(UCSR1A & (1 << UDRE1)));
    }
    static inline void transmitByte_Interleaved(byte value) noexcept {
        UDR1 = value;
        SPDR = value;
        asm volatile ("nop");
        while (!(UCSR1A & (1 << UDRE1)));
        while (!(SPSR & (1 << SPIF)));
    }
    static inline void transmitTwoByte(uint8_t first, uint8_t second) noexcept {
        SPDR = first;
        asm volatile ("nop");
        while (!(SPSR & _BV(SPIF)));
        SPDR = second;
        asm volatile ("nop");
        while (!(SPSR & _BV(SPIF)));
    }
    static inline void transmitTwoByte_Bus2(uint8_t first, uint8_t second) noexcept {
        UDR1 = first;
        UDR1 = second;
        asm volatile ("nop");
        while (!(UCSR1A & (1 << UDRE1)));
    }
    template<InterleavedMultibyteTransmissionStyle style>
    static inline void transmitTwoByte_Interleaved(uint8_t first, uint8_t second) noexcept {
        static_assert(ValidTransmissionStyle_v<style>, "Unimplemented or illegal transmission style specified");
        if constexpr (style == InterleavedMultibyteTransmissionStyle::Mirrored) {
            UDR1 = first;
            SPDR = first;
            UDR1 = second;
            asm volatile ("nop");
            while (!(SPSR & _BV(SPIF)));
            SPDR = second;
            asm volatile ("nop");
            while (!(SPSR & _BV(SPIF)));
            while (!(UCSR1A & (1 << UDRE1)));
        } else if constexpr (style == InterleavedMultibyteTransmissionStyle::SPI0_SPI1) {
            SPDR = first;
            UDR1 = second;
            asm volatile ("nop");
            while (!(SPSR & _BV(SPIF)));
            while (!(UCSR1A & (1 << UDRE1)));
        } else if constexpr (style == InterleavedMultibyteTransmissionStyle::SPI1_SPI0) {
            UDR1 = first;
            SPDR = second;
            asm volatile ("nop");
            while (!(SPSR & _BV(SPIF)));
            while (!(UCSR1A & (1 << UDRE1)));
        } else {
            // do nothing
        }
    }
    enum class OperationKind {
        Write,
        Read,
        Generic,
    };

    template<byte opcode, OperationKind kind = OperationKind::Generic>
    inline static size_t genericReadWriteOperation(uint32_t address, byte* buf, size_t capacity) noexcept {
        if (capacity == 0) {
            return 0;
        }
        PSRAMBlockAddress curr(address);
        setChipId(curr.getIndex());
        SPI.beginTransaction(SPISettings(TargetBoard::runPSRAMAt(), MSBFIRST, SPI_MODE0));
        digitalWrite<EnablePin, LOW>();
        SPDR = opcode;
        asm volatile("nop");
        PSRAMBlockAddress end(address + capacity);
        while (!(SPSR & _BV(SPIF))) ; // wait
        SPDR = curr.bytes_[2];
        asm volatile("nop");
        auto numBytesToSecondChip = end.getOffset();
        while (!(SPSR & _BV(SPIF))) ; // wait
        SPDR = curr.bytes_[1];
        asm volatile("nop");
        auto localToASingleChip = curr.getIndex() == end.getIndex();
        while (!(SPSR & _BV(SPIF))) ; // wait
        SPDR = curr.bytes_[0];
        asm volatile("nop");
        auto numBytesToFirstChip = localToASingleChip ? capacity : (capacity - numBytesToSecondChip);
        while (!(SPSR & _BV(SPIF))) ; // wait
        if constexpr (kind == OperationKind::Write) {
            for (decltype(numBytesToFirstChip) i = 0; i < numBytesToFirstChip; ++i) {
                transmitByte(buf[i]);
            }
        } else if constexpr (kind == OperationKind::Read) {
            for (decltype(numBytesToFirstChip) i = 0; i < numBytesToFirstChip; ++i) {
                transmitByte(0);
                buf[i] = SPDR;
            }
        } else {
            SPI.transfer(buf, numBytesToFirstChip);
        }
        digitalWrite<EnablePin, HIGH>();
        if (!localToASingleChip && (numBytesToSecondChip > 0)) {
            // since size_t is 16-bits on AVR we can safely reduce the largest buffer size 64k, thus we can only ever span two psram chips at a time
            // thus we can actually convert this work into two separate spi transactions
            // start writing at the start of the next chip the remaining number of bytes
            setChipId(end.getIndex());
            // we start at address zero on this new chip always
            digitalWrite<EnablePin, LOW>();
            transmitByte(opcode);
            transmitByte(0);
            transmitByte(0);
            transmitByte(0);
            if constexpr (kind == OperationKind::Write) {
                auto count = numBytesToSecondChip;
                auto actualBuf = buf + numBytesToFirstChip;
                for (decltype(count) i = 0; i < count; ++i) {
                    transmitByte(actualBuf[i]);
                }
            } else if (kind == OperationKind::Read) {
                auto count = numBytesToSecondChip;
                auto actualBuf = buf + numBytesToFirstChip;
                for (decltype(count) i = 0; i < count; ++i) {
                    transmitByte(0);
                    actualBuf[i] = SPDR;
                }
            } else {
                SPI.transfer(buf + numBytesToFirstChip, numBytesToSecondChip);
            }
            digitalWrite<EnablePin, HIGH>();
        }
        SPI.endTransaction();
        return capacity;
    }
public:
    static size_t write(uint32_t address, byte *buf, size_t capacity) noexcept {
        return genericReadWriteOperation<0x02, OperationKind::Write>(address, buf, capacity);
    }
    static size_t read(uint32_t address, byte *buf, size_t capacity) noexcept {
        return genericReadWriteOperation<0x03, OperationKind::Read>(address, buf, capacity);
    }
private:
#ifdef CHIPSET_TYPE1
    template<byte index>
    static void doWrites() noexcept {
        digitalWrite<Select0, (index & (1 << 0)) ? HIGH : LOW>();
        digitalWrite<Select1, (index & (1 << 1)) ? HIGH : LOW>();
        digitalWrite<Select2, (index & (1 << 2)) ? HIGH : LOW>();
    }
#endif
    static void setChipId(byte index) noexcept {
#ifdef CHIPSET_TYPE1
        using Action = void(*)();
        static constexpr Action Operations[8] {
            doWrites<0>, doWrites<1>, doWrites<2>, doWrites<3>,
            doWrites<4>, doWrites<5>, doWrites<6>, doWrites<7>,
        };
        if (index != currentIndex_) {
            Operations[index & 0b111]();
            currentIndex_ = index & 0b111;
        }
#endif
    }
public:
    static void begin() noexcept {
        static bool initialized_ = false;
        if (!initialized_) {
            initialized_ = true;
            setChipId(0);
            SPI.beginTransaction(SPISettings(TargetBoard::runPSRAMAt(), MSBFIRST, SPI_MODE0));
            for (int i = 0; i < NumChips; ++i) {
                setChipId(i);
                delayMicroseconds(200); // give the psram enough time to come up regardless of where you call begin
                digitalWrite<enablePin, LOW>();
                SPI.transfer(0x66);
                digitalWrite<enablePin, HIGH>();
                asm volatile ("nop");
                asm volatile ("nop");
                digitalWrite<enablePin, LOW>();
                SPI.transfer(0x99);
                digitalWrite<enablePin, HIGH>();
            }
            SPI.endTransaction();
        }
    }
private:
#ifdef CHIPSET_TYPE1
    static inline byte currentIndex_ = 0xFF;
#endif
};

using OnboardPSRAMBlock = MemoryBlock<i960Pinout::PSRAM_EN>;
#endif
#endif //SXCHIPSET_TYPE1PSRAMCHIP_H
