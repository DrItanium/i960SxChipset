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

#ifndef SXCHIPSET_DUALSPIBUSPSRAMCHIP_H
#define SXCHIPSET_DUALSPIBUSPSRAMCHIP_H
#ifdef CHIPSET_TYPE2
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
class Type2MemoryBlock {
public:
    static constexpr auto EnablePin = i960Pinout::PSRAM_EN;
    static constexpr auto EnablePin1 = i960Pinout::PSRAM_EN1;
    static constexpr auto NumChips = 2;
public:
    Type2MemoryBlock() = delete;
    ~Type2MemoryBlock() = delete;
    union PSRAMBlockAddress {
        constexpr explicit PSRAMBlockAddress(Address value = 0) : base(value) { }
        constexpr auto getAddress() const noexcept { return base; }
        constexpr auto getOffset() const noexcept { return offset; }
        constexpr auto getIndex() const noexcept { return index; }
        Address base;
        union {
            Address offset: 24;
            byte index : 2; // we have blocks of 16 megabytes instead of blocks of 8 megabytes found in type 1
        };
        byte bytes_[4];
    };
private:
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
    static inline SplitWord16 dualTransfer(byte lower, byte upper) noexcept {
        while (!(UCSR1A & (1 << UDRE1)));
        SPDR = lower;
        UDR1 = upper;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait
        while (!(UCSR1A & (1 << RXC1)));
        return SplitWord16{SPDR, UDR1};
    }
    static inline SplitWord16 dualTransferMirrored(byte value) noexcept { return dualTransfer(value, value); }
    enum class OperationKind {
        Write,
        Read,
    };

    template<byte opcode, OperationKind kind>
    inline static size_t genericReadWriteOperation(uint32_t address, byte* buf, size_t capacity) noexcept {
        if (capacity == 0) {
            return 0;
        }
        PSRAMBlockAddress curr(address);
        SPI.beginTransaction(SPISettings(TargetBoard::runPSRAMAt(), MSBFIRST, SPI_MODE0));
        //(void)UDR1;
        //(void)UDR1;
        PSRAMBlockAddress end(address + capacity);
        auto numBytesToSecondChip = end.getOffset();
        auto localToASingleChip = curr.getIndex() == end.getIndex();
        auto numBytesToFirstChip = localToASingleChip ? capacity : (capacity - numBytesToSecondChip);
        digitalWrite<EnablePin, LOW>();
        digitalWrite<EnablePin1, LOW>();
        (void)dualTransferMirrored(opcode);
        (void)dualTransferMirrored(curr.bytes_[2]);
        (void)dualTransferMirrored(curr.bytes_[1]);
        (void)dualTransferMirrored(curr.bytes_[0]);
        // okay so when we are using the separate SPI bus we need to interleaved operations, we are interleaving bytes
        // so even bytes go to bus0 and odd bytes go to bus1
        if constexpr (kind == OperationKind::Write) {
            for (decltype(numBytesToFirstChip) i = 0; i < numBytesToFirstChip; i+=2) {
                // interleave bytes, that way it is easier to access this stuff
                (void)dualTransfer(buf[i], buf[i+1]);
            }
        } else if constexpr (kind == OperationKind::Read) {
            for (decltype(numBytesToFirstChip) i = 0; i < numBytesToFirstChip; i+=2) {
                auto result = dualTransferMirrored(0);
                buf[i] = result.getLowerHalf();
                buf[i+1] = result.getUpperHalf();
            }
        } else {
            static_assert(false_v<decltype(kind)>, "OperationKind must be read or write!");
        }
        digitalWrite<EnablePin, HIGH>();
        digitalWrite<EnablePin1, HIGH>();
        // we cannot
        SPI.endTransaction();
        return numBytesToFirstChip;
    }
public:
    static size_t write(uint32_t address, byte *buf, size_t capacity) noexcept {
        return genericReadWriteOperation<0x02, OperationKind::Write>(address, buf, capacity);
    }
    static size_t read(uint32_t address, byte *buf, size_t capacity) noexcept {
        return genericReadWriteOperation<0x03, OperationKind::Read>(address, buf, capacity);
    }
private:
public:
    static void begin() noexcept {
        static bool initialized_ = false;
        if (!initialized_) {
            initialized_ = true;
            SPI.beginTransaction(SPISettings(TargetBoard::runPSRAMAt(), MSBFIRST, SPI_MODE0));
            delayMicroseconds(200); // give the psram enough time to come up regardless of where you call begin

            digitalWrite<i960Pinout::PSRAM_EN, LOW>();
            digitalWrite<i960Pinout::PSRAM_EN1, LOW>();

            SPDR = 0x66;
            UDR1 = 0x66;
            asm volatile("nop");
            while (!(SPSR & _BV(SPIF))) ; // wait
            while (!(UCSR1A & (1 << RXC1)));
            digitalWrite<i960Pinout::PSRAM_EN1, HIGH>();
            digitalWrite<i960Pinout::PSRAM_EN, HIGH>();
            (void)UDR1;
            (void)UDR1;
            digitalWrite<i960Pinout::PSRAM_EN, LOW>();
            digitalWrite<i960Pinout::PSRAM_EN1, LOW>();
            SPDR = 0x99;
            UDR1 = 0x99;
            asm volatile("nop");
            while (!(SPSR & _BV(SPIF))) ; // wait
            while (!(UCSR1A & (1 << RXC1)));
            digitalWrite<i960Pinout::PSRAM_EN1, HIGH>();
            digitalWrite<i960Pinout::PSRAM_EN, HIGH>();
            (void)UDR1;
            (void)UDR1;
            SPI.endTransaction();
        }
    }
};
using OnboardPSRAMBlock = Type2MemoryBlock;
#endif
#endif //SXCHIPSET_DUALSPIBUSPSRAMCHIP_H
