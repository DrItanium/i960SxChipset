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
            Address offset: 23;
            byte index : 3; // we have blocks of 16 megabytes instead of blocks of 8 megabytes found in type 1
        };
        byte bytes_[4];
    };
private:
    static inline byte transmitByte(byte value) noexcept {
        SPDR = value;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait
        return SPDR;
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
        // unlike a single channel design, the dual channel design only stores half the bytes per chip.
        // Thus we need to make sure the address actually reflects this. For instance, if we are transferring 16 bytes then
        // 8 bytes to go one side and 8 byte to the other. The difference is that the address in question must map to a real 23 bit address
        if (capacity == 0) {
            return 0;
        }
        // since we are commiting half the bytes to each chip we have to treat the address as though we are only commiting half the capacity
        // so shift right by 1 to reflect this change
        auto realAddress = address >> 1;
        // the realCapacity is the number of bytes stored to each chip in the transaction
        auto realCapacity = capacity >> 1;
        PSRAMBlockAddress curr(realAddress);
        SPI.beginTransaction(SPISettings(TargetBoard::runPSRAMAt(), MSBFIRST, SPI_MODE0));
        // computing the end address is a little strange with a dual channel setup. It is realAddress + realCapacity because only
        // half the bytes go to a given chip. This is why we can get 16 megabytes of storage out of a 23 bit address
        PSRAMBlockAddress end((realAddress) + realCapacity);
        auto numBytesToSecondChip = end.getOffset();
        auto localToASingleChip = curr.getIndex() == end.getIndex();
        // the only part where we have to be careful is when returning the number of bytes committed.
        // It will always be double the number computed here, so just double the number right now
        auto numBytesToFirstChip = (localToASingleChip ? realCapacity : (realCapacity - numBytesToSecondChip)) * 2;

        digitalWrite<EnablePin, LOW>();
        digitalWrite<EnablePin1, LOW>();
        (void)dualTransferMirrored(opcode);
        (void)dualTransferMirrored(curr.bytes_[2]);
        (void)dualTransferMirrored(curr.bytes_[1]);
        (void)dualTransferMirrored(curr.bytes_[0]);
        // at this point just iterate through and transfer all of the bytes, hopefully the total number of bytes is even
        for (decltype(numBytesToFirstChip) i = 0; i < numBytesToFirstChip; i+=2) {
            // interleave bytes, that way it is easier to access this stuff
            if constexpr (kind == OperationKind::Write) {
                (void)dualTransfer(buf[i], buf[i+1]);
            } else if constexpr (kind == OperationKind::Read) {
                auto result = dualTransferMirrored(0);
                buf[i] = result.getLowerHalf();
                buf[i+1] = result.getUpperHalf();
            } else {
                static_assert(false_v<decltype(kind)>, "OperationKind must be read or write!");
            }
        }
        digitalWrite<EnablePin1, HIGH>();
        digitalWrite<EnablePin, HIGH>();
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
            digitalWrite<EnablePin, LOW>();
            digitalWrite<EnablePin1, LOW>();
            (void)dualTransferMirrored(0x66);
            digitalWrite<EnablePin1, HIGH>();
            digitalWrite<EnablePin, HIGH>();
            __builtin_avr_nops(4);
            digitalWrite<EnablePin, LOW>();
            digitalWrite<EnablePin1, LOW>();
            (void) dualTransferMirrored(0x99);
            digitalWrite<EnablePin1, HIGH>();
            digitalWrite<EnablePin, HIGH>();
            SPI.endTransaction();
        }
    }
};
using OnboardPSRAMBlock = Type2MemoryBlock;
#endif
#endif //SXCHIPSET_DUALSPIBUSPSRAMCHIP_H
