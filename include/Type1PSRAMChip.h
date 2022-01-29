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
    [[gnu::always_inline]] static inline void transmitByte(byte value) noexcept {
        SPDR = value;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait
    }
    [[gnu::always_inline]] static inline byte receiveByte(byte transmitValue = 0) noexcept {
        transmitByte(transmitValue);
        return SPDR;

    }
    enum class OperationKind {
        Write,
        Read,
    };

    template<byte opcode, OperationKind kind>
    inline static size_t genericReadWriteOperation(uint32_t address, byte* buf, size_t capacity) noexcept {
        static_assert(kind == OperationKind::Read || kind == OperationKind::Write, "Must be a valid OperationKind type");
        if (capacity == 0) {
            return 0;
        }
        PSRAMBlockAddress end;
        uint32_t numBytesToSecondChip;
        uint32_t numBytesToFirstChip;
        bool localToASingleChip, spansMultipleChips;
        byte tmp;
        PSRAMBlockAddress curr(address);
        setChipId(curr.getIndex());
        SPI.beginTransaction(SPISettings(TargetBoard::runPSRAMAt(), MSBFIRST, SPI_MODE0));
        digitalWrite<EnablePin, LOW>();
        SPDR = opcode;
        {
            end = PSRAMBlockAddress(address + capacity);
            numBytesToSecondChip = end.getOffset();
            tmp = curr.bytes_[2];
        }
        while (!(SPSR & _BV(SPIF))) ; // wait
        SPDR = tmp;
        {
            localToASingleChip = curr.getIndex() == end.getIndex();
            tmp = curr.bytes_[1];
        }
        while (!(SPSR & _BV(SPIF))) ; // wait
        SPDR = tmp;
        {
            numBytesToFirstChip = localToASingleChip ? capacity : (capacity - numBytesToSecondChip);
            tmp = curr.bytes_[0];
        }
        while (!(SPSR & _BV(SPIF))) ; // wait
        SPDR = tmp;
        {
            spansMultipleChips =  (!localToASingleChip && (numBytesToSecondChip > 0));
            if constexpr (kind == OperationKind::Write) {
                tmp = buf[0];
            }
        }
        while (!(SPSR & _BV(SPIF))) ; // wait
        for (decltype(numBytesToFirstChip) i = 0; i < numBytesToFirstChip; ++i) {
           if constexpr (kind == OperationKind::Read)  {
               buf[i] = receiveByte();
           } else {
               SPDR = tmp;
               tmp = buf[i + 1];
               while (!(SPSR & _BV(SPIF))) ; // wait
           }
        }
        digitalWrite<EnablePin, HIGH>();
        if (spansMultipleChips) {
            // since size_t is 16-bits on AVR we can safely reduce the largest buffer size 64k, thus we can only ever span two psram chips at a time
            // thus we can actually convert this work into two separate spi transactions
            // start writing at the start of the next chip the remaining number of bytes
            setChipId(end.getIndex());
            // we start at address zero on this new chip always
            digitalWrite<EnablePin, LOW>();
            SPDR = opcode;
            auto actualBuf = buf + numBytesToFirstChip;
            while (!(SPSR & _BV(SPIF))) ; // wait
            transmitByte(0);
            transmitByte(0);
            SPDR = 0;
            {
                if constexpr (kind == OperationKind::Write) {
                    tmp = actualBuf[0];
                } else {
                    asm volatile ("nop");
                }
            }
            while (!(SPSR & _BV(SPIF))) ; // wait
            for (decltype(numBytesToSecondChip) i = 0; i < numBytesToSecondChip; ++i) {
               if constexpr (kind == OperationKind::Read) {
                   actualBuf[i] = receiveByte();
               } else {
                   transmitByte(actualBuf[i]);
                   SPDR = tmp;
                   tmp = actualBuf[i + 1];
                   while (!(SPSR & _BV(SPIF))) ; // wait
               }
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
    static constexpr byte computePortLookup(byte value) noexcept {
        return (value & 0b111) << 2;
    }
    static void setChipId(byte index) noexcept {
        static constexpr byte theItemMask = 0b00011100;
        static constexpr byte theInvertedMask = ~theItemMask;
        static constexpr byte LookupTable[8] {
                computePortLookup(0),
                computePortLookup(1),
                computePortLookup(2),
                computePortLookup(3),
                computePortLookup(4),
                computePortLookup(5),
                computePortLookup(6),
                computePortLookup(7),
        };
        // since this is specific to type 1 we should speed this up significantly
        auto contents = PORTC;
        contents &= theInvertedMask;
        contents |= LookupTable[index & 0b111];
        PORTC = contents;
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
};

using OnboardPSRAMBlock = MemoryBlock<i960Pinout::PSRAM_EN>;
#endif
#endif //SXCHIPSET_TYPE1PSRAMCHIP_H
