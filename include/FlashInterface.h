/*
i960SxChipset
Copyright (c) 2020-2022, Joshua Scoggins
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
// Created by jwscoggins on 3/5/22.
//

#ifndef SXCHIPSET_FLASHINTERFACE_H
#define SXCHIPSET_FLASHINTERFACE_H
#ifdef CHIPSET_TYPE1
#include "Pinout.h"
/**
 * @brief Interface to the two flash chips in memblk0 on the type 1.01 flash and psram card
 */
class FlashInterface final {
public:
    static constexpr auto EnablePin = i960Pinout::MEMBLK0_;
    static constexpr auto Select0 = i960Pinout::MEMBLK0_A0;
    static constexpr auto Select1 = i960Pinout::MEMBLK0_A1;
    FlashInterface() = delete;
    ~FlashInterface() = delete;
private:
    static void setFirstChip() noexcept {
        // with MEMBLK0 0b00 is Flash0
        digitalWrite<Select0, LOW>();
        digitalWrite<Select1, LOW>();
    }
    static void setSecondChip() noexcept {
        // with memblk0 0b10 is Flash1
        digitalWrite<Select0, LOW>();
        digitalWrite<Select1, HIGH>();

    }
    static void setChipId(byte index) noexcept {
        // do not attempt to cache anything since we could eventually want to directly reference the flash text chip at some point
        if ((index & 1) == 0) {
            setFirstChip();
        } else {
            setSecondChip();
        }
    }
    union BlockAddress {
        constexpr explicit BlockAddress(Address value = 0) : base(value) { }
        [[nodiscard]] constexpr auto getAddress() const noexcept { return base; }
        [[nodiscard]] constexpr auto getOffset() const noexcept { return offset; }
        [[nodiscard]] constexpr auto getIndex() const noexcept { return index; }
        Address base;
        struct {
            Address offset : 22;
            byte index : 1;
        };
        byte bytes_[4];
    };
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
        static_assert(kind == OperationKind::Read, "Must be a valid OperationKind type");
        if (capacity == 0) {
            return 0;
        }
        BlockAddress end;
        uint32_t numBytesToSecondChip;
        uint32_t numBytesToFirstChip;
        bool localToASingleChip, spansMultipleChips;
        byte tmp;
        BlockAddress curr(address);
        setChipId(curr.getIndex());
        SPI.beginTransaction(SPISettings(TargetBoard::runFlashAt(), MSBFIRST, SPI_MODE0));
        digitalWrite<EnablePin, LOW>();
        SPDR = opcode;
        {
            end = BlockAddress(address + capacity);
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
    static void begin() noexcept {
    }
    [[nodiscard]] static constexpr uint32_t length() noexcept { return 8_MB; }
    static size_t write(uint32_t address, byte *buf, size_t capacity) noexcept {
        // disallow writing
        //return genericReadWriteOperation<0x02, OperationKind::Write>(address, buf, capacity);
        return 0;
    }
    static size_t read(uint32_t address, byte *buf, size_t capacity) noexcept {
        return genericReadWriteOperation<0x03, OperationKind::Read>(address, buf, capacity);
    }
private:
};
#endif

#endif //SXCHIPSET_FLASHINTERFACE_H
