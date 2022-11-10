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

#ifndef SXCHIPSET_TYPE1PSRAMCHIP_POOL2_H
#define SXCHIPSET_TYPE1PSRAMCHIP_POOL2_H
#ifdef CHIPSET_TYPE1
#include <Arduino.h>
#include <SPI.h>
#include "MCUPlatform.h"
#include "Pinout.h"
#include "TaggedCacheAddress.h"
/**
 * @brief Interface to the second memory pool found in the type 1.01 chipset. It is different than pool1 in that it is faster, smaller, and linked up with some flash memory as well.
 * The flash memory is _not_ controlled by this class, only the psram
 * @tparam enablePin The pin that is used to signal this pool of memory
 */
template<i960Pinout enablePin>
class MemoryBlock_Pool2 {
public:
    static constexpr auto EnablePin = enablePin;
    static constexpr auto Select0 = i960Pinout::MEMBLK0_A0;
    static constexpr auto Select1 = i960Pinout::MEMBLK0_A1;
    static constexpr auto NumChips = 2;
    static constexpr auto NumSections = NumChips / 2;
    using Self = MemoryBlock_Pool2<EnablePin>;

public:
    MemoryBlock_Pool2() = delete;
    ~MemoryBlock_Pool2() = delete;
    using PSRAMBlockAddress = SplitWord32;
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
    [[gnu::always_inline]] inline static size_t genericReadWriteOperation(uint32_t address, byte* buf, size_t capacity) noexcept {
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
        setChipId(curr.getPool2Index());
        SPI.beginTransaction(SPISettings(TargetBoard::runPSRAM2At(), MSBFIRST, SPI_MODE0));
        digitalWrite<EnablePin, LOW>();
        SPDR = opcode;
        {
            end = PSRAMBlockAddress(address + capacity);
            numBytesToSecondChip = end.getPool2Offset();
            tmp = curr.bytes[2];
        }
        while (!(SPSR & _BV(SPIF))) ; // wait
        SPDR = tmp;
        {
            localToASingleChip = curr.getPool2Index() == end.getPool2Index();
            tmp = curr.bytes[1];
        }
        while (!(SPSR & _BV(SPIF))) ; // wait
        SPDR = tmp;
        {
            numBytesToFirstChip = localToASingleChip ? capacity : (capacity - numBytesToSecondChip);
            tmp = curr.bytes[0];
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
            setChipId(end.getPool2Index());
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
    [[gnu::always_inline]] inline static size_t write(uint32_t address, byte *buf, size_t capacity) noexcept {
        return genericReadWriteOperation<0x02, OperationKind::Write>(address, buf, capacity);
    }
    [[gnu::always_inline]] inline static size_t read(uint32_t address, byte *buf, size_t capacity) noexcept {
        return genericReadWriteOperation<0x03, OperationKind::Read>(address, buf, capacity);
    }
private:
    [[gnu::always_inline]] inline static void setFirstChip() noexcept {
        // with MEMBLK0 0b01 is PSRAM0
        digitalWrite<Select0, HIGH>();
        digitalWrite<Select1, LOW>();
    }
    [[gnu::always_inline]] inline static void setSecondChip() noexcept {
        // with memblk0 0b11 is PSRAM1
        digitalWrite<Select0, HIGH>();
        digitalWrite<Select1, HIGH>();

    }
    [[gnu::always_inline]] inline static void setChipId(byte index) noexcept {
        // do not attempt to cache anything since we could eventually want to directly reference the flash text chip at some point
        if ((index & 1) == 0) {
            setFirstChip();
        } else {
            setSecondChip();
        }
    }
public:
    static void begin() noexcept {
        static bool initialized_ = false;
        if (!initialized_) {
            initialized_ = true;
            setChipId(0);
            SPI.beginTransaction(SPISettings(TargetBoard::runPSRAM2At(), MSBFIRST, SPI_MODE0));
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

using OnboardPSRAMBlock_Pool2 = MemoryBlock_Pool2<i960Pinout::MEMBLK0_>;
#endif
#endif //SXCHIPSET_TYPE1PSRAMCHIP_POOL2_H
