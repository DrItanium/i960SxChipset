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
// Created by jwscoggins on 12/29/21.
//

#ifndef SXCHIPSET_23LC1024_H
#define SXCHIPSET_23LC1024_H
#include <SPI.h>
#include "MCUPlatform.h"
#include "Pinout.h"
template<i960Pinout enablePin>
class SRAM_23LC1024Chip {
public:
    static constexpr auto EnablePin = enablePin;
    static constexpr Address Capacity = 128_KB;
    static constexpr Address AddressMask = 0x1FFFF;
    static constexpr Address InvertedAddressMask = ~AddressMask;
    using TheEnablePin = DigitalPin<EnablePin>;
    using EnableCommunicationWithChip = PinAsserter<EnablePin>;
    static_assert(TheEnablePin ::isOutputPin(), "The sram chip must be tagged as output");
    SRAM_23LC1024Chip() = delete;
    ~SRAM_23LC1024Chip() = delete;
    SRAM_23LC1024Chip(SRAM_23LC1024Chip&&) = delete;
    SRAM_23LC1024Chip(const SRAM_23LC1024Chip&) = delete;
    SRAM_23LC1024Chip operator=(SRAM_23LC1024Chip&&) = delete;
    SRAM_23LC1024Chip operator=(const SRAM_23LC1024Chip&) = delete;
    static size_t write(uint32_t address, byte *buf, size_t capacity) noexcept {
        // it is up to the classes to handle overflow/wrap around
        auto maskedStartAddress= address & AddressMask;
        SPI.beginTransaction(SPISettings{10_MHz, MSBFIRST, SPI_MODE0});
        SplitWord32 decompose(maskedStartAddress);
        byte next = 0;
        size_t index = 0;
        {
            EnableCommunicationWithChip talk;
            SPDR = 0x02;
            asm volatile ("nop");
            while (!(SPSR & _BV(SPIF))); // wait
            SPDR = decompose.bytes[2];
            asm volatile ("nop");
            while (!(SPSR & _BV(SPIF))); // wait
            SPDR = decompose.bytes[1];
            asm volatile ("nop");
            while (!(SPSR & _BV(SPIF))); // wait
            SPDR = decompose.bytes[0];
            asm volatile ("nop");
            {
                next = buf[0];
                ++index;
            }
            while (!(SPSR & _BV(SPIF))); // wait
            // unpack the SPI.transfer for an array but do not overwrite the input array
            SPDR = next;
            for (; index < capacity; ++index) {
                next = buf[index];
                while (!(SPSR & _BV(SPIF)));
                SPDR = next;
            }
            while (!(SPSR & _BV(SPIF)));
        }
        SPI.endTransaction();
        return capacity;
    }
    static size_t read(uint32_t address, byte *buf, size_t capacity) noexcept {
        auto maskedStartAddress= address & AddressMask;
        SPI.beginTransaction(SPISettings{10_MHz, MSBFIRST, SPI_MODE0});
        SplitWord32 decompose(maskedStartAddress);
        {
            EnableCommunicationWithChip talk;
            SPDR = 0x03;
            asm volatile ("nop");
            while (!(SPSR & _BV(SPIF))); // wait
            SPDR = decompose.bytes[2];
            asm volatile ("nop");
            while (!(SPSR & _BV(SPIF))); // wait
            SPDR = decompose.bytes[1];
            asm volatile ("nop");
            while (!(SPSR & _BV(SPIF))); // wait
            SPDR = decompose.bytes[0];
            asm volatile ("nop");
            while (!(SPSR & _BV(SPIF))); // wait
            // just keep sending zero and get the result
            for (size_t index = 0; index < capacity; ++index) {
                SPDR = 0;
                asm volatile ("nop");
                while (!(SPSR & _BV(SPIF)));
                buf[index] = SPDR;
            }
        }
        SPI.endTransaction();
        return capacity;
    }
    static void begin() noexcept {
        if (!initialized_) {
            initialized_ = true;
            pinMode(TheEnablePin::getPin(), OUTPUT);
            TheEnablePin::deassertPin();
            // start at the beginning and just clear the entire chip out
            SPI.beginTransaction(SPISettings{10_MHz, MSBFIRST, SPI_MODE0});
            {
                EnableCommunicationWithChip talk;
                SPDR = 0x02;
                asm volatile ("nop");
                while (!(SPSR & _BV(SPIF))); // wait
                SPDR = 0;
                asm volatile ("nop");
                while (!(SPSR & _BV(SPIF))); // wait
                SPDR = 0;
                asm volatile ("nop");
                while (!(SPSR & _BV(SPIF))); // wait
                SPDR = 0;
                asm volatile ("nop");
                while (!(SPSR & _BV(SPIF))); // wait
                for (uint32_t i = 0; i < 128_KB; ++i) {
                    SPDR = 0;
                    asm volatile ("nop");
                    while (!(SPSR & _BV(SPIF))); // wait
                }
            }
            SPI.endTransaction();
        }
    }
private:
    static inline bool initialized_ = false;
};
#endif //SXCHIPSET_23LC1024_H
