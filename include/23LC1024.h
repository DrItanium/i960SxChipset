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
// Created by jwscoggins on 12/29/21.
//

#ifndef SXCHIPSET_23LC1024_H
#define SXCHIPSET_23LC1024_H
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
        TheEnablePin ::assertPin();
        SPI.transfer(0x02);
        SPI.transfer(maskedStartAddress >> 16);
        SPI.transfer(maskedStartAddress >> 8);
        SPI.transfer(maskedStartAddress);
        SPI.transfer(buf, capacity);
        // only transfer
        TheEnablePin ::deassertPin();
        SPI.endTransaction();
        return capacity;
    }
    static size_t read(uint32_t address, byte *buf, size_t capacity) noexcept {
        auto maskedStartAddress= address & AddressMask;
        SPI.beginTransaction(SPISettings{10_MHz, MSBFIRST, SPI_MODE0});
        TheEnablePin ::assertPin();
        SPI.transfer(0x03);
        SPI.transfer(maskedStartAddress >> 16);
        SPI.transfer(maskedStartAddress >> 8);
        SPI.transfer(maskedStartAddress);
        SPI.transfer(buf, capacity);
        // only transfer
        TheEnablePin ::deassertPin();
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
            TheEnablePin::assertPin();
            SPI.transfer(0x02);
            SPI.transfer(0);
            SPI.transfer(0);
            SPI.transfer(0);
            for (uint32_t i = 0; i < 128_KB; ++i) {
                SPI.transfer(0);
            }
            TheEnablePin::deassertPin();
            SPI.endTransaction();
        }
    }
private:
    static inline bool initialized_ = false;
};
#endif //SXCHIPSET_23LC1024_H
