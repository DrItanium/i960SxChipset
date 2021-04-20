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

#ifndef ARDUINO_ONBOARDSRAMCACHE_H
#define ARDUINO_ONBOARDSRAMCACHE_H
#include <Arduino.h>
#include "RAM.h"
template<int numBytes = 8192>
class  OnboardSRAM : public RAM {
public:
    union WordEntry {
        byte bytes[2];
        uint16_t word;
    };
    static_assert (numBytes <= 8192, "Cannot have more than 8K of SRAM");
    static_assert (numBytes > 1, "Must have at least 2 bytes associated with this onboard ram cache");
    static_assert (sizeof(WordEntry) == 2, "WordEntry must be two bytes in size");
    static constexpr uint32_t Size = numBytes;
    static constexpr auto NumEntries = Size / sizeof (WordEntry);
    static constexpr auto ByteMask = Size - 1;
    static constexpr auto EntryMask = ByteMask >> 1;
public:
    constexpr OnboardSRAM(uint32_t startAddress) : RAM(startAddress, Size) {}
    ~OnboardSRAM() override = default;
    void begin() override { }
private:
    constexpr auto computeMaskedAddress(uint32_t addr) noexcept { return (ByteMask & addr) >> 1; }
protected:
    uint8_t read8(uint32_t address) override {
        auto address = computeMaskedAddress(address);

    }
    uint16_t read16(uint32_t address) override {
    }
    void write8(uint32_t address, uint8_t value) override {
    }
    void write16(uint32_t address, uint16_t value) override {
    }
private:
    WordEntry cache_[NumEntries] = { 0 };
}
#endif //ARDUINO_ONBOARDSRAMCACHE_H
