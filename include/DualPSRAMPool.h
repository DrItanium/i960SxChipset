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

#ifndef SXCHIPSET_DUALPSRAMPOOL_H
#define SXCHIPSET_DUALPSRAMPOOL_H
#ifdef CHIPSET_TYPE1
#include "PSRAMChip.h"

/**
 * @brief Wrapper over pools 1 and 2. Pool 2 is mapped first followed by pool 1
 */
class DualPoolPSRAM {
public:
    using Pool1 = OnboardPSRAMBlock;
    using Pool2 = OnboardPSRAMBlock_Pool2;
    static constexpr auto NumChips = Pool1::NumChips + Pool2::NumChips;
    static constexpr auto NumSections = Pool1::NumSections + Pool2::NumSections;
public:
    DualPoolPSRAM() = delete;
    ~DualPoolPSRAM() = delete;
    union PSRAMBlockAddress {
        constexpr explicit PSRAMBlockAddress(Address value = 0) : base(value) { }
        constexpr auto getAddress() const noexcept { return base; }
        constexpr auto getOffset() const noexcept { return offset; }
        constexpr auto getIndex() const noexcept { return index; }
        Address base;
        struct {
            Address offset : 23;
            byte index : 4;
        };
        byte bytes_[4];
    };
    static size_t write(uint32_t address, byte *buf, size_t capacity) noexcept {
        switch (PSRAMBlockAddress addr(address); addr.getIndex()) {
            case 0:
            case 1:
                return Pool2::write(address, buf, capacity);
            case 2:
            case 3:
            case 4:
            case 5:
            case 6:
            case 7:
            case 8:
            case 9:
                return Pool1::write(address, buf, capacity);
            default:
                return 0;
        }
    }
    static size_t read(uint32_t address, byte *buf, size_t capacity) noexcept {
        switch (PSRAMBlockAddress addr(address); addr.getIndex()) {
            case 0:
            case 1:
                return Pool2::read(address, buf, capacity);
            case 2:
            case 3:
            case 4:
            case 5:
            case 6:
            case 7:
            case 8:
            case 9:
                return Pool1::read(address, buf, capacity);
            default:
                return 0;
        }
    }
    static void begin() noexcept {
        static bool initialized_ = false;
        if (!initialized_) {
            initialized_ = true;
            Pool1::begin();
            Pool2::begin();
        }
    }
};
using DualPoolMemoryBlock = DualPoolPSRAM;
#endif // end CHIPSET_TYPE1
#endif //SXCHIPSET_DUALPSRAMPOOL_H
