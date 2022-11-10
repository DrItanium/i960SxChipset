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
// Created by jwscoggins on 7/19/22.
//

#ifndef SXCHIPSET_MEMORYPOOL_H
#define SXCHIPSET_MEMORYPOOL_H
#include "DualPSRAMPool.h"
/**
 * @brief Wrapper over pools 1 and 2. Pool 2 is mapped first followed by pool 1
 */
class MemoryPool {
public:
    using PSRAMPool = DualPoolPSRAM;
public:
    MemoryPool() = delete;
    ~MemoryPool() = delete;
    MemoryPool(MemoryPool&&) = delete;
    MemoryPool(const MemoryPool&) = delete;
    MemoryPool& operator=(const MemoryPool&) = delete;
    MemoryPool& operator=(MemoryPool&&) = delete;
    union PSRAMBlockAddress {
        constexpr explicit PSRAMBlockAddress(Address value = 0) : base(value) { }
        constexpr auto getIndex() const noexcept { return index; }
    private:
        Address base;
        struct {
            Address offset : 23;
            Address index : 9;
        };
    };
    static size_t write(uint32_t address, byte *buf, size_t capacity) noexcept {
        if (PSRAMBlockAddress addr(address); addr.getIndex() < 10) {
            return PSRAMPool::write(address, buf, capacity);
        } else {
            return 0;
        }
    }
    static size_t read(uint32_t address, byte *buf, size_t capacity) noexcept {
        if (PSRAMBlockAddress addr(address); addr.getIndex() < 10) {
            return PSRAMPool::read(address, buf, capacity);
        } else {
            for (size_t i = 0;i < capacity; ++i ) {
                buf[i] = 0;
            }
            return capacity;
        }
    }
    static void begin() noexcept {
        static bool initialized_ = false;
        if (!initialized_) {
            initialized_ = true;
            PSRAMPool::begin();
        }
    }
};
using CombinedMemoryPool = MemoryPool;
#endif //SXCHIPSET_MEMORYPOOL_H
