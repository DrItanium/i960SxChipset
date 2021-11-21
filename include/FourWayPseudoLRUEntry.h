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
// Created by jwscoggins on 11/19/21.
//

#ifndef SXCHIPSET_FOURWAYPSEUDOLRUENTRY_H
#define SXCHIPSET_FOURWAYPSEUDOLRUENTRY_H
#include "MCUPlatform.h"
#include "Pinout.h"
#include "TaggedCacheAddress.h"
#include "CacheEntry.h"

template<byte numTagBits, byte totalBitCount, byte numLowestBits, typename T>
class FourWayLRUCacheWay {
public:
    static constexpr auto NumberOfWays = 4;
    static constexpr auto WayMask = NumberOfWays - 1;
    using CacheEntry = ::CacheEntry<numTagBits, totalBitCount, numLowestBits, T>;
    using TaggedAddress = typename CacheEntry::TaggedAddress;
    static constexpr auto NumBytesCached = CacheEntry::NumBytesCached;
public:
    __attribute__((noinline)) CacheEntry& getLine(TaggedAddress theAddress) noexcept {
        if (auto result = find (theAddress); result) {
            return *result;
        } else {
            return reset(theAddress);
        }

    }
    CacheEntry* find(TaggedAddress theAddress) noexcept {
        // find the inverse of the most recently used
        for (byte i = 0; i < NumberOfWays; ++i) {
            if (ways_[i]->matches(theAddress)) {
                updateFlags(i);
                return ways_[i];
            }
        }
        return nullptr;
    }
    CacheEntry&
    reset(TaggedAddress theAddress) noexcept {
        auto index = getLeastRecentlyUsed();
        updateFlags(index);
        ways_[index]->reset(theAddress);
        return *ways_[index];
    }
    void clear() noexcept {
        for (auto& way : ways_) {
            way->clear();
        }
        flags_ = 0;
    }
private:
    void updateFlags(byte index) noexcept {
        // we have to take the index and current value into account
        // bit 0: top
        // bit 1: left
        // bit 2: right
        static constexpr byte lookup[4] {
            _BV(0),
                    _BV(1),
                    _BV(2),
                    _BV(3),
        };
        flags_ |= lookup[index];
        if (flags_ >= 0xF) {
            flags_ = lookup[index];
        }
    }
    [[nodiscard]] constexpr byte getLeastRecentlyUsed() const noexcept {
        return LRUTable[flags_];
    }
    [[nodiscard]] constexpr bool valid() const noexcept {
        for (auto *a: ways_) {
            if (!a) {
                return false;
            }
        }
        return true;
    }
    [[nodiscard]] constexpr auto getWay(size_t index = 0) const noexcept { return ways_[index & WayMask]; }
    void setWay(CacheEntry& way, size_t index = 0) noexcept { ways_[index & WayMask] = &way; }
    [[nodiscard]] constexpr size_t size() const noexcept { return NumberOfWays; }
private:
    CacheEntry* ways_[NumberOfWays] = { nullptr };
    byte flags_ = 0;
    static constexpr byte LRUTable[16] {
        3, 3, 3, 3, 3, 3, 3, 3,
                2, 2, 2, 2,
                1, 1,
                0, 0,
    };

};
#endif //SXCHIPSET_FOURWAYPSEUDOLRUENTRY_H
