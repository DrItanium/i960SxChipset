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

#ifndef SXCHIPSET_SIXTEENWAYPSEUDOLRUENTRY_H
#define SXCHIPSET_SIXTEENWAYPSEUDOLRUENTRY_H
#include "MCUPlatform.h"
#include "Pinout.h"
#include "TaggedCacheAddress.h"
#include "CacheEntry.h"

template<byte numTagBits, byte totalBitCount, byte numLowestBits, typename T>
class SixteenWayLRUCacheWay {
public:
    static constexpr auto NumberOfWays = 16;
    static constexpr auto WayMask = NumberOfWays - 1;
    using CacheEntry = ::CacheEntry<numTagBits, totalBitCount, numLowestBits, T>;
    using TaggedAddress = typename CacheEntry::TaggedAddress;
public:
    __attribute__((noinline)) CacheEntry& getLine(TaggedAddress theAddress) noexcept {
        for (byte i = 0; i < NumberOfWays; ++i) {
            if (ways_[i]->matches(theAddress)) {
                updateFlags(i);
                return *ways_[i];
            }
        }
        // find the inverse of the most recently used
        auto index = getLeastRecentlyUsed();
        updateFlags(index);
        ways_[index]->reset(theAddress);
        return *ways_[index];
    }
    void clear() noexcept {
        for (auto& way : ways_) {
            way->clear();
        }
        mruBits_.wholeValue_ = 0;
    }
private:
    void updateFlags(byte index) noexcept {
        // cache the lookup
        static constexpr uint16_t lookup[NumberOfWays] {
                _BV(static_cast<uint16_t>(0)),
                _BV(static_cast<uint16_t>(1)),
                _BV(static_cast<uint16_t>(2)),
                _BV(static_cast<uint16_t>(3)),
                _BV(static_cast<uint16_t>(4)),
                _BV(static_cast<uint16_t>(5)),
                _BV(static_cast<uint16_t>(6)),
                _BV(static_cast<uint16_t>(7)),
                _BV(static_cast<uint16_t>(8)),
                _BV(static_cast<uint16_t>(9)),
                _BV(static_cast<uint16_t>(10)),
                _BV(static_cast<uint16_t>(11)),
                _BV(static_cast<uint16_t>(12)),
                _BV(static_cast<uint16_t>(13)),
                _BV(static_cast<uint16_t>(14)),
                _BV(static_cast<uint16_t>(15)),
        };
        mruBits_.wholeValue_ |= lookup[index];
        if (mruBits_.wholeValue_ == 0xFFFF) {
            mruBits_.wholeValue_ = lookup[index];
        }
    }
    [[nodiscard]] constexpr byte getLeastRecentlyUsed() const noexcept {
        if (mruBits_.bytes[1] != 0xFF) {
            return LRUTable2[mruBits_.bytes[1]]; // add eight to the position
        } else {
            return LRUTable[mruBits_.bytes[0]];
        }
    }
    [[nodiscard]] constexpr bool valid() const noexcept {
        for (auto *a : ways_) {
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
    SplitWord16 mruBits_ {0};
    static constexpr byte LRUTable[256] {
            7, 7, 7, 7, 7, 7, 7, 7,
            7, 7, 7, 7, 7, 7, 7, 7,
            7, 7, 7, 7, 7, 7, 7, 7,
            7, 7, 7, 7, 7, 7, 7, 7,
            7, 7, 7, 7, 7, 7, 7, 7,
            7, 7, 7, 7, 7, 7, 7, 7,
            7, 7, 7, 7, 7, 7, 7, 7,
            7, 7, 7, 7, 7, 7, 7, 7,
            7, 7, 7, 7, 7, 7, 7, 7,
            7, 7, 7, 7, 7, 7, 7, 7,
            7, 7, 7, 7, 7, 7, 7, 7,
            7, 7, 7, 7, 7, 7, 7, 7,
            7, 7, 7, 7, 7, 7, 7, 7,
            7, 7, 7, 7, 7, 7, 7, 7,
            7, 7, 7, 7, 7, 7, 7, 7,
            7, 7, 7, 7, 7, 7, 7, 7,

            6, 6, 6, 6, 6, 6, 6, 6,
            6, 6, 6, 6, 6, 6, 6, 6,
            6, 6, 6, 6, 6, 6, 6, 6,
            6, 6, 6, 6, 6, 6, 6, 6,
            6, 6, 6, 6, 6, 6, 6, 6,
            6, 6, 6, 6, 6, 6, 6, 6,
            6, 6, 6, 6, 6, 6, 6, 6,
            6, 6, 6, 6, 6, 6, 6, 6,

            5, 5, 5, 5, 5, 5, 5, 5,
            5, 5, 5, 5, 5, 5, 5, 5,
            5, 5, 5, 5, 5, 5, 5, 5,
            5, 5, 5, 5, 5, 5, 5, 5,

            4, 4, 4, 4, 4, 4, 4, 4,
            4, 4, 4, 4, 4, 4, 4, 4,

            3, 3, 3, 3, 3, 3, 3, 3,
            2, 2, 2, 2,
            1, 1,
            0, 0,
    };
    static constexpr byte LRUTable2[256] {
            15, 15, 15, 15, 15, 15, 15, 15,
            15, 15, 15, 15, 15, 15, 15, 15,
            15, 15, 15, 15, 15, 15, 15, 15,
            15, 15, 15, 15, 15, 15, 15, 15,
            15, 15, 15, 15, 15, 15, 15, 15,
            15, 15, 15, 15, 15, 15, 15, 15,
            15, 15, 15, 15, 15, 15, 15, 15,
            15, 15, 15, 15, 15, 15, 15, 15,
            15, 15, 15, 15, 15, 15, 15, 15,
            15, 15, 15, 15, 15, 15, 15, 15,
            15, 15, 15, 15, 15, 15, 15, 15,
            15, 15, 15, 15, 15, 15, 15, 15,
            15, 15, 15, 15, 15, 15, 15, 15,
            15, 15, 15, 15, 15, 15, 15, 15,
            15, 15, 15, 15, 15, 15, 15, 15,
            15, 15, 15, 15, 15, 15, 15, 15,

            14, 14, 14, 14, 14, 14, 14, 14,
            14, 14, 14, 14, 14, 14, 14, 14,
            14, 14, 14, 14, 14, 14, 14, 14,
            14, 14, 14, 14, 14, 14, 14, 14,
            14, 14, 14, 14, 14, 14, 14, 14,
            14, 14, 14, 14, 14, 14, 14, 14,
            14, 14, 14, 14, 14, 14, 14, 14,
            14, 14, 14, 14, 14, 14, 14, 14,

            13, 13, 13, 13, 13, 13, 13, 13,
            13, 13, 13, 13, 13, 13, 13, 13,
            13, 13, 13, 13, 13, 13, 13, 13,
            13, 13, 13, 13, 13, 13, 13, 13,

            12, 12, 12, 12, 12, 12, 12, 12,
            12, 12, 12, 12, 12, 12, 12, 12,

            11, 11, 11, 11, 11, 11, 11, 11,
            10, 10, 10, 10,
            9, 9,
            8, 0,
    };
};
#endif //SXCHIPSET_SIXTEENWAYPSEUDOLRUENTRY_H
