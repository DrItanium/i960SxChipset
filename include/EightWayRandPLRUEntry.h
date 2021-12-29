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

#ifndef SXCHIPSET_EIGHTWAYRANDPLRUENTRY_H
#define SXCHIPSET_EIGHTWAYRANDPLRUENTRY_H
#include "MCUPlatform.h"
#include "Pinout.h"
#include "TaggedCacheAddress.h"
#include "CacheEntry.h"

/**
 * @brief Divides a series of 8 cache lines in the set into several subsets where the replacement algorithm uses a combination of random replacement and tree plru
 * @tparam numTagBits The number of bits that make up a tag overall
 * @tparam totalBitCount  The total number of bits that make up an address partially serviced by this cache set
 * @tparam numLowestBits The number of bytes that each cache line will store
 * @tparam T The backing storage type or where we read from and write to on a cache miss
 */
template<byte numTagBits, byte totalBitCount, byte numLowestBits, typename T, bool useSpecificTypeSizes = true>
class EightWayRandPLRUCacheSet {
public:
    static constexpr auto NumberOfWays = 8;
    static constexpr auto WayMask = NumberOfWays - 1;
    using CacheEntry = ::CacheEntry<numTagBits, totalBitCount, numLowestBits, T, useSpecificTypeSizes>;
    using TaggedAddress = typename CacheEntry::TaggedAddress;
    static constexpr auto NumBytesCached = CacheEntry::NumBytesCached;
public:
    //[[gnu::noinline]]
    CacheEntry& getLine(TaggedAddress theAddress) noexcept {
        byte targetIndex = 0xFF;
        for (byte i = 0; i < NumberOfWays; ++i) {
            if (ways_[i]->matches(theAddress)) {
                updateFlags(i);
                return *ways_[i];
            } else if ((targetIndex >= NumberOfWays) && !ways_[i]->isValid()) {
                targetIndex = i;
            }
        }

        auto index = (targetIndex < NumberOfWays) ? targetIndex : getLeastRecentlyUsed();
        updateFlags(index);
        ways_[index]->reset(theAddress);
        return *ways_[index];
    }
    void clear() noexcept {
        for (auto& way : ways_) {
            way->clear();
        }
        bits_ = 0;
    }
    [[nodiscard]] constexpr auto getWay(size_t index = 0) const noexcept { return ways_[index & WayMask]; }
    void setWay(CacheEntry& way, size_t index = 0) noexcept { ways_[index & WayMask] = &way; }
    [[nodiscard]] constexpr size_t size() const noexcept { return NumberOfWays; }
private:
    void updateFlags(byte index) noexcept {
        constexpr byte masks[8] {
                0b1110, 0b0001, 0b1101, 0b0010,
                0b1011, 0b0100, 0b0111, 0b1000,
        };
        if (auto rIndex = index & 0b111; (rIndex & 0b1) == 0) {
            bits_ &= masks[rIndex];
        } else {
            bits_ |= masks[rIndex];
        }
    }
    static constexpr auto NumberOfGroups = 4;
    [[nodiscard]] byte getLeastRecentlyUsed() const noexcept {
        static bool initialized = false;
        static byte counter = 0;
        static byte randomTable[256] = { 0 };
        static constexpr byte secondLookupTable[4][2] {
                { 1, 0 },
                {3, 2},
                {5, 4},
                {7, 6},
        };
        static constexpr byte maskLookup[4] {
            0b0001,
            0b0010,
            0b0100,
            0b1000,
        };
        if (!initialized) {
            initialized = true;
            counter = 0;
            for (uint16_t i = 0; i < 256; ++i) {
                randomTable[i] = random(0, NumberOfGroups);
            }
        }
        auto theIndex = randomTable[counter++];
        return secondLookupTable[theIndex][(bits_ & maskLookup[theIndex]) ? 1 : 0];
    }

private:
    // This is RandPLRU Tree so we need to organize things correctly, I'm going to try four groups of two
    CacheEntry* ways_[NumberOfWays] = { nullptr };
    byte bits_ = 0;
};

#endif //SXCHIPSET_EIGHTWAYPSEUDOLRUENTRY_H
