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
template<byte numTagBits, byte totalBitCount, byte numLowestBits, typename T, bool useSpecificTypeSizes>
class EightWayRandPLRUCacheSet {
public:
    static constexpr auto NumberOfWays = 8;
    static constexpr auto WayMask = NumberOfWays - 1;
    using CacheEntry = ::CacheEntry<numTagBits, totalBitCount, numLowestBits, T, useSpecificTypeSizes>;
    using TaggedAddress = typename CacheEntry::TaggedAddress;
    static constexpr auto NumBytesCached = CacheEntry::NumBytesCached;
public:
    /**
     * @brief Fill the cache up ahead of time to start off with given a base address, all entries are reset (invalid or not)
     * @param baseAddress the base address to use as the cache offset
     */
    void
    precache(TaggedAddress baseAddress) noexcept {
        for (byte i = 0; i < NumberOfWays; ++i)   {
            // don't even worry if it was populated or not, just make sure we populate it
            updateFlags(i);
            ways_[i].reset(TaggedAddress{i, baseAddress.getTagIndex()});
        }
    }
    CacheEntry& getLine(TaggedAddress theAddress) noexcept {
        // assume all lines are valid via ahead of time precaching
        for (byte i = 0; i < NumberOfWays; ++i) {
            if (ways_[i].addressesMatch(theAddress)) {
                updateFlags(i);
                return ways_[i];
            }
        }
        auto index = getLeastRecentlyUsed();
        updateFlags(index);
        ways_[index].reset(theAddress);
        return ways_[index];
    }
    void clear() noexcept {
        for (auto& way : ways_) {
            way.clear();
        }
        bits_ = 0;
    }
    [[nodiscard]] constexpr size_t size() const noexcept { return NumberOfWays; }
    static constexpr auto NumberOfGroups = 4;
    static void begin() {
        if (!randomTableInitialized_) {
           randomTableInitialized_ = true;
            counter = 0;
            for (auto& v : randomTable) {
                v = random(0, NumberOfGroups);
            }
        }
    }
    static byte getRandomValue() noexcept {
        return randomTable[counter++];
    }
private:
    static inline bool randomTableInitialized_ = false;
    // have a shared counter that is shared across all cache sets
    // this introduces an extra layer of randomness so that it is harder to determine which
    // cache lines are going to be swapped out in a given set at any one time.
    static inline byte counter = 0;
    /**
     * @brief Used to randomly select which group to perform replacement on. Coupled with the most recently used of each group of two
     * it makes up the Rand part of Rand TreePLRU. Generated at runtime the first time this method is invoked.
     */
    static inline byte randomTable[256] = { 0 };
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
    [[nodiscard]] byte getLeastRecentlyUsed() const noexcept {
        /**
         * @brief Two dimensional array that is broken up into groups of two with the opposite index stored at each position. So if you
         * do [0][1] then you get 0 back because you're saying the first group and the odd position was the most recently used line.
         */
        static constexpr byte secondLookupTable[4][2] {
                { 1, 0 },
                {3, 2},
                {5, 4},
                {7, 6},
        };
        /**
         * @brief Mask of the individual bit to look at inside of flag to find the least recently used
         */
        static constexpr byte maskLookup[4] {
            0b0001,
            0b0010,
            0b0100,
            0b1000,
        };
        // Instead of encoding the choice layers for top level followed by either the first two or the second two we just
        // use the random table to select one of the four groups. This reduces the number of bits required significantly at
        // the cost of initial spin up time. This spin up time can even be done prior to starting up as well.
        // in this case we only need 4 bits instead of 7 to do our thing. As long as the number of ways is a multiple of two we can
        // go up to 16 ways with a single byte of overhead.

        // the random table is between [0, NumberOfGroups) in this case that would be [0, 4)
        auto theIndex = getRandomValue();
        // use the index to select which of the four bits we care about to look at.
        // If true we return 1, otherwise zero. This is used in the lookup table to select the _other_ line to swap out
        // So if we get a 1 then it means swap out the even element of the pair, otherwise it is the odd element of the pair on a zero.

        // This is a standard TreePLRU design but the use of a random table improves performance by eliminating tons of branches in the
        // resultant code.
        return secondLookupTable[theIndex][(bits_ & maskLookup[theIndex]) ? 1 : 0];
    }

private:
    // This is RandPLRU Tree so we need to organize things correctly, I'm going to try four groups of two
    CacheEntry ways_[NumberOfWays];
    byte bits_ = 0;
};

#endif //SXCHIPSET_EIGHTWAYPSEUDOLRUENTRY_H
