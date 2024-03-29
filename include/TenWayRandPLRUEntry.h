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

#ifndef SXCHIPSET_TENWAYRANDPLRUENTRY_H
#define SXCHIPSET_TENWAYRANDPLRUENTRY_H
#include "MCUPlatform.h"
#include "Pinout.h"
#include "TaggedCacheAddress.h"
#include "CacheEntry.h"

/**
 * @brief Defines a cache set that is made up of 10 lines using a random tree plru replacement policy
 * @tparam numTagBits The number of bits that make up a tag overall
 * @tparam totalBitCount  The total number of bits that make up an address partially serviced by this cache set
 * @tparam numLowestBits The number of bytes that each cache line will store
 * @tparam T The backing storage type or where we read from and write to on a cache miss
 */
template<byte numTagBits, byte totalBitCount, byte numLowestBits, typename T, bool useSpecificTypeSizes>
class TenWayRandPLRUCacheSet {
public:
    static constexpr auto NumberOfWays = 10;
    //static constexpr auto WayMask = NumberOfWays - 1;
    using CacheEntry = ::CacheEntry<numTagBits, totalBitCount, numLowestBits, T, useSpecificTypeSizes>;
    using TaggedAddress = typename CacheEntry::TaggedAddress;
    static constexpr auto NumBytesCached = CacheEntry::NumBytesCached;
public:
    CacheEntry& getLine(TaggedAddress theAddress) noexcept {
        byte targetIndex = 0xFF;
        for (byte i = 0; i < NumberOfWays; ++i) {
            if (ways_[i].matches(theAddress)) {
                updateFlags(i);
                return ways_[i];
            } else if ((targetIndex >= NumberOfWays) && !ways_[i].isValid()) {
                targetIndex = i;
            }
        }

        auto index = (targetIndex < NumberOfWays) ? targetIndex : getLeastRecentlyUsed();
        updateFlags(index);
        ways_[index].reset(theAddress);
        return ways_[index];
    }
    void clear() noexcept {
        if (!randomTableInitialized_) {
            // initialize the random table
            randomTableInitialized_ = true;
            counter_ = 0;
            for (auto& v : randomTable_) {
                v = random(0, NumberOfGroups);
            }
        }
        for (auto& way : ways_) {
            way.clear();
        }
        bits_ = 0;
    }
    [[nodiscard]] constexpr size_t size() const noexcept { return NumberOfWays; }
    void
    precache(TaggedAddress baseAddress) noexcept {
        for (byte i = 0; i < NumberOfWays; ++i)   {
            // don't even worry if it was populated or not, just make sure we populate it
            updateFlags(i);
            ways_[i].reset(TaggedAddress{i, baseAddress.getTagIndex()});
        }
    }
    static void
    begin() noexcept {
        if (!randomTableInitialized_) {
            randomTableInitialized_ = true;
            counter_ = 0;
            for (auto& v : randomTable_) {
                v = random(0, NumberOfGroups);
            }
        }
    }
private:
    static constexpr auto NumberOfGroups = 5;
    void
    updateFlags(byte index) noexcept {
        /// @note I know that this routine is unsafe but the added overhead of checking for validity adds some considerable expense on each lookup
        // It also seems that the compiler knows this is potentially unsafe and thus seems to insert extra code as well...
        // it is very strange...
        static constexpr byte masks[NumberOfGroups*2] {
                0b11110, 0b00001,
                0b11101, 0b00010,
                0b11011, 0b00100,
                0b10111, 0b01000,
                0b01111, 0b10000,
        };
        if (byte rIndex = index & 0b1111, mask = masks[rIndex]; (rIndex & 0b1) == 0) {
            bits_ &= mask;
        } else {
            bits_ |= mask;
        }
    }
    [[nodiscard]] byte getLeastRecentlyUsed() const noexcept {
        /**
         * @brief Mask of the individual bit to look at inside of flag to find the least recently used
         */
        static constexpr byte maskLookup[NumberOfGroups] {
            0b00001,
            0b00010,
            0b00100,
            0b01000,
            0b10000,
        };
        // Instead of encoding the choice layers for top level followed by either the first two or the second two we just
        // use the random table to select one of the four groups. This reduces the number of bits required significantly at
        // the cost of initial spin up time. This spin up time can even be done prior to starting up as well.
        // in this case we only need 4 bits instead of 7 to do our thing. As long as the number of ways is a multiple of two we can
        // go up to 16 ways with a single byte of overhead.

        // the random table is between [0, NumberOfGroups) in this case that would be [0, 4)
        auto theIndex = randomTable_[counter_++];
        // use the index to select which of the four bits we care about to look at.
        // If true we return 1, otherwise zero. This is used in the lookup table to select the _other_ line to swap out
        // So if we get a 1 then it means swap out the even element of the pair, otherwise it is the odd element of the pair on a zero.

        // This is a standard TreePLRU design but the use of a random table improves performance by eliminating tons of branches in the
        // resultant code.
        return theIndex + ((bits_ & maskLookup[theIndex]) ? 0 : 1);
    }

private:
    // This is RandPLRU Tree so we need to organize things correctly, I'm going to try four groups of two
    CacheEntry ways_[NumberOfWays];
    byte bits_ = 0;
    static inline bool randomTableInitialized_ = false;
    static inline byte counter_ = 0;
    static inline byte randomTable_[256] = { 0};
};

#endif //SXCHIPSET_TENWAYPSEUDOLRUENTRY_H
