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
// Created by jwscoggins on 11/7/22.
//

#ifndef SXCHIPSET_ROUNDROBINCACHESET_H
#define SXCHIPSET_ROUNDROBINCACHESET_H
#include <Arduino.h>

template<byte numTagBits, byte totalBitCount, byte numLowestBits, typename T, bool useSpecificTypeSizes>
class FourWayRoundRobinCacheWay {
public:
    static constexpr auto NumberOfWays = 4;
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
            ways_[i].reset(TaggedAddress{i, baseAddress.getTagIndex()});
        }
    }
    CacheEntry& getLine(TaggedAddress theAddress) noexcept {
        // find the inverse of the most recently used
        for (byte i = 0; i < NumberOfWays; ++i) {
            if (ways_[i].addressesMatch(theAddress)) {
                return ways_[i];
            }
        }
        auto index = count_;
        ways_[index].reset(theAddress);
        ++count_;
        return ways_[index];
    }
    void clear() noexcept {
        for (auto& way : ways_) {
            way.clear();
        }
        count_ = 0;
    }
    static void begin() noexcept {
    }
public:
    [[nodiscard]] constexpr size_t size() const noexcept { return NumberOfWays; }
private:
    CacheEntry ways_[NumberOfWays];
    byte count_ : 2;
};
template<byte numTagBits, byte totalBitCount, byte numLowestBits, typename T, bool useSpecificTypeSizes, byte numberOfWays>
class RoundRobinCacheWay {
public:
    static constexpr auto NumberOfWays = numberOfWays;
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
            ways_[i].reset(TaggedAddress{i, baseAddress.getTagIndex()});
        }
    }
    CacheEntry& getLine(TaggedAddress theAddress) noexcept {
        // find the inverse of the most recently used
        for (byte i = 0; i < NumberOfWays; ++i) {
            if (ways_[i].addressesMatch(theAddress)) {
                return ways_[i];
            }
        }
        auto index = count_;
        ways_[index].reset(theAddress);
        ++count_;
        if (count_ == NumberOfWays) {
            count_ = 0;
        }
        return ways_[index];
    }
    void clear() noexcept {
        for (auto& way : ways_) {
            way.clear();
        }
        count_ = 0;
    }
    static void begin() noexcept {
    }
public:
    [[nodiscard]] constexpr size_t size() const noexcept { return NumberOfWays; }
private:
    CacheEntry ways_[NumberOfWays];
    byte count_;
};
template<byte numTagBits, byte totalBitCount, byte numLowestBits, typename T, bool useSpecificTypeSizes>
using FiveWayRoundRobinCacheWay = RoundRobinCacheWay<numTagBits, totalBitCount, numLowestBits, T, useSpecificTypeSizes, 5>;
#endif //SXCHIPSET_ROUNDROBINCACHESET_H
