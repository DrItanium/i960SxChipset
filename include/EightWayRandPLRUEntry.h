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
template<byte numTagBits, byte totalBitCount, byte numLowestBits, typename T>
class EightWayRandPLRUCacheSet {
public:
    static constexpr auto NumberOfWays = 8;
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
        bits_ = 0;
    }
    [[nodiscard]] constexpr auto getWay(size_t index = 0) const noexcept { return ways_[index & WayMask]; }
    void setWay(CacheEntry& way, size_t index = 0) noexcept { ways_[index & WayMask] = &way; }
    [[nodiscard]] constexpr size_t size() const noexcept { return NumberOfWays; }
private:
    void updateFlags(byte index) noexcept {
        static constexpr bool ChooseLeft = false;
        static constexpr bool ChooseRight = true;
        switch (index) {
            case 0: // left, left, left
                a_ = ChooseLeft;
                c_ = ChooseLeft;
                g_ = ChooseLeft;
                break;
            case 1: // left, left, right
                a_ = ChooseRight;
                c_ = ChooseLeft;
                g_ = ChooseLeft;
                break;
            case 2: // left, right, left
                b_ = ChooseRight;
                c_ = ChooseLeft;
                g_ = ChooseLeft;
                break;
            case 3: // left, right, right
                b_ = ChooseRight;
                c_ = ChooseRight;
                g_ = ChooseLeft;
                break;
            case 4: // right, left, left
                d_ = ChooseLeft;
                f_ = ChooseLeft;
                g_ = ChooseRight;
                break;
            case 5: // right, left, right
                d_ = ChooseRight;
                f_ = ChooseLeft;
                g_ = ChooseRight;
                break;
            case 6: // right, right, left
                e_ = ChooseLeft;
                f_ = ChooseRight;
                g_ = ChooseRight;
                break;
            case 7: // right, right, right
                e_ = ChooseRight;
                f_ = ChooseRight;
                g_ = ChooseRight;
                break;
            default:
                break;
        }
    }
    [[nodiscard]] constexpr byte getLeastRecentlyUsed() const noexcept {
        if (g_) {
            // right
            if (f_) {
               // right (e)
               if (e_) {
                  // right
                  return 7;
               } else {
                  // left
                  return 6;
               }
            } else {
               // left (d)
               if (d_) {
                  // right
                  return 5;
               } else {
                  // left
                  return 4;
               }
            }
        } else {
            // left
            if (c_) {
                // right (b)
                if (b_) {
                   return 3;
                   // right
                } else {
                    return 2;
                   // left
                }
            } else {
                // left (a)
                if (a_) {
                   // right
                   return 1;
                } else {
                   // left
                   return 0;
                }
            }
        }
    }
private:
    CacheEntry* ways_[NumberOfWays] = { nullptr };
    // This is RandPLRU Tree so we need to organize things correctly, I'm going to try four groups of two
    union {
        byte bits_ = 0;
        struct {
            // depth first traversal ordering
            bool a_ : 1;  // left pair of left half
            bool b_ : 1;  // right pair of left half
            bool c_ : 1; // parent of a_ and b_ (left half)
            bool d_ : 1; // left pair of right half
            bool e_ : 1; // right pair of right half
            bool f_ : 1; // parent of d_ and e_ (right half)
            bool g_ : 1; // in a normal tree plru this would be the parent of c_ and f_
        };
    };
    byte mruBits_ = 0;
};

#endif //SXCHIPSET_EIGHTWAYPSEUDOLRUENTRY_H
