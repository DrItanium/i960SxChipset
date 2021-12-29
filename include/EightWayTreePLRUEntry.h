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

#ifndef SXCHIPSET_EIGHTWAYTREEPLRUENTRY_H
#define SXCHIPSET_EIGHTWAYTREEPLRUENTRY_H
#include "MCUPlatform.h"
#include "Pinout.h"
#include "TaggedCacheAddress.h"
#include "CacheEntry.h"

template<byte numTagBits, byte totalBitCount, byte numLowestBits, typename T, bool useSpecificTypeSizes = true>
class EightWayTreePLRUCacheSet {
public:
    static constexpr auto NumberOfWays = 8;
    static constexpr auto WayMask = NumberOfWays - 1;
    using CacheEntry = ::CacheEntry<numTagBits, totalBitCount, numLowestBits, T, useSpecificTypeSizes>;
    using TaggedAddress = typename CacheEntry::TaggedAddress;
    static constexpr auto NumBytesCached = CacheEntry::NumBytesCached;
public:
    __attribute__((noinline)) CacheEntry& getLine(TaggedAddress theAddress) noexcept {
        byte target = NumberOfWays;
        for (byte i = 0; i < NumberOfWays; ++i) {
            if (ways_[i]->matches(theAddress)) {
                updateFlags(i);
                return *ways_[i];
            } else if (!ways_[i]->isValid() && target == NumberOfWays){
                target = i;
            }
        }
        auto index = target < NumberOfWays ? target : getLeastRecentlyUsed();
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
    static constexpr byte generateLookupEntry(byte index) noexcept {
        constexpr auto GMask = 0b0100'0000;
        constexpr auto FMask = 0b0010'0000;
        constexpr auto EMask = 0b0001'0000;
        constexpr auto DMask = 0b0000'1000;
        constexpr auto CMask = 0b0000'0100;
        constexpr auto BMask = 0b0000'0010;
        constexpr auto AMask = 0b0000'0001;
        // invert the bits to make it easy to figure out the target
        // so an index of 0b000'0000 -> 0b111'1111 which would be G -> F -> E -> 7
        // 0b111'1111 -> 0b000'0000 which would be G -> C -> A -> 0
        byte inv = ~index;
        auto maskedCorrectly = inv & 0b0111'1111;
        if ((maskedCorrectly & GMask)) {
            // right
            if (maskedCorrectly & FMask) {
                // right
                if (maskedCorrectly & EMask) {
                    // right
                    return 7;
                } else {
                    return 6;
                }
            } else {
                // left
                if (maskedCorrectly & DMask) {
                    // right
                    return 5;
                } else {
                    // left
                    return 4;
                }
            }
        } else {
            // left
            if (maskedCorrectly & CMask) {
               // right
               if (maskedCorrectly & BMask) {
                   // right
                   return 3;
               } else {
                   // left
                   return 2;
               }
            } else {
               // left
               if (maskedCorrectly & AMask) {
                   // right
                   return 1;
               } else {
                   // left
                   return 0;
               }
            }
        }
    }
    static constexpr byte LookupTable[128] {
        // 0 means left, left, left, left, left, left, left
        //         6   , 5   , 4   , 3   , 2   , 1   , 0
        //         g   , f   , e   , d   , c   , b   , a
#define X(row) \
            generateLookupEntry((8 * (row)) + 0), \
            generateLookupEntry((8 * (row)) + 1), \
            generateLookupEntry((8 * (row)) + 2), \
            generateLookupEntry((8 * (row)) + 3), \
            generateLookupEntry((8 * (row)) + 4), \
            generateLookupEntry((8 * (row)) + 5), \
            generateLookupEntry((8 * (row)) + 6), \
            generateLookupEntry((8 * (row)) + 7)
            X(0),
            X(1),
            X(2),
            X(3),
            X(4),
            X(5),
            X(6),
            X(7),
            X(8),
            X(9),
            X(10),
            X(11),
            X(12),
            X(13),
            X(14),
            X(15),
#undef X
    };
    [[nodiscard]] constexpr byte getLeastRecentlyUsed() const noexcept {
        return LookupTable[bits_];
    }
private:
    CacheEntry* ways_[NumberOfWays] = { nullptr };
    // This is RandPLRU Tree so we need to organize things correctly, I'm going to try four groups of two
    union {
        byte bits_ = 0b0111'1111;
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
};

#endif //SXCHIPSET_EIGHTWAYPSEUDOLRUENTRY_H
