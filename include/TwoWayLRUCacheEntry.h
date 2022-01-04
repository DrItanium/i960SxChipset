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

#ifndef SXCHIPSET_TWOWAYLRUCACHEENTRY_H
#define SXCHIPSET_TWOWAYLRUCACHEENTRY_H
#include "MCUPlatform.h"
#include "Pinout.h"
#include "TaggedCacheAddress.h"
#include "CacheEntry.h"

template<byte numTagBits, byte totalBitCount, byte numLowestBits, typename T, bool useSpecificTypeSizes>
class TwoWayLRUCacheWay {
public:
    static constexpr auto NumberOfWays = 2;
    static constexpr auto WayMask = NumberOfWays - 1;
    using CacheEntry = ::CacheEntry<numTagBits, totalBitCount, numLowestBits, T, useSpecificTypeSizes>;
    using TaggedAddress = typename CacheEntry::TaggedAddress;
    static constexpr auto NumBytesCached = CacheEntry::NumBytesCached;
public:
    CacheEntry& getLine(TaggedAddress theAddress) noexcept {
        if (auto result = find (theAddress); result) {
            return *result;
        } else {
            return reset(theAddress);
        }
    }
    CacheEntry* find(TaggedAddress theAddress) noexcept {
        for (byte i = 0; i < NumberOfWays; ++i) {
            if (ways_[i].matches(theAddress)) {
                mostRecentlyUsed_ = (i != 0);
                return &ways_[i];
            }
        }
        return nullptr;
    }
    CacheEntry&
    reset(TaggedAddress theAddress) noexcept {
        auto index = (!mostRecentlyUsed_? 1 : 0);
        mostRecentlyUsed_ = (index != 0);
        ways_[index].reset(theAddress);
        return ways_[index];
    }
    void clear() noexcept {
        for (auto &way: ways_) {
            way.clear();
        }
        mostRecentlyUsed_ = false;
    }
    [[nodiscard]] constexpr size_t size() const noexcept { return NumberOfWays; }
private:
    CacheEntry ways_[NumberOfWays];
    bool mostRecentlyUsed_ = false;
};

#endif //SXCHIPSET_TWOWAYLRUCACHEENTRY_H
