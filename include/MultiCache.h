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
// Created by jwscoggins on 11/21/21.
//

#ifndef SXCHIPSET_MULTICACHE_H
#define SXCHIPSET_MULTICACHE_H
#include "MCUPlatform.h"
#include "Pinout.h"
#include "TaggedCacheAddress.h"
#include "CacheEntry.h"
#include "ProcessorSerializer.h"

template<template<auto, auto, auto, typename, bool> typename L,
        byte NumberOfCaches,
        uint16_t IndividualCacheSize,
        byte NumberOfAddressBits,
        byte CacheLineSize,
        typename T,
        bool useSpecificTypeSizes = true>
class MultiCache {
public:
    using Cache = CacheInstance_t<L, IndividualCacheSize, NumberOfAddressBits, CacheLineSize, T, useSpecificTypeSizes>;
    using CacheEntry = typename Cache::CacheEntry;
    using TaggedAddress = typename Cache::TaggedAddress;
    static constexpr auto NumWordsCached = Cache::NumWordsCached;
    static constexpr auto CacheEntryMask = Cache::CacheEntryMask;

    [[nodiscard]] CacheEntry& getLine() noexcept {
        // we have three cache pools to look through so check the first one
        TaggedAddress theAddress(ProcessorInterface::getAddress());
        if (auto* target = caches_[mostRecentHit_].find(theAddress); target) {
            ++counter_;
            return *target;
        }
        for (byte i = 0; i < NumberOfCaches; ++i) {
            if (i != mostRecentHit_) {
                if (auto* target = caches_[i].find(theAddress); target) {
                    mostRecentHit_ = i;
                    ++counter_;
                    return *target;
                }
            }
        }
        auto index = randomTable[counter_];
        ++counter_;
        // cache miss, do random replacement
        return caches_[index].reset(theAddress);
    }
    void clear() {
        for (auto& a : caches_) {
            a.clear();
        }
    }
    byte* viewAsStorage() noexcept {
        return reinterpret_cast<byte*>(&caches_[0]);
    }
    static constexpr auto RandomTableSize = 16;
    static constexpr auto NumCounterBits = numberOfBitsForCount(RandomTableSize);
    void begin() noexcept {
        counter_ = 0;
        for (auto i = 0; i < RandomTableSize; ++i) {
            randomTable[i] = random(NumberOfCaches);
        }
        for (auto& a : caches_) {
            a.begin();
        }
        // set everything up
    }
    [[nodiscard]] constexpr auto getCacheSize() const noexcept { return IndividualCacheSize; }
private:
    Cache caches_[NumberOfCaches];
    byte mostRecentHit_ = 0;
    byte counter_ : NumCounterBits;
    byte randomTable[RandomTableSize] = { 0};
};
#endif //SXCHIPSET_MULTICACHE_H
