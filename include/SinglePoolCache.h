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

#ifndef SXCHIPSET_SINGLEPOOLCACHE_H
#define SXCHIPSET_SINGLEPOOLCACHE_H
#include "MCUPlatform.h"
#include "Pinout.h"
#include "TaggedCacheAddress.h"
#include "CacheEntry.h"
#include "ProcessorSerializer.h"

template<template<auto, auto, auto, typename> typename C, uint16_t numEntries, byte numAddressBits, byte numOffsetBits, typename T>
class SinglePoolCache {
private:
    using FakeCacheType = C<getNumberOfBitsForNumberOfEntries(numEntries), numAddressBits, numOffsetBits, T>;
public:
    static constexpr auto NumCacheWays = FakeCacheType::NumberOfWays;
    using CacheWay = C<getNumberOfBitsForNumberOfEntries(numEntries/NumCacheWays), numAddressBits, numOffsetBits, T>;
    static constexpr auto WayMask = CacheWay::WayMask;
    static constexpr auto MaximumNumberOfEntries = numEntries;
    static constexpr auto ActualNumberOfEntries = MaximumNumberOfEntries / CacheWay :: NumberOfWays;
    using CacheEntry = typename CacheWay::CacheEntry;
    using TaggedAddress = typename CacheWay::TaggedAddress;
    static constexpr auto NumBytesCached = CacheEntry::NumBytesCached;
    static constexpr auto NumWordsCached = CacheEntry::NumWordsCached;
    static constexpr auto CacheEntryMask = CacheEntry::CacheEntryMask;
public:
    [[nodiscard]] CacheEntry& getLine() noexcept {
        // only align if we need to reset the chip
        return getLine(TaggedAddress(ProcessorInterface::getAddress()));
    }
    [[nodiscard]] CacheEntry& getLine(const TaggedAddress& theAddress) noexcept {
        // only align if we need to reset the chip
        return entries_[theAddress.getTagIndex()].getLine(theAddress);
    }
    [[nodiscard]] auto* find(const TaggedAddress& theAddress) noexcept {
        return entries_[theAddress.getTagIndex()].find(theAddress);
    }
    [[nodiscard]] CacheEntry& reset(const TaggedAddress& theAddress) noexcept {
        return entries_[theAddress.getTagIndex()].reset(theAddress);
    }
    void clear() {
        // then clear both the way and underlying entries
        for (auto& a : entries_) {
            a.clear();
        }
    }
    byte* viewAsStorage() noexcept {
        return reinterpret_cast<byte*>(backingStorage_);
    }
    void begin() noexcept {
        // populate the lines from a separate block of entries known as the backing storage
        for (size_t i = 0; i < ActualNumberOfEntries; ++i) {
            auto& way = backingStorage_[i];
            CacheWay& targetEntry = entries_[i];
            for (size_t j = 0; j < CacheWay::NumberOfWays; ++j) {
                targetEntry.setWay(way[j], j);
            }
        }
        clear();
        // set everything up
    }
    constexpr auto getCacheSize() const noexcept { return sizeof(backingStorage_); }
    size_t write(uint32_t address, byte *buf, size_t capacity) noexcept {
        // reading and writing to the cache in a linear fashion will be kinda strange.
        // There are several ways to do this but I think the biggest requirement is that we only honor the portion that spans across one
        // cache line only!
        TaggedAddress theAddress{address};
        auto startingOffset = theAddress.getOffset();
        auto realCapacity = capacity;
        if (capacity > NumBytesCached) {
            realCapacity = NumBytesCached;
            // only decrement the length if we exceeded the size of the line
            realCapacity -= startingOffset;
        }
        return getLine(theAddress).write(startingOffset, buf, realCapacity);
    }
    size_t read(uint32_t address, byte *buf, size_t capacity) noexcept {
        // reading and writing to the cache in a linear fashion will be kinda strange.
        // There are several ways to do this but I think the biggest requirement is that we only honor the portion that spans across one
        // cache line only!
        TaggedAddress theAddress{address};
        auto startingOffset = theAddress.getOffset();
        auto realCapacity = capacity;
        if (capacity > NumBytesCached) {
            realCapacity = NumBytesCached;
            // only decrement the length if we exceeded the size of the line
            realCapacity -= startingOffset;
        }
        return getLine(theAddress).read(startingOffset, buf, realCapacity);
    }
private:
    CacheEntry backingStorage_[ActualNumberOfEntries][CacheWay::NumberOfWays];
    CacheWay entries_[ActualNumberOfEntries];
};

/**
 * @brief A wrapper around a single cache pool to make chaining possible
 * @tparam C The type of the cache line
 * @tparam backingStoreSize The number of bytes that this cache will use as storage (does not include tag bits only the underlying core storage)
 * @tparam numAddressBits  the number of address bits that make up the entire tag address
 * @tparam numOffsetBits  The number of bits that make up the storage within the bytes itself
 * @tparam T A static class that the cache data is read from and written to
 */
template<template<auto, auto, auto, typename> typename C,
         uint16_t backingStoreSize,
         byte numAddressBits,
         byte numOffsetBits,
         typename T>
struct Cache {
public:
    static constexpr auto NumOffsetBits = numOffsetBits;
    static constexpr auto NumBackingStoreBytes = backingStoreSize;
    static constexpr auto TotalEntryCount = NumBackingStoreBytes/ pow2(NumOffsetBits);
    using UnderlyingCache_t = SinglePoolCache<C, TotalEntryCount, numAddressBits, numOffsetBits, T>;
    using CacheLine = typename UnderlyingCache_t::CacheEntry;
    static constexpr auto CacheEntryMask = CacheLine::CacheEntryMask;
    static constexpr auto NumWordsCached = CacheLine::NumWordsCached;
public:
    Cache() = delete;
    ~Cache() = delete;
    Cache(const Cache&) = delete;
    Cache(Cache&&) = delete;
    /// @todo delete more of the default methods
    [[nodiscard]] static CacheLine& getLine() noexcept { return theCache_.getLine(); }
    [[nodiscard]] static size_t read(uint32_t address, byte *buf, size_t capacity) noexcept { return theCache_.read(address, buf, capacity); }
    [[nodiscard]] static size_t write(uint32_t address, byte *buf, size_t capacity) noexcept { return theCache_.write(address, buf, capacity); }
    static void begin() noexcept { theCache_.begin(); }
    static void clear() noexcept { theCache_.clear(); }
    [[nodiscard]] static constexpr auto getCacheSize() noexcept { return backingStoreSize; }
    [[nodiscard]] static auto viewAsStorage() noexcept { return theCache_.viewAsStorage(); }
private:
    static inline UnderlyingCache_t theCache_;
};

template<template<auto, auto, auto, typename> typename C, uint16_t backingStoreSize, byte numAddressBits, byte numOffsetBits, typename T>
using Cache_t = Cache<C, backingStoreSize, numAddressBits, numOffsetBits, T>;

template<template<auto, auto, auto, typename> typename C, uint16_t backingStoreSize, byte numAddressBits, byte numOffsetBits, typename T>
using CacheInstance_t = typename Cache_t<C, backingStoreSize, numAddressBits, numOffsetBits, T>::UnderlyingCache_t;

#endif //SXCHIPSET_SINGLEPOOLCACHE_H
