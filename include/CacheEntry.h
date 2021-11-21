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

#ifndef SXCHIPSET_CACHEENTRY_H
#define SXCHIPSET_CACHEENTRY_H
#include "TaggedCacheAddress.h"
#include "i960SxChipset.h"
/**
 * @brief Describes a single cache line which associates an address with 32 bytes of storage
 */
template<byte numTagBits, byte maxAddressBits, byte numLowestBits, typename T>
class CacheEntry final {
public:
    static constexpr size_t NumBytesCached = pow2(numLowestBits);
    static constexpr size_t NumWordsCached = NumBytesCached / sizeof(SplitWord16);
    static constexpr byte CacheEntryMask = NumWordsCached - 1;
    using TaggedAddress = ::TaggedAddress<numTagBits, maxAddressBits, numLowestBits>;
    using OffsetType = typename TaggedAddress::LowerType ;
    static constexpr OffsetType  InvalidCacheLineState = 0xFF;
    static constexpr OffsetType CleanCacheLineState = 0xFE;
public:
    void reset(TaggedAddress newTag) noexcept {
        // no match so pull the data in from main memory
        if (isDirty()) {
            // we compute the overall range as we go through this stuff
            byte end = ((highestUpdated_ - dirty_) + 1);
            //Serial.print(F("end offset: "));
            //Serial.println(end);
            T::write(TaggedAddress{key_, newTag.getTagIndex(), 0}.getAddress() + (dirty_ * sizeof(SplitWord16)),
                                     reinterpret_cast<byte *>(data + dirty_),
                                     sizeof(SplitWord16) * end);
        }
        dirty_ = CleanCacheLineState;
        highestUpdated_ = 0;
        // since we have called reset, now align the new address internally
        key_ = newTag.getRest();
        // this is a _very_ expensive operation
        T::read(TaggedAddress{key_, newTag.getTagIndex(), 0}.getAddress(), reinterpret_cast<byte*>(data), NumBytesCached);
    }
    /**
     * @brief Clear the entry without saving what was previously in it, necessary if the memory was reused for a different purpose
     */
    void clear() noexcept {
        // clear all flags
        dirty_ = InvalidCacheLineState;
        highestUpdated_ = 0;
        key_ = 0;
        for (auto& a : data) {
            a.wholeValue_ = 0;
        }
    }
    [[nodiscard]] constexpr bool matches(TaggedAddress addr) const noexcept { return isValid() && (addr.getRest() == key_); }
    [[nodiscard]] constexpr auto get(OffsetType offset) const noexcept { return data[offset].getWholeValue(); }
    void set(OffsetType offset, LoadStoreStyle style, SplitWord16 value) noexcept {
        // while unsafe, assume it is correct because we only get this from the ProcessorSerializer, perhaps directly grab it?
        auto &target = data[offset];
        if (auto oldValue = target.getWholeValue(); oldValue != value.getWholeValue()) {
            switch (style) {
                case LoadStoreStyle::Full16:
                    target = value;
                    break;
                case LoadStoreStyle::Lower8:
                    target.bytes[0] = value.bytes[0];
                    break;
                case LoadStoreStyle::Upper8:
                    target.bytes[1] = value.bytes[1];
                    break;
                default:
                    signalHaltState(F("BAD LOAD STORE STYLE FOR SETTING A CACHE LINE"));
            }
            // do a comparison at the end to see if we actually changed anything
            // the idea is that if the values don't change don't mark the cache line as dirty again
            // it may already be dirty but don't force the matter on any write
            // we can get here if it is a lower or upper 8 bit write so oldValue != value.getWholeValue()
            if (oldValue != target.getWholeValue()) {
                // consumes more flash to do it this way but we only update ram when we have something to change
                if (offset < dirty_) {
                    dirty_ = offset;
                }
                // compute the highest updated entry, useful for computing an upper transfer range
                if (offset > highestUpdated_) {
                    highestUpdated_ = offset;
                }
            }
        }
    }
    [[nodiscard]] constexpr bool isValid() const noexcept { return dirty_ != InvalidCacheLineState; }
    [[nodiscard]] constexpr bool isDirty() const noexcept { return dirty_ < NumWordsCached; }
    [[nodiscard]] constexpr bool isClean() const noexcept { return dirty_ == CleanCacheLineState; }

    OffsetType write(OffsetType startingOffset, byte *buf, OffsetType capacity) noexcept {
        auto* ptr = reinterpret_cast<byte*>(data) + startingOffset;
        for (OffsetType i = 0; i < capacity; ++i) {
            // while the comparison slows things down, it will make sure that updates which do not affect the cache will not
            auto oldPtr = ptr[i];
            ptr[i] = buf[i];
            if (oldPtr != ptr[i]) {
                if (i < dirty_) {
                    dirty_ = i;
                }
                if (i > highestUpdated_) {
                    highestUpdated_ = startingOffset;
                }
            }
        }
        return capacity;
    }
    OffsetType read(OffsetType startingOffset, byte *buf, OffsetType capacity) noexcept {
        auto* ptr = reinterpret_cast<byte*>(data) + startingOffset;
        for (OffsetType i = 0; i < capacity; ++i) {
            // while the comparison slows things down, it will make sure that updates which do not affect the cache will not
            buf[i] = ptr[i];
        }
        return capacity;
    }
private:
    SplitWord16 data[NumWordsCached]; // 16 bytes
    typename TaggedAddress::RestType key_ = 0;
    /**
     * @brief Describes lowest dirty word in a valid cache line; also denotes if the cache line is valid or not
     */
    OffsetType dirty_ = InvalidCacheLineState;
    /**
     * @brief The highest updated word in the cache line
     */
    OffsetType highestUpdated_ = 0;
};
#endif //SXCHIPSET_CACHEENTRY_H
