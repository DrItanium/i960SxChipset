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
// Created by jwscoggins on 12/29/21.
//

#ifndef SXCHIPSET_SRAMDATACONTAINER_H
#define SXCHIPSET_SRAMDATACONTAINER_H
#include "MCUPlatform.h"
#include "Pinout.h"
#include "23LC1024.h"
#include "TaggedCacheAddress.h"

/**
 * @brief Wrapper that caches data from the true backing store into a much faster block of spi sram. The class which uses this class is unaware
 * of the caching that is going on.
 * @tparam T The underlying type to talk to when we don't contain the given data here
 */
template<typename T>
class SRAMDataContainer {
public:
    using BackingStore = T;
    using SRAM = SRAM_23LC1024Chip<i960Pinout::CACHE_EN>;
    static constexpr auto Capacity = SRAM::Capacity;
    static constexpr auto CacheLineSize = 512; // bytes
    static constexpr auto CacheLineBits = getNumberOfBitsForNumberOfEntries(CacheLineSize);
    static constexpr auto NumLines = Capacity / CacheLineSize;
    static constexpr auto NumLineBits = getNumberOfBitsForNumberOfEntries(NumLines);
    using CacheAddress = TaggedAddress<NumLineBits, 32, CacheLineBits>;
    using KeyType = typename CacheAddress::RestType;
    using TagType = typename CacheAddress::TagType ;
    // list our assumptions
    static_assert(CacheAddress::NumLowestBits == 9, "This class is written assuming a 1024 byte segment");
    static_assert(CacheAddress::NumTagBits == 8, "This class is written assuming a 1024 byte segment");
    static_assert(CacheAddress::NumRestBits == 15, "This class is written assuming a 1024 byte segment");
    SRAMDataContainer() = delete;
    ~SRAMDataContainer() = delete;
    SRAMDataContainer(SRAMDataContainer&&) = delete;
    SRAMDataContainer(const SRAMDataContainer&) = delete;
    SRAMDataContainer operator=(SRAMDataContainer&&) = delete;
    SRAMDataContainer operator=(const SRAMDataContainer&) = delete;
private:
    /**
     * @brief Each tag entry holds onto the current status of the segment, the backing store base address, and the base address in sram
     */
    struct Tag {
        enum class Status : byte {
            Invalid,
            Clean,
            Dirty,
        };
        Status status_ = Status::Invalid;
        KeyType key_ = 0;
        // the key will be used to reconstruct the sram and backing storage addresses
        [[nodiscard]] constexpr bool dirty() const noexcept { return status_ == Status::Dirty; }
        [[nodiscard]] constexpr bool clean() const noexcept { return status_ == Status::Clean; }
        [[nodiscard]] constexpr bool invalid() const noexcept { return status_ == Status::Invalid; }
        [[nodiscard]] constexpr bool valid() const noexcept { return !invalid(); }
        void setKey(KeyType value) noexcept { key_ = value; }
        void markDirty() noexcept { status_ = Status::Dirty; }
        void markClean() noexcept { status_ = Status::Clean; }
        void markInvalid() noexcept { status_ = Status::Invalid; }
        [[nodiscard]] constexpr CacheAddress reconstructBackingStoreAddress(TagType tagIndex) const noexcept {
            return CacheAddress{ key_, tagIndex };
        }
        [[nodiscard]] constexpr CacheAddress reconstructSramAddress(TagType tagIndex) const noexcept {
            return CacheAddress {0, tagIndex };
        }
        [[nodiscard]] constexpr auto matches(KeyType other) const noexcept {
            return valid() && (key_ == other);
        }
        [[nodiscard]] constexpr auto matches(const CacheAddress& other) const noexcept {
            return matches(other.getRest());
        }
        void clear() noexcept {
            status_ = Status::Invalid;
            key_ = 0;
        }
    };
    static size_t readFromSRAMToBuffer(const CacheAddress& theAddress, byte* buffer, size_t count) noexcept {
        return SRAM::read(theAddress.getAddress(), buffer, count);
    }
    static size_t writeToSRAMFromBuffer(const CacheAddress& theAddress, byte* buffer, size_t count) noexcept {
        return SRAM::write(theAddress.getAddress(), buffer, count);
    }
    static void fillBufferFromSRAM(const CacheAddress& theAddress) noexcept {
        readFromSRAMToBuffer(theAddress, transferBuffer_, CacheLineSize);
    }
    static void fillBufferFromBackingStore(const CacheAddress& theAddress)  noexcept {
        BackingStore :: read(theAddress.getAddress(), transferBuffer_, CacheLineSize);
    }
    static void saveBufferToBackingStore(const CacheAddress& theAddress) noexcept {
        BackingStore :: write(theAddress.getAddress(), transferBuffer_, CacheLineSize);
    }
    static void saveBufferToSRAM(const CacheAddress& theAddress) noexcept {
        writeToSRAMFromBuffer(theAddress, transferBuffer_, CacheLineSize);
    }
    static void transferSegmentFromSRAMToBackingStore(const CacheAddress& sramAddress, const CacheAddress& backingStoreAddress) noexcept {
        fillBufferFromSRAM(sramAddress);
        saveBufferToBackingStore(backingStoreAddress);
    }
    static void transferSegmentFromSRAMToBackingStore(const Tag& targetTag, TagType tagIndex) noexcept {
        auto sramAddress = targetTag.reconstructSramAddress(tagIndex);
        auto backingStoreAddress = targetTag.reconstructBackingStoreAddress(tagIndex);
        transferSegmentFromSRAMToBackingStore(sramAddress, backingStoreAddress);
    }
    static void transferSegmentFromBackingStoreToSRAM(const CacheAddress& sramAddress, const CacheAddress& backingStoreAddress) noexcept {
        fillBufferFromBackingStore(backingStoreAddress);
        saveBufferToSRAM(sramAddress);
    }
    static void transferSegmentFromBackingStoreToSRAM(const Tag& targetTag, TagType tagIndex) noexcept {
        auto sramAddress = targetTag.reconstructSramAddress(tagIndex);
        auto backingStoreAddress = targetTag.reconstructBackingStoreAddress(tagIndex);
        transferSegmentFromBackingStoreToSRAM(sramAddress, backingStoreAddress);
    }
    static void loadSegmentIntoSRAM(const CacheAddress& addr, Tag& theTag) noexcept {
        if (theTag.dirty()) {
            // okay so the tag is currently pointing to dirty data
            // we need to save that back to backing store first
            // use the current tag information to do this
            transferSegmentFromSRAMToBackingStore(theTag, addr.getTagIndex());
        }
        // Either the tag is invalid or it is a clean segment. Either way just dump it without committing back
        // get the aligned base address
        auto alignedAddress = addr.aligned();
        // update the tag
        theTag.setKey(alignedAddress.getRest());
        theTag.markClean();
        // then do the transfer itself
        transferSegmentFromBackingStoreToSRAM(theTag, addr.getTagIndex());
    }
    static size_t write(const CacheAddress& addr, byte* buf, size_t capacity) noexcept {
        // get the tag and see if we have a match
        auto& theTag = tags_[addr.getTagIndex()];
        if (!theTag.matches(addr)) {
            // get the right segment into memory
            loadSegmentIntoSRAM(addr, theTag);
        }
        // then write our modifications to the sram
        writeToSRAMFromBuffer(addr, buf, capacity);
        theTag.markDirty();
        return capacity;
    }
    static size_t read(const CacheAddress& addr, byte* buf, size_t capacity) noexcept {
        // get the tag and see if we have a match
        auto& theTag = tags_[addr.getTagIndex()];
        if (!theTag.matches(addr)) {
            // get the right segment into memory
            loadSegmentIntoSRAM(addr, theTag);
        }
        // then write our modifications to the sram
        readFromSRAMToBuffer(addr, buf, capacity);
        return capacity;
    }
public:
    static size_t write(uint32_t address, byte *buf, size_t capacity) noexcept {
        CacheAddress addr(address);
        return write(addr, buf, capacity);
    }
    static size_t read(uint32_t address, byte *buf, size_t capacity) noexcept {
        CacheAddress currAddr(address);
        return read(currAddr, buf, capacity);
    }
    static void clear() noexcept {
        for (auto& a : tags_) {
            a.clear();
        }
        memset(transferBuffer_, 0, CacheLineSize);
    }
    static void begin() noexcept {
        clear();
        BackingStore ::begin();
        SRAM::begin();
    }
private:
    static inline Tag tags_[NumLines];
    static inline byte transferBuffer_[CacheLineSize];
};

#endif //SXCHIPSET_SRAMDATACONTAINER_H
