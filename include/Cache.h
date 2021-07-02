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
// Created by jwscoggins on 6/16/21.
//

#ifndef I960SXCHIPSET_CACHE_H
#define I960SXCHIPSET_CACHE_H

#include <Arduino.h>
#include "MCUPlatform.h"
constexpr auto numberOfAddressBitsForGivenByteSize(uint32_t numBytes) noexcept {
    switch (numBytes) {
        case 1: return 0;
        case 2: return 1;
        case 4: return 2;
        case 8: return 3;
        case 16: return 4;
        case 32: return 5;
        case 64: return 6;
        case 128: return 7;
        case 256: return 8;
        case 512: return 9;
        case 1024: return 10;
        case 2048: return 11;
        case 4096: return 12;
        case 8192: return 13;
        case 16384: return 14;
        case 32768: return 15;
        default: return 0;
    }
}
template<uint32_t size = 16>
struct CacheLine {
public:
    [[nodiscard]] constexpr bool respondsTo(uint32_t targetAddress) const noexcept {
        return valid_ && ((address_ <= targetAddress) && (targetAddress < endAddress_));
    }
    [[nodiscard]] constexpr uint8_t getByte(uint32_t targetAddress) const noexcept {
        auto base = computeCacheByteOffset(targetAddress);
        auto componentId = base >> 1;
        auto offsetId = base & 1;
        return components_[componentId].bytes[offsetId];
    }
    void setByte(uint32_t address, uint8_t value) noexcept {
        dirty_ = true;
        auto base = computeCacheByteOffset(address);
        auto componentId = computeCacheWordOffset(address);
        auto offsetId = base & 1;
        components_[componentId].bytes[offsetId] = value;
    }
    [[nodiscard]] constexpr uint16_t getWord(uint32_t targetAddress) const noexcept {
        return components_[computeCacheWordOffset(targetAddress)].wholeValue_;
    }
    void setWord(uint32_t targetAddress, uint16_t value) noexcept {
        dirty_ = true;
        components_[computeCacheWordOffset(targetAddress)].wholeValue_ = value;
    }
    static constexpr auto isLegalCacheLineSize(uint32_t lineSize) noexcept {
        switch (lineSize) {
            case 16:
            case 32:
            case 64:
            case 128:
            case 256:
            case 512:
                return true;
            default:
                return false;
        }
    }
    static constexpr auto CacheLineSize = size;
    static constexpr auto ComponentSize = CacheLineSize / sizeof(SplitWord16);
    static constexpr auto CacheByteMask = CacheLineSize - 1;
    static constexpr auto AlignedOffsetMask = ~CacheByteMask;
    static constexpr auto CacheOffsetBitConsumption = numberOfAddressBitsForGivenByteSize(CacheLineSize);
    static_assert(isLegalCacheLineSize(CacheLineSize), "CacheLineSize must be 16, 32, 64, 128, 256, or 512 bytes");
    static_assert(CacheOffsetBitConsumption != 0, "Invalid number of bits consumed by this cache line!");
    [[nodiscard]] static constexpr uint32_t computeCacheByteOffset(uint32_t targetAddress) noexcept {
        return targetAddress & CacheByteMask;
    }
    [[nodiscard]] static constexpr uint32_t computeCacheWordOffset(uint32_t targetAddress) noexcept {
        return computeCacheByteOffset(targetAddress) >> 1;
    }
    [[nodiscard]] static constexpr uint32_t computeAlignedOffset(uint32_t targetAddress) noexcept {
        return targetAddress & AlignedOffsetMask;
    }
    void reset(uint32_t address, MemoryThing& thing) noexcept {
        byte* buf = reinterpret_cast<byte*>(components_);
        if (valid_ && dirty_) {
            thing.write(address_, buf, CacheLineSize);
        }
        dirty_ = false;
        valid_ = true;
        address_ = address;
        endAddress_ = address + CacheLineSize;
        thing.read(address_, buf, CacheLineSize);
    }
    /**
     * @brief Invalidate the cache line and mark it as invalid
     * @param thing The memory thing to commit to
     */
    void invalidate(MemoryThing& thing) noexcept {
        if (valid_ && dirty_) {
            byte *buf = reinterpret_cast<byte *>(components_);
            thing.write(address_, buf, CacheLineSize);
            // purge the contents of memory
        }
        dirty_ = false;
        valid_ = false;
        address_ = 0;
        endAddress_ = 0;
    }
    [[nodiscard]] constexpr auto isValid() const noexcept { return valid_; }
private:
    /**
     * @brief The base address of the cache line
     */
    uint32_t address_ = 0;
    uint32_t endAddress_ = 0;
    /**
     * @brief The cache line contents itself
     */
    SplitWord16 components_[ComponentSize];
    static_assert(sizeof(components_) == CacheLineSize, "The backing store for the cache line is not the same size as the cache line size! Please adapt this code to work correctly for your target!");
    bool dirty_ = false;
    bool valid_ = false;
};
static_assert(CacheLine<16>::computeAlignedOffset(0xFFFF'FFFF) == 0xFFFF'FFF0);
template<uint32_t numLines = 16, uint32_t cacheLineSize = 32>
class DataCache : public MemoryThing {
public:
    using ASingleCacheLine = CacheLine<cacheLineSize>;
    static constexpr auto NumberOfCacheLines = numLines;
    static constexpr auto NumberOfCacheLinesMask = numLines - 1;
    static constexpr auto CacheLineSize = cacheLineSize;
    static constexpr auto DataCacheSize = CacheLineSize * NumberOfCacheLines;
    static constexpr auto CacheLineAddressMask = ASingleCacheLine::CacheByteMask;
    static constexpr auto TagOffsetShiftAmount = numberOfAddressBitsForGivenByteSize(ASingleCacheLine::CacheOffsetBitConsumption);
    //we want this mask to start immediately after the the cache line mask
    static constexpr Address TagOffsetMask = NumberOfCacheLinesMask << TagOffsetShiftAmount;
    static constexpr auto isLegalNumberOfCacheLines(uint32_t num) noexcept {
        switch (num) {
            case 1:
            case 2:
            case 4:
            case 8:
            case 16:
            case 32:
            case 64:
            case 128:
            case 256:
                return true;
            default:
                return false;
        }
    }
    static_assert(DataCacheSize <= TargetBoard::oneFourthSRAMAmountInBytes(), "Overall cache size must be less than or equal to one fourth of SRAM");
    static_assert(isLegalNumberOfCacheLines(NumberOfCacheLines));
    explicit DataCache(MemoryThing& backingStore) : MemoryThing(backingStore.getBaseAddress(), backingStore.getEndAddress()), thing_(backingStore) { }
    [[nodiscard]] uint8_t getByte(uint32_t targetAddress) noexcept {
        return getCacheLine().getByte(targetAddress);
    }
    [[nodiscard]] uint16_t getWord(uint32_t targetAddress) noexcept {
        return getCacheLine(targetAddress).getWord(targetAddress);
    }
    void setByte(uint32_t targetAddress, uint8_t value) noexcept {
        getCacheLine(targetAddress).setByte(targetAddress, value);
    }
    void setWord(uint32_t targetAddress, uint16_t value) noexcept {
        getCacheLine(targetAddress).setWord(targetAddress, value);
    }
private:
    static constexpr auto computeTargetLine(uint32_t address) noexcept {
        if constexpr (NumberOfCacheLines == 0) {
            return 0;
        } else {
            // we want the bits following the offset into the cache line itself
            return (address & (~TagOffsetMask)) >> TagOffsetShiftAmount; // make sure that we actually get the thing itself
        }
    }
    /**
     * @brief Looks through the given lines and does a random replacement (like arm cortex R)
     * @param targetAddress
     * @return The line that was updated
     */
    ASingleCacheLine& getCacheLine(uint32_t targetAddress) noexcept {
        // instead of using random directly, use an incrementing counter to choose a line to invalidate
        // thus at no point will we actually know what we've dropped.
        auto alignedAddress = ASingleCacheLine::computeAlignedOffset(targetAddress);
        auto lineToCheck = computeTargetLine(alignedAddress);
        auto& replacementLine = lines_[lineToCheck];
        // now check and see if that line needs to be replaced or not
        if (!replacementLine.respondsTo(alignedAddress)) {
            replacementLine.reset(alignedAddress, thing_);
        }
        return replacementLine;
    }
public:
    uint8_t read8(Address address) noexcept override {
        if (enabled_) {
            return getByte(address);
        } else {
            return thing_.read8(address);
        }
    }
    uint16_t read16(Address address) noexcept override {
        if (enabled_) {
            return getWord(address);
        } else {
            return thing_.read16(address);
        }
    }
    [[nodiscard]] bool respondsTo(Address address) const noexcept override {
        return thing_.respondsTo(address);
    }
    void write8(Address address, uint8_t value) noexcept override {
        if (enabled_) {
            setByte(address, value);
        } else {
            thing_.write8(address, value);
        }
    }
    void write16(Address address, uint16_t value) noexcept override {
        if (enabled_) {
            setWord(address, value);
        } else {
            thing_.write16(address, value);
        }
    }
    [[nodiscard]] Address
    makeAddressRelative(Address input) const noexcept override {
        return thing_.makeAddressRelative(input);
    }

    void begin() noexcept override {
        thing_.begin();
        // cache the first entry
        (void)getByte(0);
    }
    size_t blockWrite(Address address, uint8_t *buf, size_t capacity) noexcept override {
        //return MemoryThing::blockWrite(address, buf, capacity);
        /// @todo see if it makes more sense to require that the user disables the cache if they need to do such a thing

        if (enabled_) {
            // use the cache where it makes sense
            size_t numWritten = 0;
            for (size_t i = 0; i < capacity; ++i, ++address, ++numWritten) {
                setByte(address, buf[i]);
            }
            return numWritten;
        } else {
            // directly write to the backing store
            return thing_.blockWrite(address, buf, capacity);
        }
    }
    size_t blockRead(Address address, uint8_t *buf, size_t capacity) noexcept override {
        // we want to directly read from the underlying memory thing using the buffer so we need to do cache coherency checks as well
        /// @todo figure out if we actually need to disable the cache when performing a read from memory. I don't think we need to actually
        if (enabled_) {
            size_t numRead = 0;
            for (size_t i = 0; i < capacity; ++i, ++address, ++numRead) {
                buf[i] = getByte(address);
            }
            return numRead;
        } else {
            // when the cache is disabled do the direct reads and writes
            return thing_.blockRead(address, buf, capacity);
        }
    }
    void disableCache() noexcept override {
        if (enabled_) {
            enabled_ = false;
            invalidateEntireCache();
        }
    }
    void enableCache() noexcept override {
        if (!enabled_) {
            enabled_ = true;
        }
    }
private:
    /**
     * @brief Commit all entries in the cache back to the underlying memory type
     */
    void invalidateEntireCache() noexcept {
        for (auto &line : lines_) {
            line.invalidate(thing_);
        }
    }
private:
    MemoryThing& thing_;
    ASingleCacheLine lines_[NumberOfCacheLines];
    bool enabled_ = true;
};
#endif //I960SXCHIPSET_CACHE_H
