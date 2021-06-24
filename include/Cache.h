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
#include "Device.h"
template<uint32_t size = 16>
struct CacheLine {
public:
    union MemoryElement {
        explicit MemoryElement(uint16_t value = 0) noexcept : wordValue(value) { }
        uint16_t wordValue;
        uint8_t bytes[sizeof(uint16_t)];
    };
public:
    [[nodiscard]] constexpr bool respondsTo(uint32_t targetAddress) const noexcept {
        return valid_ && (address_ <= targetAddress) && (targetAddress < (address_ + CacheLineSize));
    }
    [[nodiscard]] constexpr uint8_t getByte(uint32_t targetAddress) const noexcept {
        auto base = computeCacheByteOffset(targetAddress);
        auto componentId = base >> 1;
        auto offsetId = base & 1;
        return components_[componentId].bytes[offsetId];
    }
    [[nodiscard]] constexpr auto getBaseAddress() const noexcept { return address_; }
    [[nodiscard]] constexpr auto cacheDirty() const noexcept { return dirty_; }
    void setByte(uint32_t address, uint8_t value) noexcept {
        dirty_ = true;
        auto base = computeCacheByteOffset(address);
        auto componentId = computeCacheWordOffset(address);
        auto offsetId = base & 1;
        components_[componentId].bytes[offsetId] = value;
    }
    [[nodiscard]] constexpr uint16_t getWord(uint32_t targetAddress) const noexcept {
        return components_[computeCacheWordOffset(targetAddress)].wordValue;
    }
    void setWord(uint32_t targetAddress, uint16_t value) noexcept {
        dirty_ = true;
        components_[computeCacheWordOffset(targetAddress)].wordValue = value;
    }
    [[nodiscard]] constexpr auto getCacheLineSize() const noexcept { return CacheLineSize; }
    static constexpr auto CacheLineSize = size;
    static constexpr auto ComponentSize = CacheLineSize / sizeof(MemoryElement);
    static constexpr auto CacheByteMask = CacheLineSize - 1;
    static constexpr auto isLegalCacheLineSize(uint32_t lineSize) noexcept {
        switch (lineSize) {
            case 16:
            case 32:
            case 64:
            case 128:
            case 256:
            case 512:
            case 1024:
            case 2048:
            case 4096:
            case 8192:
            case 16384:
            case 32768:
                return true;
            default:
                return false;
        }
    }
    static_assert(isLegalCacheLineSize(CacheLineSize),
    "CacheLineSize must be 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192, 16384, 32768");
    static constexpr uint32_t computeCacheByteOffset(uint32_t targetAddress) noexcept {
        return targetAddress & CacheByteMask;
    }
    static constexpr uint32_t computeCacheWordOffset(uint32_t targetAddress) noexcept {
        return computeCacheByteOffset(targetAddress) >> 1;
    }
    static constexpr uint32_t computeAlignedOffset(uint32_t targetAddress) noexcept {
        return targetAddress & ~CacheByteMask;
    }
    [[nodiscard]] MemoryElement* getMemoryBlock() noexcept { return components_; }
    void reset(uint32_t address, Device& thing) noexcept {
        byte* buf = reinterpret_cast<byte*>(components_);
        if (valid_ && dirty_) {
            thing.write(address_, buf, CacheLineSize);
        }
        dirty_ = false;
        valid_ = true;
        address_ = address;
        thing.read(address_, buf, CacheLineSize);
    }
    /**
     * @brief Invalidate the cache line and mark it as invalid
     * @param thing The memory thing to commit to
     */
    void invalidate(Device& thing) noexcept {
        if (valid_ && dirty_) {
            byte *buf = reinterpret_cast<byte *>(components_);
            thing.write(address_, buf, CacheLineSize);
            // purge the contents of memory
        }
        dirty_ = false;
        valid_ = false;
        address_ = 0;
    }
    [[nodiscard]] constexpr auto isValid() const noexcept { return valid_; }
private:
    /**
     * @brief The base address of the cache line
     */
    uint32_t address_;
    /**
     * @brief The cache line contents itself
     */
    MemoryElement components_[ComponentSize];
    static_assert(sizeof(components_) == CacheLineSize, "The backing store for the cache line is not the same size as the cache line size! Please adapt this code to work correctly for your target!"
    "");
    bool dirty_ = false;
    bool valid_ = false;
};
static_assert(CacheLine<16>::computeAlignedOffset(0xFFFF'FFFF) == 0xFFFF'FFF0);
template<uint32_t numLines = 16, uint32_t cacheLineSize = 32>
class DataCache : public Device {
public:
    using Parent = Device;
    using ASingleCacheLine = CacheLine<cacheLineSize>;
    static constexpr auto NumberOfCacheLines = numLines;
    static constexpr auto NumberOfCacheLinesMask = numLines - 1;
    static constexpr auto CacheLineSize = cacheLineSize;
    static constexpr auto DataCacheSize = CacheLineSize * NumberOfCacheLines;
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
    explicit DataCache(Device& backingStore) : Parent(backingStore.getBaseAddress(), backingStore.getEndAddress()), thing_(backingStore) { }
    [[nodiscard]] uint8_t getByte(uint32_t targetAddress) noexcept {
        for (const ASingleCacheLine & line : lines_) {
            if (line.respondsTo(targetAddress)) {
                // cache hit!
                return line.getByte(targetAddress);
            }
        }
        // cache miss
        // need to replace a cache line
        return cacheMiss(targetAddress).getByte(targetAddress);

    }
    [[nodiscard]] uint16_t getWord(uint32_t targetAddress) noexcept {
        for (const ASingleCacheLine& line : lines_) {
            if (line.respondsTo(targetAddress)) {
                // cache hit!
                return line.getWord(targetAddress);
            }
        }
        // cache miss
        // need to replace a cache line
        return cacheMiss(targetAddress).getWord(targetAddress);
    }
    void setByte(uint32_t targetAddress, uint8_t value) noexcept {
        for (ASingleCacheLine & line : lines_) {
            if (line.respondsTo(targetAddress)) {
                // cache hit!
                line.setByte(targetAddress, value);
                return;
            }
        }
        // cache miss
        // need to replace a cache line
        cacheMiss(targetAddress).setByte(targetAddress, value);
    }
    void setWord(uint32_t targetAddress, uint16_t value) noexcept {
        for (ASingleCacheLine& line : lines_) {
            if (line.respondsTo(targetAddress)) {
                // cache hit!
                line.setWord(targetAddress, value);
                return;
            }
        }
        // cache miss
        // need to replace a cache line
        cacheMiss(targetAddress).setWord(targetAddress, value);
    }
private:
    /**
     * @brief Looks through the given lines and does a random replacement (like arm cortex R)
     * @param targetAddress
     * @return The line that was updated
     */
    ASingleCacheLine& cacheMiss(uint32_t targetAddress) noexcept {
        auto alignedAddress = ASingleCacheLine::computeAlignedOffset(targetAddress);
        cacheEmpty_ = false;
        for (ASingleCacheLine &line : lines_) {
            if (!line.isValid()) {
                line.reset(alignedAddress, thing_);
                return line;
            }
        }
        // we had no free elements so choose one to replace
        auto targetCacheLine = random(NumberOfCacheLines);
        ASingleCacheLine &replacementLine = lines_[targetCacheLine];
        // generate an aligned address
        replacementLine.reset(alignedAddress, thing_);
        return replacementLine;
    }
private:
    Device& thing_;
    ASingleCacheLine lines_[NumberOfCacheLines];
    bool cacheEmpty_ = true;
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
        //return Device::blockWrite(address, buf, capacity);
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
    [[nodiscard]] const Device& getBackingStore() const noexcept { return thing_; }
    [[nodiscard]] Device& getBackingStore() noexcept { return thing_; }
private:
    /**
     * @brief Commit all entries in the cache back to the underlying memory type
     */
    void invalidateEntireCache() noexcept {
        if (!cacheEmpty_) {
            for (auto &line : lines_) {
                line.invalidate(thing_);
            }
            cacheEmpty_ = true;
        }
    }
};
#endif //I960SXCHIPSET_CACHE_H
