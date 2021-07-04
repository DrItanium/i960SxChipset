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
template<uint32_t numLines = 16, uint32_t cacheLineSize = 32>
class DataCache : public MemoryThing {
public:
    static constexpr auto CacheLineSize = cacheLineSize;
    static constexpr auto ComponentSize = CacheLineSize / sizeof(SplitWord16);
    static constexpr auto CacheByteMask = CacheLineSize - 1;
    static constexpr auto AlignedOffsetMask = ~CacheByteMask;
    static constexpr auto CacheOffsetBitConsumption = numberOfAddressBitsForGivenByteSize(CacheLineSize);
    static constexpr auto NumberOfCacheLines = numLines;
    static constexpr auto DataCacheSize = CacheLineSize * NumberOfCacheLines;
    static constexpr auto NumberOfCacheSets = DataCacheSize / CacheLineSize;
    static constexpr auto Address_OffsetBitCount = CacheOffsetBitConsumption;
    static constexpr auto Address_SetIndexBitCount  = numberOfAddressBitsForGivenByteSize(NumberOfCacheSets);
    static constexpr auto Address_TagBitCount = (32 - (Address_OffsetBitCount + Address_SetIndexBitCount));
    static constexpr auto isLegalCacheLineSize(uint32_t lineSize) noexcept {
        switch (lineSize) {
            case 16:
            case 32:
            case 64:
            case 128:
            case 256:
            case 512:
            case 1024:
                return true;
            default:
                return false;
        }
    }
    static_assert(isLegalCacheLineSize(CacheLineSize), "CacheLineSize must be 16, 32, 64, 128, 256, or 512 bytes");
    static_assert(CacheOffsetBitConsumption != 0, "Invalid number of bits consumed by this cache line!");
    union CacheAddress {
        constexpr explicit CacheAddress(uint32_t baseValue = 0) noexcept : rawValue(baseValue) { }
        uint32_t rawValue = 0;
        struct {
            uint32_t offset : Address_OffsetBitCount;
            uint32_t index : Address_SetIndexBitCount;
            uint32_t tag : Address_TagBitCount;
        };
        [[nodiscard]] constexpr uint32_t getAlignedAddress() const noexcept { return Line ::computeAlignedOffset( rawValue); }
        [[nodiscard]] constexpr auto getByteOffset() const noexcept { return offset & 1; }
        [[nodiscard]] constexpr auto getComponentIndex() const noexcept { return offset >> 1; }
    };
    struct Line {
    public:
        [[nodiscard]] constexpr bool respondsTo(uint32_t targetTag) const noexcept {
            // just do a comparison with this design
            return valid_ && (targetTag == addr.tag);
            //return valid_ && ((address_ <= targetAddress) && (targetAddress < (address_ + CacheLineSize)));
        }
        [[nodiscard]] constexpr uint8_t getByte(uint32_t address) const noexcept { return getByte(CacheAddress{address}); }
        [[nodiscard]] constexpr uint8_t getByte(CacheAddress targetAddress) const noexcept {
            return components_[targetAddress.getComponentIndex()].bytes[targetAddress.getByteOffset()];
        }
        inline void setByte(uint32_t address, uint8_t value) noexcept  { setByte(CacheAddress{address}, value); }
        void setByte(CacheAddress address, uint8_t value) noexcept {
            dirty_ = true;
            components_[address.getComponentIndex()].bytes[address.getByteOffset()] = value;
        }
        [[nodiscard]] constexpr uint16_t getWord(uint32_t targetAddress) const noexcept { return getWord(CacheAddress{targetAddress}); }
        [[nodiscard]] constexpr uint16_t getWord(CacheAddress targetAddress) const noexcept {
            return components_[targetAddress.getComponentIndex()].wholeValue_;
        }
        inline void setWord(uint32_t targetAddress, uint16_t value) noexcept { setWord(CacheAddress{targetAddress}, value); }
        void setWord(CacheAddress targetAddress, uint16_t value) noexcept {
            dirty_ = true;
            components_[targetAddress.getComponentIndex()].wholeValue_ = value;
        }
        [[nodiscard]] static constexpr uint32_t computeAlignedOffset(uint32_t targetAddress) noexcept {
            return targetAddress & AlignedOffsetMask;
        }
        void reset(uint32_t address, MemoryThing& thing) noexcept {
            byte* buf = reinterpret_cast<byte*>(components_);
            if (valid_ && dirty_) {
                thing.write(addr.getAlignedAddress(), buf, CacheLineSize);
            }
            dirty_ = false;
            valid_ = true;
            addr.rawValue = address;
            thing.read(address, buf, CacheLineSize);
        }
        void invalidate(MemoryThing& thing) noexcept {
            if (valid_ && dirty_) {
                thing.write(addr.getAlignedAddress(), reinterpret_cast<byte*>(components_), CacheLineSize);
            }
            status_ = 0;
            addr.rawValue = 0;
        }
        [[nodiscard]] constexpr auto isValid() const noexcept { return valid_; }
    private:
        /**
         * @brief The base address of the cache line
         */
         CacheAddress addr;
        /**
         * @brief The cache line contents itself
         */
        SplitWord16 components_[ComponentSize];
        static_assert(sizeof(components_) == CacheLineSize, "The backing store for the cache line is not the same size as the cache line size! Please adapt this code to work correctly for your target!");
        union {
            byte status_ = 0;
            struct
            {
                bool dirty_: 1;
                bool valid_: 1;
            };
        };
    };
    static_assert(sizeof(CacheAddress) == sizeof(uint32_t));
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
    static constexpr auto isLegalNumberOfWays(uint32_t num) noexcept {
        switch (num) {
            case 1:
            case 2:
            case 4:
            case 8:
            case 16:
            case 32:
                return true;
            default:
                return false;
        }
    }
    static_assert(DataCacheSize <= TargetBoard::oneFourthSRAMAmountInBytes(), "Overall cache size must be less than or equal to one fourth of SRAM");
    static_assert(isLegalNumberOfCacheLines(NumberOfCacheLines));
    explicit DataCache(MemoryThing& backingStore) : MemoryThing(backingStore.getBaseAddress(), backingStore.getEndAddress()), thing_(backingStore) { }
    [[nodiscard]] uint8_t getByte(uint32_t targetAddress) noexcept {
        return getCacheLine(targetAddress).getByte(targetAddress);
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
    /**
     * @brief Looks through the given lines and does a random replacement (like arm cortex R)
     * @param targetAddress
     * @return The line that was updated
     */
    Line& getCacheLine(uint32_t targetAddress) noexcept {
            // direct mapped cache
            CacheAddress addr(targetAddress);
            auto &replacementLine = lines_[addr.index];
            if (!replacementLine.respondsTo(addr.tag)) {
                replacementLine.reset(addr.getAlignedAddress(), thing_);
            }
            return replacementLine;
    }
public:
    uint8_t read8(Address address) noexcept override {
        if (enabled_) {
            return getByte(address);
        } else {
            return thing_.read8(address) ;
        }
    }
    uint16_t read16(Address address) noexcept override {
        if (enabled_) {
            return getWord(address);
        } else {
            return thing_.read16(address) ;
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
        // since we have all the time in the world on startup, just fill the cache up to speed up startup
    }
    size_t blockWrite(Address address, uint8_t *buf, size_t capacity) noexcept override {
        // use the cache where it makes sense
        if (enabled_) {
            size_t numWritten = 0;
            for (size_t i = 0; i < capacity; ++i, ++address, ++numWritten) {
                setByte(address, buf[i]);
            }
            return numWritten;
        } else {
           return thing_.blockWrite(address, buf, capacity);
        }
    }
    size_t blockRead(Address address, uint8_t *buf, size_t capacity) noexcept override {
        if (enabled_) {
            size_t numRead = 0;
            for (size_t i = 0; i < capacity; ++i, ++address, ++numRead) {
                buf[i] = getByte(address);
            }
            return numRead;
        } else {
            return thing_.blockRead(address, buf, capacity);
        }
    }
    void disableCache() noexcept override {
        if (enabled_) {
            enabled_ = false;
            invalidate();
        }
    }
    void enableCache() noexcept override {
        if (!enabled_) {
            enabled_ = true;
        }
    }
private:
    void invalidate() noexcept {
        for (auto& line : lines_) {
            line.invalidate(thing_);
        }
    }
private:
    MemoryThing& thing_;
    Line lines_[NumberOfCacheLines];
    bool enabled_ = true;
};

#endif //I960SXCHIPSET_CACHE_H
