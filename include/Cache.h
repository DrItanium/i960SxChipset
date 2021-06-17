//
// Created by jwscoggins on 6/16/21.
//

#ifndef I960SXCHIPSET_CACHE_H
#define I960SXCHIPSET_CACHE_H
#include <Arduino.h>
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
                return true;
            default:
                return false;
        }
    }
    static_assert(isLegalCacheLineSize(CacheLineSize),
    "CacheLineSize must be 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, or 8192");
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
    void reset(uint32_t address, MemoryThing* thing) noexcept {
        byte* buf = reinterpret_cast<byte*>(components_);
        if (valid_ && dirty_) {
            thing->write(address_, buf, CacheLineSize);
        }
        dirty_ = false;
        valid_ = true;
        address_ = address;
        thing->read(address_, buf, CacheLineSize);
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
class DataCache {
public:
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
    static_assert(DataCacheSize <= 4096, "Overall cache size must be less than or equal to 4k of sram");
    static_assert(isLegalNumberOfCacheLines(NumberOfCacheLines));
    explicit DataCache(MemoryThing* backingStore) : thing_(backingStore) { }
    [[nodiscard]] uint8_t getByte(uint32_t targetAddress) noexcept {
        if (chipsetFunctions.displayCacheLineUpdates()) {
            Serial.print(F("\tGetByte: 0x"));
            Serial.println(targetAddress, HEX);
        }
        for (const ASingleCacheLine & line : lines_) {
            if (line.respondsTo(targetAddress)) {
                // cache hit!
                auto result = line.getByte(targetAddress);
                if (chipsetFunctions.displayCacheLineUpdates()) {
                    Serial.print(F("\t\tResult: 0x"));
                    Serial.println(result, HEX);
                }
                return result;

            }
        }
        // cache miss
        // need to replace a cache line
        auto hit = cacheMiss(targetAddress).getByte(targetAddress);
        if (chipsetFunctions.displayCacheLineUpdates()) {
            Serial.print(F("\t\tHIT: 0x"));
            Serial.println(hit, HEX);
        }
        return hit;

    }
    [[nodiscard]] uint16_t getWord(uint32_t targetAddress) noexcept {
        if (chipsetFunctions.displayCacheLineUpdates()) {
            Serial.print(F("\tGetWord: 0x"));
            Serial.print(targetAddress, HEX);
        }
        for (const ASingleCacheLine& line : lines_) {
            if (line.respondsTo(targetAddress)) {
                // cache hit!
                auto result = line.getWord(targetAddress);
                if (chipsetFunctions.displayCacheLineUpdates()) {
                    Serial.print(F("\t\tResult: 0x"));
                    Serial.println(result, HEX);
                }
                return result;
            }
        }
        // cache miss
        // need to replace a cache line
        auto hit = cacheMiss(targetAddress).getWord(targetAddress);
        if (chipsetFunctions.displayCacheLineUpdates()) {
            Serial.print(F("\t\tHIT: 0x"));
            Serial.println(hit, HEX);
        }
        return hit;
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
        if (chipsetFunctions.displayCacheLineUpdates()) {
            Serial.println(F("Cache Miss: handling cache miss"));
            Serial.print(F("\tTarget Address: 0x"));
            Serial.println(targetAddress, HEX);
            Serial.print(F("\tAligned Address: 0x"));
            Serial.println(alignedAddress, HEX);
        }
        // use random number generation to do this
        for (ASingleCacheLine& line : lines_) {
            if (!line.isValid()) {
                line.reset(alignedAddress, thing_);
                return line;
            }
        }
        // we had no free elements so choose one to replace
        auto targetCacheLine = rand() & NumberOfCacheLinesMask;
        ASingleCacheLine& replacementLine = lines_[targetCacheLine];
        if (chipsetFunctions.displayCacheLineUpdates()) {
            Serial.print(F("\tNumber of cache lines: 0x"));
            Serial.println(NumberOfCacheLines, HEX);
            Serial.print(F("\tCache lines mask: 0b"));
            Serial.println(NumberOfCacheLinesMask, BIN);
            Serial.print(F("\tCache Miss: target cache line: "));
            Serial.println(targetCacheLine, DEC);
            Serial.print(F("\tCache Miss: raw address: 0x"));
            Serial.println(targetAddress, HEX);
            Serial.print(F("\tCache Miss: aligned address: 0x"));
            Serial.println(alignedAddress, HEX);
        }
        // generate an aligned address
        replacementLine.reset(alignedAddress, thing_);
        return replacementLine;
    }
private:
    MemoryThing* thing_ = nullptr;
    ASingleCacheLine lines_[NumberOfCacheLines];
};
#endif //I960SXCHIPSET_CACHE_H
