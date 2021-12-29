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
    using CacheBackingStore = SRAM_23LC1024Chip<i960Pinout::CACHE_EN>;
    static constexpr auto Capacity = CacheBackingStore::Capacity;
    static constexpr auto CacheLineSize = 1024; // bytes
    static constexpr auto CacheLineBits = getNumberOfBitsForNumberOfEntries(CacheLineSize);
    static constexpr auto NumLines = Capacity / CacheLineSize;
    static constexpr auto NumLineBits = getNumberOfBitsForNumberOfEntries(NumLines);
    using CacheAddress = TaggedAddress<NumLineBits, 32, CacheLineBits>;
    // list our assumptions
    static_assert(CacheAddress::NumLowestBits == 10, "This class is written assuming a 1024 byte segment");
    static_assert(CacheAddress::NumTagBits == 7, "This class is written assuming a 1024 byte segment");
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
        uint16_t key_ = 0;
        // the key will be used to reconstruct the sram and backing storage addresses
        [[nodiscard]] constexpr bool dirty() const noexcept { return status_ == Status::Dirty; }
        [[nodiscard]] constexpr bool clean() const noexcept { return status_ == Status::Clean; }
        [[nodiscard]] constexpr bool invalid() const noexcept { return status_ == Status::Invalid; }
        [[nodiscard]] constexpr CacheAddress reconstructAddress(typename CacheAddress::TagType tagIndex) noexcept {
            return CacheAddress{ key_, tagIndex };
        }
        void clear() noexcept {
            status_ = Status::Invalid;
            key_ = 0;
        }
    };
public:
    static size_t write(uint32_t address, byte *buf, size_t capacity) noexcept {
        CacheAddress currAddr(address);
        return 0;
    }
    static size_t read(uint32_t address, byte *buf, size_t capacity) noexcept {
        CacheAddress currAddr(address);
        return 0;
    }
    static void clear() noexcept {
        for (auto& a : tagData_) {
            a.clear();
        }
    }
    static void begin() noexcept {
        clear();
    }
private:
    static inline Tag tagData_[NumLines];
    static inline byte transferBuffer_[CacheLineSize];
};

#endif //SXCHIPSET_SRAMDATACONTAINER_H
