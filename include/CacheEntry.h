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
// Created by jwscoggins on 11/19/21.
//

#ifndef SXCHIPSET_CACHEENTRY_H
#define SXCHIPSET_CACHEENTRY_H
#include "TaggedCacheAddress.h"
#include "i960SxChipset.h"
/**
 * @brief Describes a single cache line which holds onto a customizable number of words which are some multiple of 8 bit bytes
 * @tparam numTagBits The number of bits in a given address devoted to the index of the cache line
 * @tparam maxAddressBits How many bits the entire address is comprised of (maximum of 32)
 * @tparam numLowestBits How many bits (and through some may bytes) describe the offset into this cache line (this determines how many bytes are stored in this line)
 * @tparam T The type of the backing memory storage (SDCard, PSRAM, etc)
 * @tparam useSpecificTypeSizes When true, use the smallest type for each field in the address type used by this cache line. When false, use uint32_t. Should only be false on ARM platforms
 */
template<byte numTagBits, byte maxAddressBits, byte numLowestBits, typename T, bool useSpecificTypeSizes = true>
class CacheEntry final {
public:
    static_assert(numTagBits > 0, "Number of tag bits is zero");
    static_assert(numLowestBits != 0, "Number of of bytes per cache line is zero");
    /**
     * @brief Divides the bytes that make up this cache line to this type
     */
    using Word = SplitWord16;
    /**
     * @brief The number of bytes cached by this line
     */
    static constexpr size_t NumBytesCached = pow2(numLowestBits);
    /**
     * @brief The number of words cached by this line (NumBytesCached / sizeof(Word))
     */
    static constexpr size_t NumWordsCached = NumBytesCached / sizeof(Word);

    static_assert(NumBytesCached != 0, "the number of bytes to be cached is a non power of 2");
    /**
     * @brief A bitmask of the word types
     */
    static constexpr byte CacheEntryMask = NumWordsCached - 1;
    /**
     * @brief The number of positions to shift the offset component by when computing the entry offset
     */
    static constexpr byte CacheEntryShiftAmount = numberOfBitsForCount(sizeof(Word));
    static_assert(CacheEntryShiftAmount != 0, "Defined word type must be a power of 2");
    static_assert(CacheEntryShiftAmount < numLowestBits, "The words that make up a cache line need to be smaller than the entire storage in the cache line itself");
    /**
     * @brief The address type used to describe addresses passed to this line
     */
    using TaggedAddress = ::TaggedAddress<numTagBits, maxAddressBits, numLowestBits, useSpecificTypeSizes>;
    /**
     * @brief The type of the offset component of the line address (the least significant bits of the address)
     */
    using OffsetType = typename TaggedAddress::LowerType ;
    /**
     * @brief The type of the key component of the line address (the most significant bits of the address)
     */
    using KeyType = typename TaggedAddress::RestType;
public:
    /**
     * @brief Clear the contents of the line out (committing the contents to backing store if dirty) and then populate the line with new data from the backing store.
     * @param newTag The address that represents the base of the new line (offset will be zero)
     */
    void reset(TaggedAddress newTag) noexcept {
        // no match so pull the data in from main memory
        if (isValid() && isDirty()) {
            // we compute the overall range as we go through this stuff
            (void)T::write(TaggedAddress{key_, newTag.getTagIndex()}.getAddress(),
                           reinterpret_cast<byte*>(data),
                           NumBytesCached);
        }
        dirty_ = false;
        // since we have called reset, now align the new address internally
        key_ = newTag.getRest();
        // this is a _very_ expensive operation
        (void)T::read(newTag.aligned().getAddress(), reinterpret_cast<byte*>(data), NumBytesCached);
    }
    /**
     * @brief Clear the entry without saving what was previously in it, necessary if the memory was reused for a different purpose
     */
    void clear() noexcept {
        // clear all flags
        bits = 0;
        key_ = 0;
        for (auto& a : data) {
            a.wholeValue_ = 0;
        }
    }
    /**
     * @brief Regardless of line validity, check to see if the address keys match.
     * @param addr The address to pull the keys out
     * @return True if the keys match
     */
    [[nodiscard]] constexpr bool addressesMatch(TaggedAddress addr) const noexcept { return (addr.getRest() == key_); }
    /**
     * @brief Given a valid line, compare the cache keys to see if they are equal
     * @param addr The address to compare with the key stored inside the line
     * @return True if line is valid and the keys match
     */
    [[nodiscard]] constexpr bool matches(TaggedAddress addr) const noexcept { return isValid() && addressesMatch(addr); }
    /**
     * @brief Return the word at a given offset; NOTE THERE IS NO CHECKS TO MAKE SURE YOU DIDN'T GO OUT OF BOUNDS!
     * @param offset The offset into the line in word form (it is not a byte offset)
     * @return The word itself at the given position (a copy!)
     */
    [[nodiscard]] constexpr auto get(OffsetType offset) const noexcept { return data[offset].getWholeValue(); }
    /**
     * @brief Return two words smashed into a double word; NOTE THERE IS NO CHECKS TO MAKE SURE YOU DIDN'T GO OUT OF BOUNDS!!!
     * @param offset The offset into the line for the lower word (not a byte offset)
     * @return The two words returned in a little endian format
     */
    [[nodiscard]] constexpr auto get2(OffsetType offset) const noexcept { return SplitWord32{get(offset), get(offset+1)}; }
    /**
     * @brief Update a given word in the line and check it to see if the dirty bits should be updated as well
     * @param offset The word offset into the line
     * @param style Is this a Full 16-bit update, lower8 update, or upper8 update? Chipset halt on anything else
     * @param value The new value to update the target word in the line with
     */
    inline void set(OffsetType offset, LoadStoreStyle style, Word value) noexcept {
        /// @todo add support for Words which are larger than bus width
        // while unsafe, assume it is correct because we only get this from the ProcessorSerializer, perhaps directly grab it?
        switch (auto& target = data[offset]; style) {
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
        dirty_ = true;
    }
    /**
     * @brief Check to see if the given line is valid
     * @return True if the line is not invalid
     */
    [[nodiscard]] constexpr bool isValid() const noexcept { return valid_; }
    /**
     * @brief Check to see if any words in the line have been modified in a meaningful way
     * @return True if any words in the line have been modified
     */
    [[nodiscard]] constexpr bool isDirty() const noexcept { return dirty_; }
    /**
     * @brief Return a pointer to the underlying data to be used when performing read operations
     * @return The based pointer to the underlying memory contents
     */
    [[nodiscard]] const Word * getRawData() const noexcept { return data; }
private:
    Word data[NumWordsCached]; // 16 bytes
    KeyType key_ = 0;
    union {
        uint8_t bits = 0;
        struct {
            uint8_t valid_ : 1;
            uint8_t dirty_ : 1;
        };
    };
};
#endif //SXCHIPSET_CACHEENTRY_H
