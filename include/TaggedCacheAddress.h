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
// Created by jwscoggins on 9/19/21.
//

#ifndef SXCHIPSET_TAGGEDCACHEADDRESS_H
#define SXCHIPSET_TAGGEDCACHEADDRESS_H
#include "MCUPlatform.h"
/**
 * @brief A wrapper around a 32-bit i960Sx address broken up into four components [unused, key, tag, offset] in order from most significant bit to least.
 * The unused portion is the most significant bits of the value which are beyond the totalBits template parameter (it cannot be accessed and is only logically a part of it).
 * The key is used to uniquely identify an cache line, it is primarily used when checking for cache line matches.
 * The tag is the array index where a given cache line will be stored.
 * The offset represents the starting position in the cache line for the given request.
 * @tparam tagBits The number of tag bits
 * @tparam totalBits The total number of bits that are used of the 32-bit number (default is 32)
 * @tparam offsetBits The number of bits allocated to the cache line itself (default is 4)
 * @tparam useSpecificTypeSizes When true use the smallest type that can hold onto each of the three important components. When false, use uint32_t for all types. On ARM processors, set this to false otherwise errors happen
 */
template<byte tagBits, byte totalBits = 32, byte offsetBits = 4, bool useSpecificTypeSizes = true>
union TaggedAddress {
    /**
     * @brief The number of bits allocated to the offset component
     */
    static constexpr auto NumLowestBits = offsetBits;
    /**
     * @brief The number of bits allocated to the tag component
     */
    static constexpr auto NumTagBits = tagBits;
    /**
     * @brief The number of bits allocated to the key component
     */
    static constexpr auto NumRestBits = totalBits - (NumTagBits + NumLowestBits);
    /**
     * @brief The number of bits of a 32-bit address used to compute cache addresses
     */
    static constexpr auto MaximumAddressSize = totalBits;
    static_assert((NumLowestBits + NumTagBits + NumRestBits) == MaximumAddressSize, "Too many or too few bits for this given tagged address!");
    /**
     * @brief The data type for the tag component
     */
    using TagType = conditional_t<useSpecificTypeSizes, ClosestBitValue_t<NumTagBits>, Address>;
    /**
     * @brief The data type for the key component
     */
    using RestType = conditional_t<useSpecificTypeSizes, ClosestBitValue_t<NumRestBits>, Address>;
    /**
     * @brief The data type for the offset component
     */
    using LowerType = conditional_t<useSpecificTypeSizes, ClosestBitValue_t<NumLowestBits>, Address>;
    /**
     * @brief Construct an address from a full 32-bit address
     * @param value The full 32-bit address to decompose
     */
    constexpr explicit TaggedAddress(Address value = 0) noexcept : base(value) { }
    /**
     * @brief Construct an address from the three important components
     */
    constexpr explicit TaggedAddress(RestType key, TagType tag, LowerType offset = 0) noexcept : lowest(offset), tagIndex(tag), rest(key) { }
    /**
     * @brief Set the core 32-bit value to 0
     */
    void clear() noexcept { base = 0; }
    /**
     * @brief retrieve the complete 32-bit address
     * @return The underlying full 32-bit address
     */
    [[nodiscard]] constexpr auto getAddress() const noexcept { return base; }
    /**
     * @brief Get the tag component of this address
     * @return The tag component of this address as a standalone value
     */
    [[nodiscard]] constexpr auto getTagIndex() const noexcept { return tagIndex; }
    /**
     * @brief Get the offset component of this address
     * @return The offset component of this address as a standalone value
     */
    [[nodiscard]] constexpr auto getOffset() const noexcept { return lowest; }
    /**
     * @brief Get the key component of this address
     * @return The key component of this address as a standalone value
     */
    [[nodiscard]] constexpr auto getRest() const noexcept { return rest; }
    /**
     * @brief Construct a copy of this address with the offset set to zero
     * @return A copy of address with the offset field set to zero
     */
    [[nodiscard]] TaggedAddress aligned() const noexcept {
        return TaggedAddress{rest, tagIndex};
    }
private:
    Address base;
    struct {
        LowerType lowest : NumLowestBits;
        TagType tagIndex : NumTagBits;
        RestType rest : NumRestBits;
    };
    byte bytes_[4];
};
#endif //SXCHIPSET_TAGGEDCACHEADDRESS_H
