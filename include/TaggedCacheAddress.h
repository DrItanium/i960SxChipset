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
template<byte tagBits, byte totalBits = 32, byte offsetBits = 4, bool useSpecificTypeSizes = true>
union TaggedAddress {
    static constexpr auto NumLowestBits = offsetBits;
    static constexpr auto NumTagBits = tagBits;
    static constexpr auto NumRestBits = totalBits - (NumTagBits + NumLowestBits);
    static constexpr auto MaximumAddressSize = totalBits;
    static_assert((NumLowestBits + NumTagBits + NumRestBits) == MaximumAddressSize, "Too many or too few bits for this given tagged address!");
    static_assert((MaximumAddressSize >= 26) && (MaximumAddressSize <= 32), "Addresses cannot be smaller than 26 bits!");
    using TagType = conditional_t<useSpecificTypeSizes, ClosestBitValue_t<NumTagBits>, Address>;
    using RestType = conditional_t<useSpecificTypeSizes, ClosestBitValue_t<NumRestBits>, Address>;
    using LowerType = conditional_t<useSpecificTypeSizes, ClosestBitValue_t<NumLowestBits>, Address>;
    using AddressType = conditional_t<useSpecificTypeSizes, ClosestBitValue_t<MaximumAddressSize>, Address>;
    constexpr explicit TaggedAddress(Address value = 0) noexcept : base(value) { }
    constexpr explicit TaggedAddress(RestType key, TagType tag, LowerType offset = 0) noexcept : lowest(offset), tagIndex(tag), rest(key) { }
    void clear() noexcept { base = 0; }
    [[nodiscard]] constexpr auto getTagIndex() const noexcept { return tagIndex; }
    [[nodiscard]] constexpr auto getAddress() const noexcept { return base; }
    [[nodiscard]] constexpr auto getLowest() const noexcept { return lowest; }
    [[nodiscard]] constexpr auto getOffset() const noexcept { return lowest; }
    [[nodiscard]] constexpr auto getRest() const noexcept { return rest; }
    [[nodiscard]] TaggedAddress aligned() const noexcept {
        TaggedAddress result(base);
        result.lowest = 0;
        return result;
    }
    [[nodiscard]] bool restEqual(TaggedAddress other) const noexcept { return getRest() == other.getRest(); }
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
