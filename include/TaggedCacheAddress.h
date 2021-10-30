//
// Created by jwscoggins on 9/19/21.
//

#ifndef SXCHIPSET_TAGGEDCACHEADDRESS_H
#define SXCHIPSET_TAGGEDCACHEADDRESS_H
#include "MCUPlatform.h"
template<byte tagIndexBits, typename T = byte>
union TaggedAddress {
    static constexpr auto NumLowestBits = 4;
    static constexpr auto NumTagBits = tagIndexBits;
    static constexpr auto NumRestBits = 32 - (NumTagBits + NumLowestBits);
    using TagType = T;
    constexpr explicit TaggedAddress(Address value = 0) noexcept : base(value) { }
    void clear() noexcept { base = 0; }
    [[nodiscard]] constexpr auto getTagIndex() const noexcept { return tagIndex; }
    [[nodiscard]] constexpr auto getAddress() const noexcept { return base; }
    [[nodiscard]] constexpr auto getLowest() const noexcept { return lowest; }
    [[nodiscard]] constexpr auto getRest() const noexcept { return rest; }
    [[nodiscard]] constexpr auto getPSRAMChipId() const noexcept { return offset; }
    [[nodiscard]] constexpr auto getPSRAMIndex() const noexcept { return psramIndex; }
    [[nodiscard]] constexpr auto getPSRAMAddress() const noexcept { return psramIndex; }
    [[nodiscard]] constexpr auto getPSRAMAddress_High() const noexcept { return bytes_[2]; }
    [[nodiscard]] constexpr auto getPSRAMAddress_Middle() const noexcept { return bytes_[1]; }
    [[nodiscard]] constexpr auto getPSRAMAddress_Low() const noexcept { return bytes_[0]; }
    [[nodiscard]] TaggedAddress aligned() const noexcept {
        TaggedAddress result(base);
        result.lowest = 0;
        return result;
    }
    [[nodiscard]] bool restEqual(TaggedAddress other) const noexcept { return getRest() == other.getRest(); }
private:
    Address base;
    struct {
        byte lowest : NumLowestBits;
        TagType tagIndex : NumTagBits;
        Address rest : NumRestBits;
    };
    struct {
        Address psramIndex : 23;
        byte offset : 3;
    };
    byte bytes_[4];
};
#endif //SXCHIPSET_TAGGEDCACHEADDRESS_H
