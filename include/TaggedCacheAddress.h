//
// Created by jwscoggins on 9/19/21.
//

#ifndef SXCHIPSET_TAGGEDCACHEADDRESS_H
#define SXCHIPSET_TAGGEDCACHEADDRESS_H
#include "MCUPlatform.h"
union TaggedAddress {
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
        byte lowest : 4;
        byte tagIndex : 8;
        Address rest : 20;
    };
    struct {
        Address psramIndex : 24;
        byte offset : 2; // 64 megs max
    };
    byte bytes_[4];
};
#endif //SXCHIPSET_TAGGEDCACHEADDRESS_H
