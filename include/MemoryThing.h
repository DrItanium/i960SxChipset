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

#ifndef I960SXCHIPSET_MEMORYTHING_H
#define I960SXCHIPSET_MEMORYTHING_H
#include "Pinout.h"
/**
 * @brief Describes the interface between a component and a memory request, it introduces some latency with the trade off being easier maintenance
 */
class MemoryThing {
public:
    MemoryThing(Address baseAddress, Address endAddress) : base_(baseAddress), end_(endAddress) { }
    /**
     * @brief Construct a memory thing that is only concerned with a single address
     * @param baseAddress
     */
    explicit MemoryThing(Address baseAddress) : MemoryThing(baseAddress, baseAddress + 1) { }
    virtual ~MemoryThing() = default;
    [[nodiscard]] virtual uint8_t read8(Address address) noexcept { return 0; }
    [[nodiscard]] virtual uint16_t read16(Address address) noexcept { return 0; }
    [[nodiscard]] virtual bool respondsTo(Address address) const noexcept { return base_ <= address && address < end_; }
    [[nodiscard]] bool respondsTo(Address address, LoadStoreStyle style) const noexcept {
        switch (style) {
            case LoadStoreStyle::Upper8:
                // we need to see if it is legal to write to the upper 8 bits by itself
                return respondsTo(address + 1);
            case LoadStoreStyle::Lower8:
            case LoadStoreStyle::Full16:
                // while a hack, if the default check passes then it is safe to write 8 or 16 bits!
                // this may need to expanded out later to disallow 16-bit writes and lock to 8-bit writes only, etc, etc
                return respondsTo(address);
            default:
                return false;

        }
    }
    virtual void write8(Address address, uint8_t value) noexcept { }
    virtual void write16(Address address, uint16_t value) noexcept { }
    [[nodiscard]] constexpr auto getBaseAddress() const noexcept { return base_; }
    [[nodiscard]] constexpr auto getEndAddress() const noexcept { return end_; }
    [[nodiscard]] virtual Address makeAddressRelative(Address input) const noexcept { return input - base_; }
    [[nodiscard]] uint16_t read(Address address, LoadStoreStyle style) noexcept {
#if 0
        Serial.print(F("READ from 0x"));
        Serial.println(address, HEX);
#endif
        auto offset = makeAddressRelative(address);
        switch (style) {
            case LoadStoreStyle::Full16:
                return read16(offset);
            case LoadStoreStyle::Lower8:
                return read8(offset);
            case LoadStoreStyle::Upper8:
                return read8(offset + 1);
            default:
                return 0;
        }
    }
    void write(Address address, uint16_t value, LoadStoreStyle style) noexcept {
#if 0
        Serial.print(F("WRITE 0x"));
        Serial.print(value, HEX);
        Serial.print(F("to 0x"));
        Serial.println(address, HEX);
#endif
        auto offset = makeAddressRelative(address);
        switch (style) {
            case LoadStoreStyle::Full16:
                write16(offset, value);
                break;
            case LoadStoreStyle::Upper8:
                write8(offset+1, (value >> 8) );
                break;
            case LoadStoreStyle::Lower8:
                write8(offset, value);
                break;
            default:
                break;
        }
    }
    /**
     * @brief Called during startup to activate any extra functionality
     */
    virtual void begin() noexcept { }
    /**
     * @brief Write a buffer to the given thing mapped into memory, this is used by the DataCache to perform cache replacement
     * @param baseAddress the starting address where data will be written to within the memory thing
     * @param buffer the buffer to write to memory
     * @param size the number of elements to write
     */
    virtual void write(uint32_t baseAddress, byte* buffer, size_t size) { }
    /**
     * @brief Read into a provided buffer from the mapped memory thing, this is used by the DataCache to perform cache replacement
     * @param baseAddress the starting address where data will be read from
     * @param buffer the buffer to store the read operation into
     * @param size the number of bytes to read from the memory thing into the buffer
     */
    virtual void read(uint32_t baseAddress, byte* buffer, size_t size) { }
private:
    Address base_;
    Address end_;
};

class LambdaMemoryThing : public MemoryThing {
public:
    using Read8Operation = uint8_t(*)(Address);
    using Read16Operation = uint16_t(*)(Address);
    using Write8Operation = void(*)(Address, uint8_t);
    using Write16Operation = void(*)(Address, uint16_t);
public:
    LambdaMemoryThing(Address base, Address end, Read8Operation r8, Write8Operation w8, Read16Operation r16, Write16Operation w16) noexcept : MemoryThing(base, end), read8_(r8), read16_(r16), write8_(w8), write16_(w16) { }
    LambdaMemoryThing(Address base, Address end, Read8Operation r8, Write8Operation w8) noexcept : LambdaMemoryThing(base, end, r8, w8, nullptr, nullptr) { }
    LambdaMemoryThing(Address base, Address end, Read16Operation r16, Write16Operation w16) noexcept : LambdaMemoryThing(base, end, nullptr, nullptr, r16, w16) { }
    ~LambdaMemoryThing() override = default;
    uint8_t
    read8(Address address) noexcept override {
            if (read8_) {
                return read8_(address);
            }
            return 0;
    }
    uint16_t
    read16(Address address) noexcept override {
            if (read16_) {
                return read16_(address);
            }
            return 0;
    }
    void
    write8(Address address, uint8_t value) noexcept override {
            if (write8_) {
                write8_(address, value);
            }
    }
    void
    write16(Address address, uint16_t value) noexcept override {
            if (write16_) {
                write16_(address, value);
            }
    }
private:
    Read8Operation read8_;
    Read16Operation read16_;
    Write8Operation write8_;
    Write16Operation write16_;

};

/**
 * @brief An intermediate type which automatically adds the IOBaseAddress to the start and end addresses
 */
class IOSpaceThing : public MemoryThing {
public:
    static constexpr Address SpaceBaseAddress = 0xFE00'0000;
public:
    IOSpaceThing(Address base, Address end) : MemoryThing(base + SpaceBaseAddress, end + SpaceBaseAddress) { }
    explicit IOSpaceThing(Address base) : MemoryThing(base + SpaceBaseAddress) { }
    ~IOSpaceThing() override = default;
};

#endif //I960SXCHIPSET_MEMORYTHING_H
