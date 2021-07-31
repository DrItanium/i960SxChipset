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
[[noreturn]] void signalHaltState(const __FlashStringHelper* msg);
/**
 * @brief Describes the interface between a component and a memory request, it introduces some latency with the trade off being easier maintenance
 */
class MemoryThing {
public:
    MemoryThing(Address baseAddress, Address endAddress) : base_(baseAddress), end_(endAddress) {
        if (endAddress < baseAddress) {
            signalHaltState(F("End address comes before base address"));
        }
        if ((endAddress - baseAddress) < 16) {
            signalHaltState(F("Minimum mapped size of a memory thing must be 16-bytes"));
        }
        if ((baseAddress & 0b1111) != 0) {
            signalHaltState(F("memory things must be aligned to 16 byte boundaries"));
        }
    }
    /**
     * @brief Construct a memory thing that is only concerned with a single address
     * @param baseAddress
     */
    explicit MemoryThing(Address baseAddress) : MemoryThing(baseAddress, baseAddress + 16) { }
    virtual ~MemoryThing() = default;
    virtual size_t blockWrite(Address address, uint8_t* buf, size_t capacity) noexcept { return 0; }
    virtual size_t blockRead(Address address, uint8_t* buf, size_t capacity) noexcept { return 0; }
    [[nodiscard]] virtual uint8_t read8(Address address) noexcept { return 0; }
    [[nodiscard]] virtual uint16_t read16(Address address) noexcept { return 0; }
    virtual void write8(Address address, uint8_t value) noexcept { }
    virtual void write16(Address address, uint16_t value) noexcept { }
    [[nodiscard]] virtual bool respondsTo(Address address) const noexcept { return base_ <= address && address < end_; }
    /**
     * @brief Get the number of bytes from the target address to the end of section
     * @param address the base address to compute the length from
     * @return End address of memory thing minus address
     */
    [[nodiscard]] virtual Address lengthFollowingTargetAddress(Address address) const noexcept {
        if (address >= end_) {
            return 0;
        } else {
            return end_ - address;
        }
    }
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
    [[nodiscard]] constexpr auto getBaseAddress() const noexcept { return base_; }
    [[nodiscard]] constexpr auto getEndAddress() const noexcept { return end_; }
    [[nodiscard]] virtual Address makeAddressRelative(Address input) const noexcept { return input - base_; }
    [[nodiscard]] virtual uint16_t read(Address address, LoadStoreStyle style) noexcept {
        SplitWord16 outcome;
        outcome.wholeValue_ = 0;
        switch (auto offset = makeAddressRelative(address); style) {
            case LoadStoreStyle::Full16:
                outcome.wholeValue_ = read16(offset);
                break;
            case LoadStoreStyle::Lower8:
                outcome.bytes[0] = read8(offset);
                break;
            case LoadStoreStyle::Upper8:
                outcome.bytes[1] = read8(offset + 1);
                break;
            default:
                break;
        }
        return outcome.wholeValue_;
    }
    virtual void write(Address address, uint16_t value, LoadStoreStyle style) noexcept {
        SplitWord16 input;
        input.wholeValue_ = value;
        switch (auto offset = makeAddressRelative(address); style) {
            case LoadStoreStyle::Full16:
                write16(offset, input.wholeValue_);
                break;
            case LoadStoreStyle::Upper8:
                write8(offset + 1, input.bytes[1]);
                break;
            case LoadStoreStyle::Lower8:
                write8(offset, input.bytes[0]);
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
     * @brief Perform a transfer of a block of bytes
     * @param baseAddress the base address according to the i960
     * @param buffer the buffer to commit
     * @param size the number of bytes to transfer out of the buffer
     * @return The number of bytes written
     */
    virtual size_t write(uint32_t baseAddress, byte* buffer, size_t size) noexcept {
        auto relativeAddress = makeAddressRelative(baseAddress);
        // compute how many bytes we can actually write within this memory thing in case the buffer spans multiple devices
        size_t count = size;
        if (auto actualLength = lengthFollowingTargetAddress(baseAddress); size > actualLength) {
            count = actualLength;
        }
        return blockWrite(relativeAddress, buffer, count);
    }
    /**
     * @brief Transfer a block of bytes into a provides buffer from the memory thing
     * @param baseAddress The address to start at
     * @param buffer  the buffer to store into
     * @param size the number of bytes to transfer
     * @return the number of bytes actually read into the buffer
     */
    virtual size_t read(uint32_t baseAddress, byte* buffer, size_t size) noexcept {
        auto relativeAddress = makeAddressRelative(baseAddress);
        // compute how many bytes we can actually write within this memory thing in case the buffer spans multiple devices
        size_t count = size;
        if (auto actualLength = lengthFollowingTargetAddress(baseAddress); size > actualLength) {
            count = actualLength;
        }
        return blockRead(relativeAddress, buffer, count);
    }
    /**
     * @brief Turn off any sort of caching that this memory thing has
     */
    virtual void disableCache() noexcept { }
    /**
     * @brief Reactivate any sort of caching this memory thing has
     */
    virtual void enableCache() noexcept { }

    [[nodiscard]] virtual bool bypassesCache() const noexcept { return false; }
    virtual void signalHaltState(const __FlashStringHelper* thing) noexcept { ::signalHaltState(thing); }
private:
    Address base_;
    Address end_;
};

/**
 * @brief Turn cache enable and disable actions into RAII
 */
struct TemporarilyDisableThingCache final {
public:
    explicit TemporarilyDisableThingCache(MemoryThing* theThing) : thing_(theThing) {
        if (thing_) {
            thing_->disableCache();
        }
    }
    ~TemporarilyDisableThingCache() {
        if (thing_) {
            thing_->enableCache();
        }
    }
private:
    MemoryThing* thing_;
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
    [[nodiscard]] bool bypassesCache() const noexcept override { return true; }
};
/**
 * @brief Does a lookup in the global thing collection to try and find a thing that will respond to the given address
 * @param address the address to check for a response to
 * @param style the width of the request
 * @return The thing that will respond to the given address
 */
MemoryThing* getThing(Address address, LoadStoreStyle style) noexcept;

#endif //I960SXCHIPSET_MEMORYTHING_H
