//
// Created by jwscoggins on 8/12/21.
//

#ifndef SXCHIPSET_ONCHIPMEMORYTHING_H
#define SXCHIPSET_ONCHIPMEMORYTHING_H
#include "MCUPlatform.h"
#include "MemoryThing.h"
class OnChipMemoryThing : public MemoryThing {
public:
    using Parent = MemoryThing;
    OnChipMemoryThing(Address baseAddress, Address length, byte* pointer) : Parent(baseAddress, length), pointer_(pointer) { }
    ~OnChipMemoryThing() override = default;
public:
    size_t blockWrite(Address address, uint8_t *buf, size_t capacity) noexcept override {
        for (size_t i = 0; i < capacity; ++i, ++address) {
            write8(address, buf[i]);
        }
        return capacity;
    }
    size_t blockRead(Address address, uint8_t *buf, size_t capacity) noexcept override {
        for (size_t i = 0; i < capacity; ++i, ++address) {
            buf[i] = read8(address);
        }
        return capacity;
    }
    uint8_t read8(Address address) noexcept override {
        return pointer_[address];
    }
    uint16_t read16(Address address) noexcept override {
        return SplitWord16(pointer_[address], pointer_[address+1]).getWholeValue();
    }
    void write8(Address address, uint8_t value) noexcept override {
        pointer_[address] = value;
    }
    void write16(Address address, uint16_t value) noexcept override {
        SplitWord16 decomp(value);
        pointer_[address] = decomp.bytes[0];
        pointer_[address+1] = decomp.bytes[1];
    }
    [[nodiscard]] bool bypassesCache() const noexcept override { return true; }
private:
    byte* pointer_ = nullptr;
};

class ReadOnlyOnChipMemoryThing : public OnChipMemoryThing {
public:
    using Parent = OnChipMemoryThing;
    ReadOnlyOnChipMemoryThing(Address base, Address length, const byte* pointer) : Parent(base, length, const_cast<byte*>(pointer)) { }
    ~ReadOnlyOnChipMemoryThing() override = default;
    size_t blockWrite(Address address, uint8_t *buf, size_t capacity) noexcept override {
        return 0;
    }
    void write8(Address address, uint8_t value) noexcept override {
        // do nothing
    }
    void write16(Address address, uint16_t value) noexcept override {
        // do nothing
    }
};

#endif //SXCHIPSET_ONCHIPMEMORYTHING_H
