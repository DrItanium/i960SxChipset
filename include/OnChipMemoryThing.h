//
// Created by jwscoggins on 8/12/21.
//

#ifndef SXCHIPSET_ONCHIPMEMORYTHING_H
#define SXCHIPSET_ONCHIPMEMORYTHING_H
#include "MCUPlatform.h"
#include "MemoryThing.h"
class ReadOnlyOnChipMemoryThing : public MemoryThing {
public:
    using Parent = MemoryThing;
    ReadOnlyOnChipMemoryThing(Address baseAddress, Address length, const byte* pointer) : Parent(baseAddress, length), pointer_(pointer) { }
    ~ReadOnlyOnChipMemoryThing() override = default;
public:
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
    [[nodiscard]] bool bypassesCache() const noexcept override { return true; }
private:
    const byte* pointer_ = nullptr;
};


#endif //SXCHIPSET_ONCHIPMEMORYTHING_H
