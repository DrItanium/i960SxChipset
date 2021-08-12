//
// Created by jwscoggins on 8/12/21.
//

#ifndef SXCHIPSET_ONCHIPMEMORYTHING_H
#define SXCHIPSET_ONCHIPMEMORYTHING_H
#include "MemoryThing.h"
class OnChipMemoryThing : public MemoryThing {
public:
    using Parent = MemoryThing;
    OnChipMemoryThing(Address baseAddress, Address length, byte* pointer) : Parent(baseAddress, length), pointer_(pointer) { }
private:
    byte* pointer_ = nullptr;

};

#endif //SXCHIPSET_ONCHIPMEMORYTHING_H
