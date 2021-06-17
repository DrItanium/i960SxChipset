//
// Created by jwscoggins on 6/16/21.
//

#ifndef I960SXCHIPSET_OPL2THING_H
#define I960SXCHIPSET_OPL2THING_H
#include "MemoryThing.h"
#include <OPL2.h>
class OPL2Thing : public IOSpaceThing {
public:
    explicit OPL2Thing(Address base) : IOSpaceThing(base, base + 0x100) { }

private:
    OPL2 theOPL2_;
};
#endif //I960SXCHIPSET_OPL2THING_H
