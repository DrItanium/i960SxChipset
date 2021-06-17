//
// Created by jwscoggins on 6/16/21.
//

#ifndef I960SXCHIPSET_OPL2THING_H
#define I960SXCHIPSET_OPL2THING_H
#include <Arduino.h>
#include "MemoryThing.h"
#include <OPL2.h>
#include <Pinout.h>

template<i960Pinout resetPin, i960Pinout addressPin, i960Pinout latchPin>
class OPL2Thing : public IOSpaceThing {
public:
    explicit OPL2Thing(Address base) : IOSpaceThing(base, base + 0x100) ,
                                       theOPL2_(static_cast<byte>(resetPin),
                                                static_cast<byte>(addressPin),
                                                static_cast<byte>(latchPin)) { }

    ~OPL2Thing() override = default;
    void begin() noexcept override {
        theOPL2_.begin();
    }
private:
    OPL2 theOPL2_;
};
#endif //I960SXCHIPSET_OPL2THING_H
