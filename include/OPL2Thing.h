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
    enum class Registers : uint16_t {
        NumberOfChannels = 0,
        ResetDevice,
        // features, perhaps one byte?
        Percussion,
        NoteSelect,
        WaveFormSelect,
        DeepTremolo,
        DeepVibrato,
        Drums,
        Channel0 = 0x100,
        ChannelRegister,
        Attack,
        Block,
        Volume,
        Decay,
        EnvelopeScaling,
        FNumber,
        Channel1 = 0x200,
        Channel2 = 0x300,
        Channel3 = 0x400,
        Channel4 = 0x500,
        Channel5 = 0x600,
        Channel6 = 0x700,
        Channel7 = 0x800,
        Channel8 = 0x900,
    };
public:
    explicit OPL2Thing(Address base) : IOSpaceThing(base, base + 0x1000) ,
                                       theOPL2_(static_cast<byte>(resetPin),
                                                static_cast<byte>(addressPin),
                                                static_cast<byte>(latchPin)) { }

    ~OPL2Thing() override = default;
    void begin() noexcept override {
        theOPL2_.begin();
    }
private:
    OPL2 theOPL2_;
    float frequencyCache_[OPL2_NUM_CHANNELS] = { 0 };
};
#endif //I960SXCHIPSET_OPL2THING_H
