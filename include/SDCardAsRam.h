/*
i960SxChipset
Copyright (c) 2020-2022, Joshua Scoggins
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
// Created by jwscoggins on 11/22/21.
//

#ifndef SXCHIPSET_SDCARDASRAM_H
#define SXCHIPSET_SDCARDASRAM_H
#include "CoreChipsetFeatures.h"
#include "OpenFileHandle.h"
#include <SdFat.h>
extern SdFat SD;
class ConfigurationSpace;
template<typename T>
class SDCardAsRam {
public:
    using SDInterface = T;
    SDCardAsRam() = delete;
    ~SDCardAsRam() = delete;
    static void begin() noexcept {
        SDInterface::begin();
        if (!theRam_.open("ram.bin", FILE_WRITE)) {
            signalHaltState(F("COULD NOT OPEN RAM.BIN FOR READ/WRITE"));
        }
        theRam_.seekToBeginning();
    }
public:
    static size_t write(uint32_t address, byte *buf, size_t capacity) noexcept {
        theRam_.setAbsolutePosition(address);
        return theRam_.write(buf, capacity);
    }
    static size_t read(uint32_t address, byte *buf, size_t capacity) noexcept {
        theRam_.setAbsolutePosition(address);
        return theRam_.read(buf, capacity);
    }
private:
    static inline OpenFileHandle theRam_;
};
using FallbackMemory = SDCardAsRam<ConfigurationSpace>;

#endif //SXCHIPSET_SDCARDASRAM_H
