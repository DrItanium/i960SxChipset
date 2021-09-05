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
// Created by jwscoggins on 6/21/21.
//

#ifndef I960SXCHIPSET_CORECHIPSETFEATURES_H
#define I960SXCHIPSET_CORECHIPSETFEATURES_H
#include "MemoryThing.h"
#include "ProcessorSerializer.h"
class CoreChipsetFeatures /* : public IOSpaceThing */ {
public:
    enum class Registers : uint8_t {
#define TwoByteEntry(Prefix) Prefix ## 0, Prefix ## 1
#define FourByteEntry(Prefix) \
        TwoByteEntry(Prefix ## 0), \
        TwoByteEntry(Prefix ## 1)
#define EightByteEntry(Prefix) \
        FourByteEntry(Prefix ## 0), \
        FourByteEntry(Prefix ## 1)
#define TwelveByteEntry(Prefix) \
        EightByteEntry(Prefix ## 0), \
        FourByteEntry(Prefix ## 1)
#define SixteenByteEntry(Prefix) \
        EightByteEntry(Prefix ## 0), \
        EightByteEntry(Prefix ## 1)
        TwoByteEntry(ConsoleFlush),
        TwoByteEntry(ConsoleAvailable),
        TwoByteEntry(ConsoleAvailableForWrite),
        TwoByteEntry(ConsoleIO),
#undef SixteenByteEntry
#undef TwelveByteEntry
#undef EightByteEntry
#undef FourByteEntry
#undef TwoByteEntry
        End,
        ConsoleFlush = ConsoleFlush0,
        ConsoleAvailable = ConsoleAvailable0,
        ConsoleAvailableForWrite = ConsoleAvailableForWrite0,
        ConsoleIO = ConsoleIO0,
    };
    static_assert(static_cast<int>(Registers::End) < 0x100);
public:
    CoreChipsetFeatures() = delete;
    ~CoreChipsetFeatures() = delete;
    static void begin() noexcept { }
    static uint16_t read() noexcept {
        // force override the default implementation
        switch (static_cast<Registers>(ProcessorInterface::getLeastSignificantAddressByte())) {
            case Registers::ConsoleIO: return Serial.read();
            case Registers::ConsoleAvailable: return Serial.available();
            case Registers::ConsoleAvailableForWrite: return Serial.availableForWrite();
            default: return 0;
        }
    }
    static void write() noexcept {
        switch (static_cast<Registers>(ProcessorInterface::getLeastSignificantAddressByte())) {
            case Registers::ConsoleFlush:
                Serial.flush();
                break;
            case Registers::ConsoleIO:
                Serial.write(static_cast<char>(ProcessorInterface::getDataBits()));
                break;
            default:
                break;
        }
    }
    static constexpr bool respondsTo(byte index) noexcept {
        return index == 0xFE;
    }
};
#endif //I960SXCHIPSET_CORECHIPSETFEATURES_H
