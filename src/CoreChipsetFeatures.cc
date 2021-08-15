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
#include "CoreChipsetFeatures.h"

CoreChipsetFeatures::CoreChipsetFeatures(Address offsetFromIOBase) : IOSpaceThing(offsetFromIOBase, offsetFromIOBase + 0x100) { }


Address
CoreChipsetFeatures::makeAddressRelative(Address input) const noexcept {
    // we already know that this address is exactly one byte wide so just return the lowest byte as it's own thing
    return SplitWord32(input).bytes[0];
}
bool
CoreChipsetFeatures::respondsTo(Address address) const noexcept {
    SplitWord32 theAddr(address);
    SplitWord32 theBase(getBaseAddress());
    return (theAddr.bytes[3] == theBase.bytes[3]) && // okay we are in io space
           (theAddr.bytes[1] == theBase.bytes[1]);   // and the proper sub subsection of io space
}
uint16_t
CoreChipsetFeatures::read(Address address, LoadStoreStyle) noexcept {
    // force override the default implementation
    SplitWord32 addr(address);
    switch (static_cast<Registers>(addr.bytes[0])) {
        case Registers::ConsoleIO: return Serial.read();
        case Registers::ConsoleAvailable: return Serial.available();
        case Registers::ConsoleAvailableForWrite: return Serial.availableForWrite();
        default: return 0;
    }
}
void
CoreChipsetFeatures::write(Address address, uint16_t value, LoadStoreStyle) noexcept {
    SplitWord32 addr(address);
    switch (static_cast<Registers>(addr.bytes[0])) {
        case Registers::ConsoleFlush:
            Serial.flush();
            break;
        case Registers::ConsoleIO:
            Serial.write(static_cast<char>(value));
            break;
        default:
            break;
    }
}
