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
// Created by jwscoggins on 10/31/21.
//

#ifndef SXCHIPSET_I960SXCHIPSET_H
#define SXCHIPSET_I960SXCHIPSET_H

using BodyFunction = void (*)();
using DispatchTable = BodyFunction[256];
extern DispatchTable lookupTableRead;
extern DispatchTable lookupTableWrite;
inline BodyFunction getNonDebugReadBody(byte index) noexcept { return lookupTableRead[index]; }
inline BodyFunction getNonDebugWriteBody(byte index) noexcept { return lookupTableWrite[index]; }
BodyFunction getDebugReadBody(byte index) noexcept;
BodyFunction getDebugWriteBody(byte index) noexcept;

template<bool inDebugMode>
BodyFunction getReadBody(byte index) noexcept {
    if constexpr (inDebugMode) {
        return getDebugReadBody(index);
    } else {
        return getNonDebugReadBody(index);
    }
}
template<bool inDebugMode>
BodyFunction getWriteBody(byte index) noexcept {
    if constexpr (inDebugMode) {
        return getDebugWriteBody(index);
    } else {
        return getNonDebugWriteBody(index);
    }
}

[[noreturn]] void signalHaltState(const __FlashStringHelper* msg);
#endif //SXCHIPSET_I960SXCHIPSET_H
