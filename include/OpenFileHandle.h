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
// Created by jwscoggins on 10/3/21.
//

#ifndef SXCHIPSET_OPENFILEHANDLE_H
#define SXCHIPSET_OPENFILEHANDLE_H
#include "Pinout.h"
#include <SdFat.h>
extern SdFat SD;

/**
 * @brief A wrapper class around an sdfat file that exposes further functionality for the chipset to expose to the i960 via memory mapping
 */
class OpenFileHandle {
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
#undef SixteenByteEntry
#undef TwelveByteEntry
#undef EightByteEntry
#undef FourByteEntry
#undef TwoByteEntry
        End,
    };
public:
    bool open(const char* path, SplitWord32 permissions) noexcept {
        if (!backingStore_) {
            backingStore_ = SD.open(path, permissions.bytes[0]);
            return backingStore_;
        } else {
            return false;
        }
    }
    bool close() noexcept {
        if (backingStore_) {
            return backingStore_.close();
        } else {
            return false;
        }
    }

    [[nodiscard]] bool isOpen() const noexcept { return backingStore_.isOpen(); }
    explicit operator bool() const noexcept { return backingStore_.operator bool(); }
    [[nodiscard]] uint16_t read(uint8_t targetPage, uint8_t offset, LoadStoreStyle lss) noexcept {
        return 0;
    }
    void write(uint8_t offset, LoadStoreStyle lss, SplitWord16 value) noexcept {
    }
private:
    File backingStore_;
};

#endif //SXCHIPSET_OPENFILEHANDLE_H
