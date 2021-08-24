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
// Created by jwscoggins on 8/24/21.
//

#ifndef SXCHIPSET_ROMSECTIONS_H
#define SXCHIPSET_ROMSECTIONS_H
#include "Pinout.h"
#include "MemoryMappedFileThing.h"

class ROMTextSection : public MemoryMappedFile {
public:
    static constexpr Address Size = 512_MB;
    static constexpr Address Mask = Size - 1;
    using Parent = MemoryMappedFile;
public:
    explicit ROMTextSection(Address base) noexcept : Parent(base, base + Size, Size, "boot.rom", FILE_READ){ }
    ~ROMTextSection() override = default;
    Address makeAddressRelative(Address input) const noexcept override {
            SplitWord32 newAddr(input);
            newAddr.bytes[3] &= 0b00011111; // We just need to factor out the offset
            return newAddr.wholeValue_;
    }
    bool
    respondsTo(Address address) const noexcept override {
            // we just need to make sure that the upper most byte has the following pattern set
            // if the upper most byte is less than 0b0010'0000 then we have a match
            return SplitWord32(address).bytes[3] < 0b0010'0000;
    }
};

/// @todo add support for the boot data section that needs to be copied into ram by the i960 on bootup
class ROMDataSection : public MemoryMappedFile {
public:
    // two clusters are held onto at a time
    static constexpr Address Size = 512_MB;
    static constexpr auto Mask = Size - 1;
    using Parent = MemoryMappedFile;
public:
    explicit ROMDataSection(Address base) noexcept : Parent(base, base + Size, Size, "boot.dat", FILE_READ) { }
    ~ROMDataSection() override = default;
    Address makeAddressRelative(Address input) const noexcept override {
            /// @todo test the assumption that we can just return the input address since this will only fire if we invoke the proper mapped thing
            SplitWord32 newAddr(input);
            newAddr.bytes[3] &= 0b00011111; // We just need to factor out the offset
            return newAddr.wholeValue_;
    }
    bool
    respondsTo(Address address) const noexcept override {
            auto targetByte = (SplitWord32(address).bytes[3]);
            return (targetByte < 0b0100'0000) && (targetByte >= 0b0010'0000);
    }
};
#endif //SXCHIPSET_ROMSECTIONS_H
