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
// Created by jwscoggins on 8/13/21.
//

#ifndef SXCHIPSET_RAMFILE_H
#define SXCHIPSET_RAMFILE_H
#include "Pinout.h"
#include "MemoryMappedFileThing.h"
class RAMFile : public MemoryMappedFile {
    //<TargetBoard::numberOfDataCacheLines(), TargetBoard::getDataCacheLineSize()>
public:
    static constexpr Address Size = 512_MB;
    static constexpr auto Mask = Size - 1;
    using Parent = MemoryMappedFile;
    explicit RAMFile(Address baseAddress) noexcept : Parent(baseAddress, baseAddress + Size, Size, "ram.bin", FILE_WRITE) { }
    ~RAMFile() override = default;
    void begin() noexcept override {
            Parent::begin();
            if (Parent::getFileSize() != Size) {
                signalHaltState(F("RAM.BIN MUST BE 512 MEGS IN SIZE!"));
            }
    }
};
#endif //SXCHIPSET_RAMFILE_H
