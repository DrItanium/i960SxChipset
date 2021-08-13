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
