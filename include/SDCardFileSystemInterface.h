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

#ifndef I960SXCHIPSET_SDCARDFILESYSTEMINTERFACE_H
#define I960SXCHIPSET_SDCARDFILESYSTEMINTERFACE_H
#include "MemoryThing.h"
#include <SdFat.h>
extern SdFat SD;
class SDCardFilesystemInterface : public IOSpaceThing {
public:
    static constexpr uint8_t FixedPathSize = 80;
    static constexpr auto MaxFileCount = TargetBoard::maximumNumberOfOpenFilesFromSDCard();
    static constexpr auto ReadBufferSize = 64;
    static constexpr auto WriteBufferSize = 64;
public:
    explicit SDCardFilesystemInterface(Address base) : IOSpaceThing(base, base + 0x100) { }
    void begin() noexcept override;
    enum class Registers : uint16_t {
#define X(name) name, name ## Upper
#define Y(name) \
        X(name ## 0), \
        X(name ## 1), \
        X(name ## 2), \
        X(name ## 3)

        X(Doorbell),
        X(Command),
        X(FileId),
        X(ModeBits),
        X(SeekPositionLower),
        X(SeekPositionUpper),
        X(Whence),
        X(PermissionBitsLower),
        X(PermissionBitsUpper),
        X(OpenReadWrite),
        X(ErrorCode),
        X(Result0),
        X(Result1),
        X(Result2),
        X(Result3),
        X(Result4),
        X(Result5),
        X(Result6),
        X(Result7),
        Y(Path0),
        Y(Path1),
        Y(Path2),
        Y(Path3),
        Y(Path4),
        Y(Path5),
        Y(Path6),
        Y(Path7),
        Y(Path8),
        Y(Path9),
        X(PathEnd),
        X(AddressLower),
        X(AddressUpper),
        X(CountLower),
        X(CountUpper),
#undef Y
#undef X

        // Path and result are handled differently but we can at least compute base offsets
        Path = Path00, // FixedPathSize bytes
        Result = Result0, // 16 bytes in size
        Address = AddressLower,
        Count = CountLower,
    };
    static_assert(static_cast<int>(Registers::Address) - static_cast<int>(Registers::PathEnd) == 2, "There should be a buffer following the path container");
    static constexpr bool inResultArea(Registers value) noexcept {
        switch(value) {
#define X(name) \
    case Registers:: name : \
    case Registers:: name ## Upper
            X(Result0):
            X(Result1):
            X(Result2):
            X(Result3):
            X(Result4):
            X(Result5):
            X(Result6):
            X(Result7):
#undef X
            return true;
            default:
                return false;
        }

    }
    static constexpr bool inPathArea(Registers value) noexcept {
        switch (value) {

#define X(name) \
    case Registers:: name : \
    case Registers:: name ## Upper

#define Y(name) \
        X(name ## 0): \
        X(name ## 1): \
        X(name ## 2): \
        X(name ## 3)
            Y(Path0):
            Y(Path1):
            Y(Path2):
            Y(Path3):
            Y(Path4):
            Y(Path5):
            Y(Path6):
            Y(Path7):
            Y(Path8):
            Y(Path9):
#undef X
#undef Y
            return true;
            default:
                return false;
        }
    }
    static_assert((static_cast<int>(Registers::PathEnd) - static_cast<int>(Registers::Path)) == FixedPathSize, "Path is not of the correct size");
    uint8_t read8(Address address) noexcept override;
    void write8(Address address, uint8_t value) noexcept override;

    enum class SDCardOperations : uint16_t {
        None = 0,
        // General SD Operations
        OpenFile,
        CloseFile,
        FileExists,
        MakeDirectory,
        RemoveDirectory,
        GetNumberOfOpenFiles,
        GetMaximumNumberOfOpenFiles,
        GetFixedPathMaximum,
        // File specific operations
        IsValidFileId = 0x8000,
        FileRead,
        FileWrite,
        FileFlush,
        FileSeek,
        FileIsOpen,
        GetFileName,
        GetFileBytesAvailable,
        GetFilePosition,
        GetFilePermissions,
        GetFileSize,
        /**
         * @brief Return the position and size of the given file in a single call
         */
        GetFileCoordinates,
    };

    enum class ErrorCodes : uint16_t {
        None = 0,
        NoCommandProvided,
        UndefinedCommandProvided,
        BadFileId,
        FileIsNotValid,
        /**
         * @brief Attempts to open ram.bin, boot.rom, or boot.data will trigger this fault
         */
        CriticalFileSideChannelAttempt,
        UnimplementedCommand,
        AllFileSlotsInUse,
        AttemptToReadFromUnmappedMemory,
        AttemptToWriteToUnmappedMemory,
        UnableToSeekToRequestedDestination,
    };
public:
    uint16_t invoke(uint16_t doorbellValue) noexcept;
    uint16_t read16(Address address) noexcept override;
    void write16(Address address, uint16_t value) noexcept override;
private:
    uint16_t getFileName() noexcept;
    uint16_t getFileBytesAvailable() noexcept;
    uint16_t getFilePermissions() noexcept;
    uint16_t getFilePosition() noexcept;
    uint16_t getFileSize() noexcept;
    uint16_t getFileCoordinates() noexcept;
    uint16_t fileIsOpen() noexcept;
    uint16_t isValidFileId() noexcept;
    uint16_t openFile() noexcept;
    uint16_t readFile() noexcept;
    uint16_t seekFile() noexcept;
    uint16_t fileFlush() noexcept;
private:
    union SplitWord {
        uint32_t wholeValue_ = 0;
        int32_t signedWholeValue;
        uint16_t halves[sizeof(uint32_t) / sizeof(uint16_t)];
    };
    uint16_t openedFileCount_ = 0;
    File files_[MaxFileCount];
    uint32_t permissions_[MaxFileCount] = { 0 };
    SDCardOperations command_ = SDCardOperations::None;
    uint16_t fileId_ = 0;
    uint16_t modeBits_ = 0;
    SplitWord seekPositionInfo_;
    uint16_t whence_ = 0;
    SplitWord flags_;
    bool openReadWrite_ = false;
    union {
        uint8_t bytes[16];
        uint16_t shorts[16/sizeof(uint16_t)];
        uint32_t words[16/sizeof(uint32_t)];
        uint64_t quads[16/sizeof(uint64_t)];
    } result_;
    ErrorCodes errorCode_ = ErrorCodes::None;
    char path_[FixedPathSize] = { 0 };
    volatile uint32_t fixedPadding = 0; // always should be here to make sure an overrun doesn't cause problems
    SplitWord address_;
    SplitWord count_;
    uint8_t readBuffer_[ReadBufferSize] = { 0 };
    uint8_t writeBuffer_[WriteBufferSize] = { 0 };
};
#endif //I960SXCHIPSET_SDCARDFILESYSTEMINTERFACE_H
