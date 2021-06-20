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
public:
    explicit SDCardFilesystemInterface(Address base) : IOSpaceThing(base, base + 0x100) { }
    void begin() noexcept override {
#ifdef ADAFRUIT_GRAND_CENTRAL_M4
        if (!SD.begin(static_cast<int>(i960Pinout::SD_EN),
                      static_cast<int>(i960Pinout::SD_MOSI),
                      static_cast<int>(i960Pinout::SD_MISO),
                      static_cast<int>(i960Pinout::SD_SCK))) {
            signalHaltState(F("SD CARD INIT FAILED"));
        }
#else
        if (!SD.begin(static_cast<int>(i960Pinout::SD_EN))) {
                signalHaltState(F("SD CARD INIT FAILED"));
        }
#endif
    }
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

    uint8_t read8(Address address) noexcept override {
        if (auto reg = static_cast<Registers>(address); inPathArea(reg)) {
            auto offset = address - static_cast<Address>(Registers::Path);
            return static_cast<uint8_t>(path_[offset]);
        } else if (inResultArea(reg)) {
            auto offset = address - static_cast<Address>(Registers::Result);
            return result_.bytes[offset];
        }
        return 0;
    }

    void write8(Address address, uint8_t value) noexcept override {
        if (auto reg = static_cast<Registers>(address); inPathArea(reg)) {
            auto offset = address - static_cast<Address>(Registers::Path);
            path_[offset] = static_cast<char>(value);

        } else if (inResultArea(reg)) {
            auto offset = address - static_cast<Address>(Registers::Result);
            result_.bytes[offset] = value;
        }
    }

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
    uint16_t invoke(uint16_t doorbellValue) noexcept {

        // clear the error code on startup
        errorCode_ = ErrorCodes::None;
        result_.quads[0] = -1;
        result_.quads[1] = -1;
        switch (command_) {
            case SDCardOperations::None:
                errorCode_ = ErrorCodes::NoCommandProvided;
                result_.words[0] = -1;
                return -1;
            case SDCardOperations::FileExists:
                // oh man this is freaking dangerous but I have put in a zero padding buffer following the byte addresses
                // so that should prevent a host of problems
                result_.bytes[0] = SD.exists(path_);
                return 0;
            case SDCardOperations::GetNumberOfOpenFiles:
                result_.words[0] = openedFileCount_;
                return 0;
            case SDCardOperations::GetMaximumNumberOfOpenFiles:
                result_.words[0] = MaxFileCount;
                return 0;
            case SDCardOperations::GetFileName: return getFileName();
            case SDCardOperations::IsValidFileId: return isValidFileId();
            case SDCardOperations::GetFixedPathMaximum:
                result_.bytes[0] = FixedPathSize;
                return 0;
            case SDCardOperations::GetFileBytesAvailable: return getFileBytesAvailable();
            case SDCardOperations::GetFilePosition: return getFilePosition();
            case SDCardOperations::GetFilePermissions: return getFilePermissions();
            case SDCardOperations::GetFileSize: return getFileSize();
            case SDCardOperations::GetFileCoordinates: return getFileCoordinates();
            case SDCardOperations::FileIsOpen: return fileIsOpen();
            case SDCardOperations::OpenFile: return openFile();
            case SDCardOperations::CloseFile:
                signalHaltState(F("UNIMPLEMENTED FUNCTION CLOSE FILE"));
                break;
            case SDCardOperations::MakeDirectory:
                signalHaltState(F("UNIMPLEMENTED FUNCTION MAKE DIRECTORY"));
                break;
            case SDCardOperations::RemoveDirectory:
                signalHaltState(F("UNIMPLEMENTED FUNCTION REMOVE DIRECTORY"));
                break;
            case SDCardOperations::FileRead: return readFile();
            case SDCardOperations::FileWrite:
                signalHaltState(F("UNIMPLEMENTED FUNCTION FILE WRITE"));
                break;
            case SDCardOperations::FileFlush:
                signalHaltState(F("UNIMPLEMENTED FUNCTION FILE FLUSH"));
                break;
            case SDCardOperations::FileSeek:
                errorCode_ = ErrorCodes::UnimplementedCommand;
                return -1;
            default:
                errorCode_ = ErrorCodes::UndefinedCommandProvided;
                return -1;
        }
        return -1;
    }
    uint16_t read16(Address address) noexcept override {
        if (auto theReg = static_cast<Registers>(address); inResultArea(theReg)) {
                auto offset = address - static_cast<Address>(Registers::Result);
                return result_.shorts[offset];
        }  else {
            switch (theReg) {
                case Registers::Doorbell:
                    return invoke(0);
                case Registers::Command:
                    return static_cast<uint16_t>(command_);
                case Registers::FileId:
                    return fileId_;
                case Registers::ModeBits:
                    return modeBits_;
                case Registers::SeekPositionLower:
                    return seekPositionInfo_.halves[0];
                case Registers::SeekPositionUpper:
                    return seekPositionInfo_.halves[1];
                case Registers::PermissionBitsLower:
                    return flags_.halves[0];
                case Registers::PermissionBitsUpper:
                    return flags_.halves[1];
                case Registers::Whence:
                    return whence_;
                case Registers::ErrorCode:
                    return static_cast<uint16_t>(errorCode_);
                case Registers::OpenReadWrite:
                    return static_cast<uint16_t>(openReadWrite_);
                case Registers::AddressLower:
                    return address_.halves[0];
                case Registers::AddressUpper:
                    return address_.halves[1];
                case Registers::CountLower:
                    return count_.halves[0];
                case Registers::CountUpper:
                    return count_.halves[1];
                default:
                    return 0;
            }
        }
    }
    void
    write16(Address address, uint16_t value) noexcept override {
        if (auto theReg = static_cast<Registers>(address); inResultArea(theReg)) {
            auto offset = address - static_cast<Address>(Registers::Result) ;
            result_.shorts[offset] = value;
        } else {
            switch (theReg) {
                case Registers::Doorbell:
                    (void)invoke(value);
                    break;
                case Registers::Command:
                    command_ = static_cast<SDCardOperations>(value);
                    break;
                case Registers::FileId:
                    fileId_ = value;
                    break;
                case Registers::ModeBits:
                    modeBits_ = value;
                    break;
                case Registers::SeekPositionLower:
                    seekPositionInfo_.halves[0] = value;
                    break;
                case Registers::SeekPositionUpper:
                    seekPositionInfo_.halves[1] = value;
                    break;
                case Registers::Whence:
                    whence_ = value;
                    break;
                case Registers::ErrorCode:
                    errorCode_ = static_cast<ErrorCodes>(value);
                    break;
                case Registers::OpenReadWrite:
                    openReadWrite_ = value != 0;
                case Registers::AddressLower:
                    address_.halves[0] = value;
                    break;
                case Registers::AddressUpper:
                    address_.halves[1] = value;
                    break;
                case Registers::CountLower:
                    count_.halves[0] = value;
                    break;
                case Registers::CountUpper:
                    count_.halves[1] = value;
                    break;
                default:
                    break;
            }
        }
    }

private:
    uint16_t getFileName() noexcept {
        if (fileId_ >= MaxFileCount) {
            // bad file id!
            errorCode_ = ErrorCodes::BadFileId;
            return -1;
        } else if (!files_[fileId_]){
            errorCode_ = ErrorCodes::FileIsNotValid;
            return -1;
        } else {
            auto& file = files_[fileId_];
            const char* name = file.name();
            // 8.3 file names assumption!!!
            for (int i = 0; i < 13; ++i) {
                result_.bytes[i] = name[i];
            }
            result_.bytes[13] = 0;
            return 0;
        }
    }
    uint16_t getFileBytesAvailable() noexcept {
        if (fileId_ >= MaxFileCount) {
            // bad file id!
            errorCode_ = ErrorCodes::BadFileId;
            return -1;
        } else if (!files_[fileId_]){
            errorCode_ = ErrorCodes::FileIsNotValid;
            return -1;
        } else {
            result_.shorts[0] = static_cast<uint16_t>(files_[fileId_].available());
            return 0;
        }
    }
    uint16_t getFilePermissions() noexcept {
        if (fileId_ >= MaxFileCount) {
            // bad file id!
            errorCode_ = ErrorCodes::BadFileId;
            return -1;
        } else if (!files_[fileId_]){
            errorCode_ = ErrorCodes::FileIsNotValid;
            return -1;
        } else {
            result_.shorts[0] = permissions_[fileId_];
            return 0;
        }
    }
    uint16_t getFilePosition() noexcept {
        if (fileId_ >= MaxFileCount) {
            // bad file id!
            errorCode_ = ErrorCodes::BadFileId;
            return -1;
        } else if (!files_[fileId_]){
            errorCode_ = ErrorCodes::FileIsNotValid;
            return -1;
        } else {
            result_.words[0] = files_[fileId_].position();
            return 0;
        }
    }
    uint16_t getFileSize() noexcept {
        if (fileId_ >= MaxFileCount) {
            // bad file id!
            errorCode_ = ErrorCodes::BadFileId;
            return -1;
        } else if (!files_[fileId_]){
            errorCode_ = ErrorCodes::FileIsNotValid;
            return -1;
        } else {
            result_.words[0] = files_[fileId_].size();
            return 0;
        }
    }
    uint16_t getFileCoordinates() noexcept {
        if (fileId_ >= MaxFileCount) {
            // bad file id!
            errorCode_ = ErrorCodes::BadFileId;
            return -1;
        } else if (!files_[fileId_]){
            errorCode_ = ErrorCodes::FileIsNotValid;
            return -1;
        } else {
            result_.words[0] = files_[fileId_].position();
            result_.words[1] = files_[fileId_].size();
            return 0;
        }
    }
    uint16_t fileIsOpen() noexcept {
        if (fileId_ >= MaxFileCount) {
            // bad file id!
            errorCode_ = ErrorCodes::FileIsNotValid;
            return -1;
        } else {
            result_.bytes[0] = files_[fileId_] ? -1 : 0;
            return 0;
        }
    }
    uint16_t isValidFileId() noexcept {
        result_.bytes[0] = (fileId_ < MaxFileCount && files_[fileId_]);
        return 0;
    }
    uint16_t openFile() noexcept {
        if (openedFileCount_ >= MaxFileCount) {
            // too many files opened
            errorCode_ = ErrorCodes::AllFileSlotsInUse;
            openedFileCount_ =  MaxFileCount;
            return -1;
        } else {
            // okay, so lets do an open from this point
            // we also need to decode the
            permissions_[openedFileCount_] = flags_.wholeValue_;
            files_[openedFileCount_] = SD.open(path_, openReadWrite_ ? FILE_WRITE : FILE_READ);
            if (!files_[openedFileCount_]) {
                errorCode_ = ErrorCodes::FileIsNotValid;
                return -1;
            }
            auto handleId = openedFileCount_;
            ++openedFileCount_;
            result_.words[0] = handleId;
            Serial.print(F("Opened file on sd card at path \""));
            Serial.print(path_);
            if (openReadWrite_) {
                Serial.println(F("\" for Reading and Writing"));
            } else {
                Serial.println(F("\" for Reading"));
            }
            return 0;
        }
    }
    uint16_t readFile() noexcept {
        // we have the fileId, address to read into within memory, and the number of items to write
        if (fileId_ >= MaxFileCount) {
            // bad file id!
            errorCode_ = ErrorCodes::BadFileId;
            return -1;
        } else if (auto& theFile = files_[fileId_]; !theFile) {
            errorCode_ = ErrorCodes::FileIsNotValid;
            return -1;
        } else {
            Address baseAddress = address_.wholeValue_;
            Address count = count_.wholeValue_;
            auto thing = getThing(baseAddress, LoadStoreStyle::Lower8);
            if (!thing) {
                errorCode_ = ErrorCodes::AttemptToReadFromUnmappedMemory;
                return -1;
            }
            uint32_t bytesRead = 0;
            for (Address i = 0, j = baseAddress; i < count; ++i, ++j) {
                // slow but the simplest design for now since we are jumping between radically different implementations of things
                // read byte by byte into the thing
                if (auto value = theFile.read(); value == -1) {
                    thing->write(j, 0, LoadStoreStyle::Lower8) ; // we've gone beyond the end of the original file so just write zeros to be safe
                } else {
                    thing->write(j, static_cast<uint8_t>(value), LoadStoreStyle::Lower8);
                    ++bytesRead;
                }
            }
            result_.words[0] = bytesRead;
            return 0;
        }
    }
    uint16_t seekFile() noexcept {
        if (fileId_ >= MaxFileCount) {
            // bad file id!
            errorCode_ = ErrorCodes::BadFileId;
            return -1;
        } else if (auto& theFile = files_[fileId_]; !theFile) {
            errorCode_ = ErrorCodes::FileIsNotValid;
            return -1;
        } else {
            bool result = false;
            switch (whence_) {
                case 1: // seek_cur
                    result = theFile.seekCur(seekPositionInfo_.wholeValue_);
                    break;
                case 2: // seek_end
                    result = theFile.seekEnd(seekPositionInfo_.wholeValue_);
                    break;
                case 0: // seek_set
                default:
                    result = theFile.seekSet(seekPositionInfo_.wholeValue_);
                    break;
            }
            if (!result) {
                errorCode_ = ErrorCodes::UnableToSeekToRequestedDestination;
                return -1;
            } else {
                result_.words[0] = theFile.position();
                return 0;
            }
        }
    }
private:
    union SplitWord {
        uint32_t wholeValue_ = 0;
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
};
#endif //I960SXCHIPSET_SDCARDFILESYSTEMINTERFACE_H
