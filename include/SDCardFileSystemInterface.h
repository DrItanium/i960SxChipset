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

    uint8_t read8(Address address) noexcept override {
            if (address >= PathStart) {
                if (auto offset = address - PathStart; offset < FixedPathSize) {
                    return static_cast<uint8_t>(path_[offset]);
                } else {
                    // make sure we can't do an overrun!
                    return 0;
                }
            }
            if (address >= ResultStart) {
                if (auto offset = address - ResultStart; offset < 16) {
                    return result_.bytes[offset];
                } else {
                    return 0;
                }
            }
            return 0;
    }

    void write8(Address address, uint8_t value) noexcept override {
            if (address >= PathStart) {
                if (auto offset = address - PathStart; offset < FixedPathSize) {
                    path_[offset] = static_cast<char>(value);
                } else {
                    // should not get here! so fail out hardcore!
                    signalHaltState(F("LARGER THAN FIXED PATH SIZE!!!"));
                }
            }
            if (address >= ResultStart) {
                if (auto offset = address - ResultStart; offset < 16) {
                    result_.bytes[offset] = value;
                }
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
            case SDCardOperations::FileRead:
                signalHaltState(F("UNIMPLEMENTED FUNCTION FILE READ"));
                break;
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
    enum class Registers : uint16_t {
        Doorbell, // two bytes
        Command,
        FileId,
        ModeBits,
        SeekPositionLower,
        SeekPositionUpper,
        Whence,
        PermissionBitsLower,
        PermissionBitsUpper,
        OpenReadWrite,
        ErrorCode,
        // Path and result are handled differently but we can at least compute base offsets
        Result, // 16 bytes in size
        Path, // FixedPathSize bytes
        // always last
        BuffersStartAt = ErrorCode,
    };
    uint16_t read16(Address address) noexcept override {
            if (address >= ResultStart && address < PathStart) {
                if (auto offset = (address - ResultStart) / 2; offset < 8) {
                    return result_.shorts[offset];
                } else {
                    return 0;
                }
            }
            switch (address) {
                case static_cast<Address>(Registers::Doorbell) * 2: return invoke(0);
                case static_cast<Address>(Registers::Command) * 2: return static_cast<uint16_t>(command_);
                case static_cast<Address>(Registers::FileId) * 2: return fileId_;
                case static_cast<Address>(Registers::ModeBits) * 2: return modeBits_;
                case static_cast<Address>(Registers::SeekPositionLower) * 2: return seekPositionInfo_.halves[0];
                case static_cast<Address>(Registers::SeekPositionUpper) * 2: return seekPositionInfo_.halves[1];
                case static_cast<Address>(Registers::PermissionBitsLower) * 2: return flags_.halves[0];
                case static_cast<Address>(Registers::PermissionBitsUpper) * 2: return flags_.halves[1];
                case static_cast<Address>(Registers::Whence) * 2: return whence_;
                case static_cast<Address>(Registers::ErrorCode) * 2: return static_cast<uint16_t>(errorCode_);
                case static_cast<Address>(Registers::OpenReadWrite) * 2: return static_cast<uint16_t>(openReadWrite_);
                default: return 0;
            }
    }
    static constexpr auto ResultStart = static_cast<int>(Registers::BuffersStartAt) * 2;
    static constexpr auto PathStart = ResultStart + 16;
    void write16(Address address, uint16_t value) noexcept override {
            if (address >= ResultStart && address < PathStart) {
                if (auto offset = (address - ResultStart) / 2; offset < 8) {
                    result_.shorts[offset] = value;
                }
            } else {
                switch (address) {
                    case static_cast<Address>(Registers::Doorbell) * 2:
                        (void)invoke(value);
                        break;
                    case static_cast<Address>(Registers::Command) * 2:
                        command_ = static_cast<SDCardOperations>(value);
                        break;
                    case static_cast<Address>(Registers::FileId) * 2:
                        fileId_ = value;
                        break;
                    case static_cast<Address>(Registers::ModeBits) * 2:
                        modeBits_ = value;
                        break;
                    case static_cast<Address>(Registers::SeekPositionLower) * 2:
                        seekPositionInfo_.halves[0] = value;
                        break;
                    case static_cast<Address>(Registers::SeekPositionUpper) * 2:
                        seekPositionInfo_.halves[1] = value;
                        break;
                    case static_cast<Address>(Registers::Whence) * 2:
                        whence_ = value;
                        break;
                    case static_cast<Address>(Registers::ErrorCode) * 2:
                        errorCode_ = static_cast<ErrorCodes>(value);
                        break;
                    case static_cast<Address>(Registers::OpenReadWrite) * 2:
                        openReadWrite_ = value != 0;
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
        if (openedFileCount_ == MaxFileCount) {
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
    // these are the actual addresses
private:
    uint16_t openedFileCount_ = 0;
    File files_[MaxFileCount];
    uint32_t permissions_[MaxFileCount] = { 0 };
    SDCardOperations command_ = SDCardOperations::None;
    uint16_t fileId_ = 0;
    uint16_t modeBits_ = 0;
    union {
        uint32_t wholeValue_ = 0;
        uint16_t halves[sizeof(uint32_t) / sizeof(uint16_t)];
    } seekPositionInfo_;
    uint16_t whence_ = 0;
    union {
        uint32_t wholeValue_ = 0;
        uint16_t halves[sizeof(uint32_t) / sizeof(uint16_t)];
    } flags_;
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

};
#endif //I960SXCHIPSET_SDCARDFILESYSTEMINTERFACE_H
