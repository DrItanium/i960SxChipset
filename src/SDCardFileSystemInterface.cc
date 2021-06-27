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
#include "SDCardFileSystemInterface.h"


uint16_t
SDCardFilesystemInterface::seekFile() noexcept {
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
                result = theFile.seekCur(seekPositionInfo_.signedWholeValue);
                break;
            case 2: // seek_end
                result = theFile.seekEnd(seekPositionInfo_.signedWholeValue);
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

uint16_t
SDCardFilesystemInterface::invoke(uint16_t doorbellValue) noexcept {

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
            return fileFlush();
        case SDCardOperations::FileSeek:
            return seekFile();
        default:
            errorCode_ = ErrorCodes::UndefinedCommandProvided;
            return -1;
    }
    return -1;
}

uint16_t
SDCardFilesystemInterface::read16(Address address) noexcept {
    if (auto theReg = static_cast<Registers>(address); inResultArea(theReg)) {
        auto offset = (address - static_cast<Address>(Registers::Result)) / sizeof(uint16_t);
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
SDCardFilesystemInterface::write16(Address address, uint16_t value) noexcept {
    if (auto theReg = static_cast<Registers>(address); inResultArea(theReg)) {
        auto offset = (address - static_cast<Address>(Registers::Result)) / sizeof(uint16_t) ;
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
uint16_t
SDCardFilesystemInterface::getFileName() noexcept {
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
uint16_t
SDCardFilesystemInterface::getFileBytesAvailable() noexcept {
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
uint16_t
SDCardFilesystemInterface::getFilePermissions() noexcept {
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
uint16_t
SDCardFilesystemInterface::getFilePosition() noexcept {
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
uint16_t
SDCardFilesystemInterface::getFileSize() noexcept {
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
uint16_t
SDCardFilesystemInterface::getFileCoordinates() noexcept {
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
uint16_t
SDCardFilesystemInterface::fileIsOpen() noexcept {
    if (fileId_ >= MaxFileCount) {
        // bad file id!
        errorCode_ = ErrorCodes::FileIsNotValid;
        return -1;
    } else {
        result_.bytes[0] = files_[fileId_] ? -1 : 0;
        return 0;
    }
}
uint16_t
SDCardFilesystemInterface::isValidFileId() noexcept {
    result_.bytes[0] = (fileId_ < MaxFileCount && files_[fileId_]);
    return 0;
}
uint16_t
SDCardFilesystemInterface::openFile() noexcept {
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
        return 0;
    }
}
uint16_t
SDCardFilesystemInterface::readFile() noexcept {
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
        if (auto thing = getThing(baseAddress, LoadStoreStyle::Lower8); !thing) {
            errorCode_ = ErrorCodes::AttemptToReadFromUnmappedMemory;
            return -1;
        } else {
            uint32_t bytesRead = 0;
            Address count = count_.wholeValue_;
            if (count == 0) {
                result_.words[0] = 0;
                return 0;
            } else {
                // we can keep the cache on at this point in time now
                if (count > 0 && count <= TransferBufferSize) {
                    thing->disableCache();
                    bytesRead = theFile.read(transferBuffer_, count);
                    (void) thing->write(baseAddress, transferBuffer_, bytesRead);
                    result_.words[0] = bytesRead;
                    thing->enableCache();
                    return 0;
                } else {
                    thing->disableCache();
                    auto times = count / TransferBufferSize;
                    auto spillOver = count % TransferBufferSize;
                    Address a = baseAddress;
                    for (Address i = 0;
                         i < times;
                         ++i, a += TransferBufferSize) {
                        uint32_t actualBytesRead = theFile.read(transferBuffer_, TransferBufferSize);
                        (void) thing->write(a, transferBuffer_, actualBytesRead);
                        bytesRead += actualBytesRead;
                    }
                    result_.words[0] = bytesRead;
                    if (spillOver > 0) {
                        uint32_t leftOverBytesRead = theFile.read(transferBuffer_, spillOver);
                        result_.words[0] += thing->write(a, transferBuffer_, leftOverBytesRead);
                    }
                    thing->enableCache();
                    return 0;
                }
            }
        }
    }
}
uint16_t
SDCardFilesystemInterface::fileFlush() noexcept {
    if (fileId_ >= MaxFileCount) {
        // bad file id!
        errorCode_ = ErrorCodes::BadFileId;
        return -1;
    } else if (auto& theFile = files_[fileId_]; !theFile) {
        errorCode_ = ErrorCodes::FileIsNotValid;
        return -1;
    } else {
        theFile.flush();
        return 0;
    }
}
void
SDCardFilesystemInterface::begin() noexcept {
    if (!SD.begin(static_cast<int>(i960Pinout::SD_EN))) {
        signalHaltState(F("SD CARD INIT FAILED"));
    }
}
uint8_t
SDCardFilesystemInterface::read8(Address address) noexcept {
    if (auto reg = static_cast<Registers>(address); inPathArea(reg)) {
        auto offset = address - static_cast<Address>(Registers::Path);
        return static_cast<uint8_t>(path_[offset]);
    } else if (inResultArea(reg)) {
        auto offset = address - static_cast<Address>(Registers::Result);
        return result_.bytes[offset];
    }
    return 0;
}

void
SDCardFilesystemInterface::write8(Address address, uint8_t value) noexcept {
    if (auto reg = static_cast<Registers>(address); inPathArea(reg)) {
        auto offset = address - static_cast<Address>(Registers::Path);
        path_[offset] = static_cast<char>(value);

    } else if (inResultArea(reg)) {
        auto offset = address - static_cast<Address>(Registers::Result);
        result_.bytes[offset] = value;
    }
}
