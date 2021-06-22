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

#ifndef I960SXCHIPSET_MEMORYMAPPEDFILETHING_H
#define I960SXCHIPSET_MEMORYMAPPEDFILETHING_H
#include <Arduino.h>
#include "MCUPlatform.h"
#include "MemoryThing.h"
#include "Cache.h"
#include <SdFat.h>
extern SdFat SD;

class MemoryMappedFile : public MemoryThing {
public:
    MemoryMappedFile(Address startingAddress, Address endingAddress, Address maximumSize, const char* path, decltype(FILE_WRITE) permissions) noexcept : MemoryThing(startingAddress, endingAddress), maxSize_(maximumSize), path_(path), permissions_(permissions) { }
    ~MemoryMappedFile() override {
        // while this will never get called, it is still a good idea to be complete
        theFile_.close();
    }
    void
    begin() noexcept override {
        if (!SD.exists(const_cast<char*>(path_))) {
            // delete the file and start a new
            signalHaltState(F("Could not find ram.bin! Please create one 512 megs in size!"));
        }
        theFile_ = SD.open(path_, permissions_);
        if (!theFile_) {
            Serial.print(F("Couldn't open "));
            Serial.println(path_);
            signalHaltState(F("Could not open memory mapped file! SD CARD may be corrupt")) ;
        }
        fileSize_ = theFile_.size();
        if (fileSize_ > maxSize_) {
            signalHaltState(F("File too large!"));
        }
        Serial.print(path_);
        Serial.println(F(" OPEN SUCCESS!"));
        //(void)theCache_.getByte(0); // cache something into memory on startup to improve performance
    }
protected:
    [[nodiscard]] uint8_t
    read8(Address offset) noexcept override {
        theFile_.seekSet(offset);
        return static_cast<uint8_t>(theFile_.read());
    }
    void
    write8(Address offset, uint8_t value) noexcept override {
        if (permissions_ != FILE_READ) {
            theFile_.seekSet(offset);
            theFile_.write(value);
        }
    }
    [[nodiscard]] uint16_t
    read16(Address offset) noexcept override {
        theFile_.seekSet(offset);
        uint16_t value = 0;
        theFile_.read(reinterpret_cast<char*>(&value), sizeof(value));
        return value;
    }
    void
    write16(Address offset, uint16_t value) noexcept override {
        if (permissions_ != FILE_READ) {
            theFile_.seekSet(offset);
            theFile_.write(reinterpret_cast<char *>(&value), sizeof(value));
        }
    }
    size_t blockWrite(Address address, uint8_t *buf, size_t capacity) noexcept override {
        if (permissions_ != FILE_READ) {
            theFile_.seek(address);
            auto result = theFile_.write(buf, capacity);
            // make sure...
            theFile_.flush();
            return result;
        }
        return 0;
    }
    size_t blockRead(Address address, uint8_t *buf, size_t capacity) noexcept override {
        // at this point the methods that call this will have fixed up the addresses and sizes to prevent spanning across this file
        theFile_.seek(address);
        return theFile_.readBytes(buf, capacity);
    }
    [[nodiscard]] constexpr auto getFileSize() const noexcept { return fileSize_; }

private:
    File theFile_; // use an SDCard as ram for the time being
    Address maxSize_;
    const char* path_;
    decltype(FILE_WRITE) permissions_;
    Address fileSize_;
};

#endif //I960SXCHIPSET_MEMORYMAPPEDFILETHING_H
