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
#include "MemoryThing.h"
#include "Cache.h"
template<uint32_t numCacheLines = 8, uint32_t cacheLineSize = 512>
class MemoryMappedFile : public MemoryThing {
public:
    static constexpr auto CacheLineCount = numCacheLines < 1 ? 1 : numCacheLines;
    static constexpr auto CacheLineSize = cacheLineSize < 16 ? 16 : cacheLineSize;
    MemoryMappedFile(Address startingAddress, Address endingAddress, Address maximumSize, const char* path, decltype(FILE_WRITE) permissions) noexcept : MemoryThing(startingAddress, endingAddress), maxSize_(maximumSize), theCache_(this), path_(path), permissions_(permissions) { }
    ~MemoryMappedFile() override {
        // while this will never get called, it is still a good idea to be complete
        theFile_.close();
    }
    [[nodiscard]] uint8_t
    read8(Address offset) noexcept override {
            return theCache_.getByte(offset);
    }
    void
    write8(Address offset, uint8_t value) noexcept override {
            theCache_.setByte(offset, value);
    }
    [[nodiscard]] uint16_t
    read16(Address offset) noexcept override {
            return theCache_.getWord(offset);
    }
    void
    write16(Address offset, uint16_t value) noexcept override {
            theCache_.setWord(offset, value);
    }
public:
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
            (void)theCache_.getByte(0); // cache something into memory on startup to improve performance
    }
private:
public:
    void write(uint32_t baseAddress, byte* buffer, size_t size) override {
        if (permissions_ != FILE_READ) {
            theFile_.seek(baseAddress);
            theFile_.write(buffer, size);
            // make sure...
            theFile_.flush();
        }
    }
    void read(uint32_t baseAddress, byte *buffer, size_t size) override {
        theFile_.seek(baseAddress);
        theFile_.read(buffer, size);
    }
    [[nodiscard]] constexpr auto getFileSize() const noexcept { return fileSize_; }
private:
    File theFile_; // use an SDCard as ram for the time being
    Address maxSize_;
    DataCache<CacheLineCount, CacheLineSize> theCache_;
    const char* path_;
    decltype(FILE_WRITE) permissions_;
    Address fileSize_;
};
#endif //I960SXCHIPSET_MEMORYMAPPEDFILETHING_H
