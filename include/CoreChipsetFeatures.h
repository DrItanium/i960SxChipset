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

#ifndef I960SXCHIPSET_CORECHIPSETFEATURES_H
#define I960SXCHIPSET_CORECHIPSETFEATURES_H
#include "ConfigurationFlags.h"
#include "ProcessorSerializer.h"
#include "MCUPlatform.h"
#include "OpenFileHandle.h"
#include <SdFat.h>
extern SdFat SD;
class ConfigurationSpace {
public:
    static constexpr Address IOBaseAddress = 0xFE00'0000;
    static constexpr SplitWord32 IOBaseSplit { IOBaseAddress };
    static constexpr byte SectionID = IOBaseSplit.getMostSignificantByte();
    // each one of these 256 byte pages have a prescribed start and end
    static constexpr Address ConfigurationSpaceCtl0 = IOBaseAddress;
    static constexpr Address ConsoleCtl = IOBaseAddress + 0x100;
    static constexpr Address SDCtl = IOBaseAddress + 0x200;
    static constexpr Address SDFilesStart = IOBaseAddress + 0x1000;
    static constexpr Address SDFilesEnd = SDFilesStart + (MaximumNumberOfOpenFiles * 0x100);
    static constexpr Address SeesawCtl = IOBaseAddress + 0x300;
    static constexpr Address DisplayCtl = IOBaseAddress + 0x400;
    static constexpr Address RTCCtl= IOBaseAddress + 0x500;

private:


public:
    ConfigurationSpace() = delete;
    ~ConfigurationSpace() = delete;
    ConfigurationSpace(const ConfigurationSpace&) = delete;
    ConfigurationSpace(ConfigurationSpace&&) = delete;
    ConfigurationSpace& operator=(const ConfigurationSpace&) = delete;
    ConfigurationSpace& operator=(ConfigurationSpace&&) = delete;
    static void begin() noexcept;
private:
    static uint16_t readIOConfigurationSpace0(uint8_t offset) noexcept;
    static uint16_t readSerial(uint8_t offset, LoadStoreStyle ) noexcept;
    static void writeSerial(uint8_t offset, LoadStoreStyle, SplitWord16 value) noexcept;
private:
    static void unmountSDCard() noexcept;
    /**
     * @brief Try to mount/remount the primary SDCard
     * @return
     */
    static bool tryMountSDCard() noexcept;
    static uint16_t findFreeFile() noexcept;
    static uint16_t tryOpenFile() noexcept;
    static bool tryMakeDirectory(bool makeMissingParents = false) noexcept;
    static bool exists() noexcept;
    static bool remove() noexcept;
    static uint16_t sdctlRead(uint8_t offset, LoadStoreStyle lss) noexcept;
    static void sdctlWrite(uint8_t offset, LoadStoreStyle lss, SplitWord16 value) noexcept;
    static uint16_t sdfileRead(uint8_t index, uint8_t offset, LoadStoreStyle lss) noexcept;
    static void sdfileWrite(uint8_t index, uint8_t offset, LoadStoreStyle lss, SplitWord16 value);
public:
    [[nodiscard]] static uint16_t read(uint8_t targetPage, uint8_t offset, LoadStoreStyle lss) noexcept;
    static void write(uint8_t targetPage, uint8_t offset, LoadStoreStyle lss, SplitWord16 value) noexcept;
    static void prime(uint8_t targetPage) noexcept;
private:
    // sd file system structures
    static inline bool initialized_ = false;
    static inline SplitWord32 clusterCount_ {0};
    static inline SplitWord32 volumeSectorCount_ {0};
    static inline uint16_t bytesPerSector_ = 0;
    static inline uint16_t numberOfOpenFiles_ = 0;
    static inline char sdCardPath_[81] = { 0 };
    static inline OpenFileHandle files_[MaximumNumberOfOpenFiles];
    static inline bool makeMissingParentDirectories_ = false;
    static inline uint16_t filePermissions_ = 0;
    static inline bool cardMounted_ = false;
    static inline uint8_t primedPage_ = 0;
};
#endif //I960SXCHIPSET_CORECHIPSETFEATURES_H
