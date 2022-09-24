/*
i960SxChipset
Copyright (c) 2020-2022, Joshua Scoggins
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

#include "CoreChipsetFeatures.h"
namespace
{
    constexpr uint32_t AddressTable[] PROGMEM = {
            ConfigurationSpace::ConsoleCtl,
            ConfigurationSpace::SDCtl,
            ConfigurationSpace::SDFilesStart,
            ConfigurationSpace::SeesawCtl,
            ConfigurationSpace::DisplayCtl,
            ConfigurationSpace::RTCCtl,
    };
    constexpr auto NumWordsCS0 = sizeof(AddressTable) / (sizeof(uint16_t));
    enum class SeesawRegisters : uint8_t
    {
#define TwoByteEntry(Prefix) Prefix ## 0, Prefix ## 1
#define FourByteEntry(Prefix) \
        TwoByteEntry(Prefix ## 0), \
        TwoByteEntry(Prefix ## 1)
#define EightByteEntry(Prefix) \
        FourByteEntry(Prefix ## 0), \
        FourByteEntry(Prefix ## 1)
#define TwelveByteEntry(Prefix) \
        EightByteEntry(Prefix ## 0), \
        FourByteEntry(Prefix ## 1)
#define SixteenByteEntry(Prefix) \
        EightByteEntry(Prefix ## 0), \
        EightByteEntry(Prefix ## 1)
        TwoByteEntry(Backlight),
#undef SixteenByteEntry
#undef TwelveByteEntry
#undef EightByteEntry
#undef FourByteEntry
#undef TwoByteEntry
        End,
        Backlight = Backlight0,
    };
    enum class DisplayInterfaceRegisters : uint8_t
    {
#define TwoByteEntry(Prefix) Prefix ## 0, Prefix ## 1
#define FourByteEntry(Prefix) \
        TwoByteEntry(Prefix ## 0), \
        TwoByteEntry(Prefix ## 1)
#define EightByteEntry(Prefix) \
        FourByteEntry(Prefix ## 0), \
        FourByteEntry(Prefix ## 1)
#define TwelveByteEntry(Prefix) \
        EightByteEntry(Prefix ## 0), \
        FourByteEntry(Prefix ## 1)
#define SixteenByteEntry(Prefix) \
        EightByteEntry(Prefix ## 0), \
        EightByteEntry(Prefix ## 1)
        SixteenByteEntry(InstructionPort0),
        TwoByteEntry(Invoke0),
#undef SixteenByteEntry
#undef TwelveByteEntry
#undef EightByteEntry
#undef FourByteEntry
#undef TwoByteEntry
        End,
#define X(index) \
        InstructionField ## index ## 0 = InstructionPort ## index ## 0000, \
        InstructionField ## index ## 1 = InstructionPort ## index ## 0010, \
        InstructionField ## index ## 2 = InstructionPort ## index ## 0100, \
        InstructionField ## index ## 3 = InstructionPort ## index ## 0110, \
        InstructionField ## index ## 4 = InstructionPort ## index ## 1000, \
        InstructionField ## index ## 5 = InstructionPort ## index ## 1010, \
        InstructionField ## index ## 6 = InstructionPort ## index ## 1100, \
        InstructionField ## index ## 7 = InstructionPort ## index ## 1110

        X(0),
#undef X
        Invoke0 = Invoke00,
    };
    enum class SerialRegisters : uint8_t
    {
#define TwoByteEntry(Prefix) Prefix ## 0, Prefix ## 1
#define FourByteEntry(Prefix) \
        TwoByteEntry(Prefix ## 0), \
        TwoByteEntry(Prefix ## 1)
#define EightByteEntry(Prefix) \
        FourByteEntry(Prefix ## 0), \
        FourByteEntry(Prefix ## 1)
#define TwelveByteEntry(Prefix) \
        EightByteEntry(Prefix ## 0), \
        FourByteEntry(Prefix ## 1)
#define SixteenByteEntry(Prefix) \
        EightByteEntry(Prefix ## 0), \
        EightByteEntry(Prefix ## 1)
        TwoByteEntry(ConsoleIO),
        TwoByteEntry(ConsoleFlush),
        //FourByteEntry(ConsoleTimeout),
        //TwoByteEntry(ConsoleRXBufferSize),
        //TwoByteEntry(ConsoleTXBufferSize),
        //FourByteEntry(ChipsetClockSpeed),
        //TwoByteEntry(CacheLineCount),
        //TwoByteEntry(CacheLineSize),
        //TwoByteEntry(NumberOfCacheWays),
        TwoByteEntry(TriggerInterrupt),
        FourByteEntry(AddressDebuggingFlag),
#undef SixteenByteEntry
#undef TwelveByteEntry
#undef EightByteEntry
#undef FourByteEntry
#undef TwoByteEntry
        End,
        ConsoleIO = ConsoleIO0,
        ConsoleFlush = ConsoleFlush0,
        TriggerInterrupt = TriggerInterrupt0,
        AddressDebuggingFlag = AddressDebuggingFlag00,
        // we ignore the upper half of the register but reserve it to make sure
    };
    enum class SDCardFileSystemRegisters : uint8_t
    {
#define TwoByteEntry(Prefix) Prefix ## 0, Prefix ## 1
#define FourByteEntry(Prefix) \
        TwoByteEntry(Prefix ## 0), \
        TwoByteEntry(Prefix ## 1)
#define EightByteEntry(Prefix) \
        FourByteEntry(Prefix ## 0), \
        FourByteEntry(Prefix ## 1)
#define TwelveByteEntry(Prefix) \
        EightByteEntry(Prefix ## 0), \
        FourByteEntry(Prefix ## 1)
#define SixteenByteEntry(Prefix) \
        EightByteEntry(Prefix ## 0), \
        EightByteEntry(Prefix ## 1)
        SixteenByteEntry(Path0),
        SixteenByteEntry(Path1),
        SixteenByteEntry(Path2),
        SixteenByteEntry(Path3),
        SixteenByteEntry(Path4),
        TwoByteEntry(OpenPort),
        TwoByteEntry(MakeDirectoryPort),
        TwoByteEntry(ExistsPort),
        TwoByteEntry(RemovePort),
        FourByteEntry(SDClusterCount),
        FourByteEntry(SDVolumeSectorCount),
        TwoByteEntry(SDBytesPerSector),
        TwoByteEntry(NumberOfOpenFiles),
        TwoByteEntry(MaximumNumberOfOpenFiles),
        TwoByteEntry(ErrorCode),
        TwoByteEntry(MakeMissingParentDirectories),
        TwoByteEntry(FilePermissions), // raw interface
        TwoByteEntry(OpenReadWrite), // O_READ | O_WRITE
        TwoByteEntry(OpenReadOnly), // O_READ
        TwoByteEntry(OpenWriteOnly), // O_WRITE
        TwoByteEntry(CreateFileIfMissing), // O_CREAT
        TwoByteEntry(ClearFileContentsOnOpen), // O_TRUNC
        TwoByteEntry(MountCTL), // controls mount/unmount functionality when writing and reading it yields the status

#undef SixteenByteEntry
#undef TwelveByteEntry
#undef EightByteEntry
#undef FourByteEntry
#undef TwoByteEntry
        End,
        PathStart = Path00000,
        PathEnd = Path41111,
        OpenPort = OpenPort0,
        MakeDirectoryPort = MakeDirectoryPort0,
        ExistsPort = ExistsPort0,
        RemovePort = RemovePort0,
        SDClusterCountLower = SDClusterCount00,
        SDClusterCountUpper = SDClusterCount10,
        SDVolumeSectorCountLower = SDVolumeSectorCount00,
        SDVolumeSectorCountUpper = SDVolumeSectorCount10,
        SDBytesPerSector = SDBytesPerSector0,
        NumberOfOpenFiles = NumberOfOpenFiles0,
        MaximumNumberOfOpenFiles = MaximumNumberOfOpenFiles0,
        ErrorCode = ErrorCode0,
        MakeMissingParentDirectories = MakeMissingParentDirectories0,
        OpenReadWrite = OpenReadWrite0,
        OpenReadOnly = OpenReadOnly0,
        OpenWriteOnly = OpenWriteOnly0,
        CreateFileIfMissing = CreateFileIfMissing0,
        ClearFileContentsOnOpen = ClearFileContentsOnOpen0,
        FilePermissions = FilePermissions0,
        MountCTL = MountCTL0,
        // we ignore the upper half of the register but reserve it to make sure
    };
}
void
ConfigurationSpace::write(uint8_t targetPage, uint8_t offset, LoadStoreStyle lss, SplitWord16 value) noexcept {
    if (targetPage >= static_cast<uint8_t>(SDFilesStart >> 8) && targetPage <= static_cast<uint8_t>(SDFilesEnd >> 8)) {
        sdfileWrite(targetPage - static_cast<uint8_t>(SDFilesStart >> 8), offset, lss, value);
    } else {
        switch (targetPage) {
            case static_cast<uint8_t>(ConfigurationSpaceCtl0 >> 8):
// do nothing
                break;
            case static_cast<uint8_t>(ConsoleCtl >> 8):
                writeSerial(offset, lss, value);
                break;
            case static_cast<uint8_t>(SDCtl >> 8):
                sdctlWrite(offset, lss, value);
                break;
#if 0
                case static_cast<uint8_t>(RTCCtl >> 8):
                    break;
                case static_cast<uint8_t>(DisplayCtl >> 8):
                    break;
                case static_cast<uint8_t>(SeesawCtl >> 8):
                    break;
#endif
            default:
                break;
        }
    }
}
uint16_t
ConfigurationSpace::read(uint8_t targetPage, uint8_t offset, LoadStoreStyle lss) noexcept {
    // force override the default implementation
    if (targetPage >= static_cast<uint8_t>(SDFilesStart >> 8) && targetPage <= static_cast<uint8_t>(SDFilesEnd >> 8)) {
        return sdfileRead(targetPage - static_cast<uint8_t>(SDFilesStart >> 8), offset, lss);
    } else {
        switch (targetPage) {
            case static_cast<uint8_t>(ConfigurationSpaceCtl0 >> 8):
                return readIOConfigurationSpace0(offset);
            case static_cast<uint8_t>(ConsoleCtl >> 8):
                return readSerial(offset, lss);
            case static_cast<uint8_t>(SDCtl >> 8):
                return sdctlRead(offset, lss);
#if 0
                case static_cast<uint8_t>(RTCCtl >> 8):
                    return 0;
                case static_cast<uint8_t>(DisplayCtl >> 8):
                    return 0;
                case static_cast<uint8_t>(SeesawCtl >> 8):
                    return 0;
#endif
            default:
                return 0;
        }
    }
}
uint16_t ConfigurationSpace::sdfileRead(uint8_t index, uint8_t offset, LoadStoreStyle lss) noexcept {
    return files_[index].read(offset, lss);
}
void ConfigurationSpace::sdfileWrite(uint8_t index, uint8_t offset, LoadStoreStyle lss, SplitWord16 value) {
    files_[index].write(offset, lss, value);
}
bool ConfigurationSpace::tryMakeDirectory(bool makeMissingParents) noexcept { return SD.mkdir(sdCardPath_, makeMissingParents); }
bool ConfigurationSpace::exists() noexcept { return SD.exists(sdCardPath_); }
bool ConfigurationSpace::remove() noexcept { return SD.remove(sdCardPath_); }
uint16_t ConfigurationSpace::sdctlRead(uint8_t offset, LoadStoreStyle lss) noexcept {
    if (offset < 80) {
        if (auto result = SplitWord16(reinterpret_cast<uint16_t*>(sdCardPath_)[offset >> 1]); lss == LoadStoreStyle::Upper8) {
            return result.bytes[1];
        } else if (lss == LoadStoreStyle::Lower8) {
            return result.bytes[0];
        } else {
            return result.getWholeValue();
        }
    } else {
        using T = SDCardFileSystemRegisters;
        switch (static_cast<T>(offset)) {
            case T::OpenPort:
                return tryOpenFile();
            case T::MakeDirectoryPort:
                return tryMakeDirectory(makeMissingParentDirectories_);
            case T::ExistsPort:
                return exists();
            case T::RemovePort:
                return remove();
            case T::SDClusterCountLower:
                return clusterCount_.halves[0];
            case T::SDClusterCountUpper:
                return clusterCount_.halves[1];
            case T::SDVolumeSectorCountLower:
                return volumeSectorCount_.halves[0];
            case T::SDVolumeSectorCountUpper:
                return volumeSectorCount_.halves[1];
            case T::SDBytesPerSector:
                return bytesPerSector_;
            case T::MaximumNumberOfOpenFiles:
                return MaximumNumberOfOpenFiles;
            case T::NumberOfOpenFiles:
                return numberOfOpenFiles_;
            case T::MakeMissingParentDirectories:
                return makeMissingParentDirectories_;
            case T::FilePermissions:
                return filePermissions_;
            case T::MountCTL:
                return cardMounted_ ? 0xFFFF : 0;
            default:
                return 0;
        }
    }
}
void
ConfigurationSpace::sdctlWrite(uint8_t offset, LoadStoreStyle lss, SplitWord16 value) noexcept {
    if (offset < 80) {
        if (lss == LoadStoreStyle::Upper8) {
            sdCardPath_[offset + 1] = static_cast<char>(value.bytes[1]);
        } else if (lss == LoadStoreStyle::Lower8) {
            sdCardPath_[offset] = static_cast<char>(value.bytes[0]);
        } else {
            sdCardPath_[offset] = static_cast<char>(value.bytes[0]);
            sdCardPath_[offset+1] = static_cast<char>(value.bytes[1]);
        }
    } else {
        using T = SDCardFileSystemRegisters;
        switch (static_cast<T>(offset)) {
            case T::MakeMissingParentDirectories:
                makeMissingParentDirectories_ = value.getWholeValue() != 0;
                break;
            case T::FilePermissions:
                filePermissions_ = value.getWholeValue();
                break;
            case T::OpenReadWrite:
                if (value.getWholeValue() != 0) {
                    filePermissions_ |= O_RDWR;
                }
                break;
            case T::OpenReadOnly:
                if (value.getWholeValue() != 0) {
                    filePermissions_ |= O_RDONLY;
                }
                break;
            case T::OpenWriteOnly:
                if (value.getWholeValue() != 0) {
                    filePermissions_ |= O_WRITE;
                }
                break;
            case T::CreateFileIfMissing:
                if (value.getWholeValue() != 0) {
                    filePermissions_ |= O_CREAT;
                }
                break;
            case T::ClearFileContentsOnOpen:
                if (value.getWholeValue() != 0) {
                    filePermissions_ |= O_TRUNC;
                }
                break;
            case T::MountCTL:
                // 0 means unmount,
                // 1 means mount
                // other values are ignored
                if (value.getWholeValue() == 0) {
                    // unmount
                    unmountSDCard();
                } else if (value.getWholeValue() == 1) {
                    // mount
                    (void)tryMountSDCard();
                }
                break;
            default:
                break;
        }
    }
}
void
ConfigurationSpace::unmountSDCard() noexcept {
    if (cardMounted_) {
        // first close all open files
        for (auto &file: files_) {
            if (file.isOpen()) {
                // flush everything in progress
                file.flush();
                file.close();
            }
        }
        // according to my research this should be enough
        cardMounted_ = false;
    }
}
bool
ConfigurationSpace::tryMountSDCard() noexcept {
    if (!cardMounted_) {
        cardMounted_ = SD.begin(static_cast<int>(i960Pinout::SD_EN));
    }
    return cardMounted_;
}
uint16_t
ConfigurationSpace::findFreeFile() noexcept {
    for (uint16_t i = 0; i < MaximumNumberOfOpenFiles; ++i) {
        if (!files_[i].isOpen()) {
            return i;
        }
    }
    return 0xFFFF;
}
uint16_t
ConfigurationSpace::tryOpenFile() noexcept {
    if (numberOfOpenFiles_ < MaximumNumberOfOpenFiles) {
        // when we open a new file we have to make sure that we are less than the number of open files
        // But we also need to keep track of proper indexes as well. This is a two layer process
        auto newId = findFreeFile();
        auto& targetFile = files_[newId];
        if (targetFile.open(sdCardPath_, filePermissions_)) {
            ++numberOfOpenFiles_;
            return newId;
        } else {
            /// @todo set appropriate error condition for bad file open
        }
    } else {
        /// @todo set appropriate error condition for too many open files
    }
    return -1;
}

uint16_t ConfigurationSpace::readIOConfigurationSpace0(uint8_t offset) noexcept {
    //auto lower = pgm_read_byte_far(pgm_get_a);
    //auto upper = pgm_read_byte_far(address + offset + 1);
    //SplitWord16 result{0, upper};
    if (size_t realOffset = (offset >> 1); realOffset >= NumWordsCS0 ) {
        return 0;
    } else {
        return pgm_read_word_near(reinterpret_cast<const uint16_t*>(AddressTable) + realOffset);
    }
}
uint16_t ConfigurationSpace::readSerial(uint8_t offset, LoadStoreStyle ) noexcept {
    switch (static_cast<SerialRegisters>(offset)) {
        case SerialRegisters::ConsoleIO:
            return Serial.read();
        default:
            return 0;
    }
}

void ConfigurationSpace::writeSerial(uint8_t offset, LoadStoreStyle, SplitWord16 value) noexcept {
    switch (static_cast<SerialRegisters>(offset)) {
        case SerialRegisters::TriggerInterrupt:
            ProcessorInterface::triggerInt0();
            break;
        case SerialRegisters::ConsoleFlush:
            Serial.flush();
            break;
        case SerialRegisters::ConsoleIO:
            Serial.write(static_cast<char>(value.getWholeValue()));
            break;
        case SerialRegisters::AddressDebuggingFlag:
            break;
        default:
            break;
    }
}
void
ConfigurationSpace::begin() noexcept {
    // console always comes first
    if constexpr (DisplayBootupInformation) {
        Serial.println(F("CONSOLE UP!"));
    }
    //TheDisplayInterface::begin();
    if (!initialized_) {
        while (!tryMountSDCard()) {
            Serial.println(F("SD CARD INIT FAILED...WILL RETRY SOON"));
            delay(1000);
        }
        if constexpr (DisplayBootupInformation) {
            Serial.println(F("SD CARD UP!"));
        }
        clusterCount_ = SplitWord32(SD.clusterCount());
        volumeSectorCount_ = SplitWord32(SD.clusterCount() / SD.sectorsPerCluster());
        bytesPerSector_ = SD.bytesPerCluster() / SD.sectorsPerCluster();
        initialized_ = true;
    }
    //TheRTCInterface::begin();
}
