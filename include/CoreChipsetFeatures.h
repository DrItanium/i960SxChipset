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
#include "MemoryThing.h"
#include "ProcessorSerializer.h"
#include "OpenFileHandle.h"
#include <SdFat.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_seesaw.h>
#include <Adafruit_ST7735.h>
#include <Adafruit_TFTShield18.h>
extern SdFat SD;
class CoreChipsetFeatures /* : public IOSpaceThing */ {
public:
    static constexpr auto MaximumNumberOfOpenFiles = 32;
    static constexpr Address IOBaseAddress = 0xFE00'0000;
    // each one of these 256 byte pages have a prescribed start and end
    static constexpr Address IOConfigurationSpaceStart = IOBaseAddress;
    static constexpr Address IOConfigurationSpaceEnd = IOConfigurationSpaceStart + (16 * 0x100);
    // then start our initial designs after this point
    static constexpr Address RegisterPage0BaseAddress = IOConfigurationSpaceEnd;
    static constexpr Address SDCardInterfaceBaseAddress = RegisterPage0BaseAddress + 0x100;
    static constexpr Address SDCardFileInterfaceBlockBaseAddress = SDCardInterfaceBaseAddress + 0x100;
    static constexpr Address SDCardFileInterfaceBlockEndAddress = SDCardFileInterfaceBlockBaseAddress + (MaximumNumberOfOpenFiles * 0x100);
    static constexpr Address DisplayShieldBaseAddress = SDCardFileInterfaceBlockEndAddress;
    static constexpr Address DisplayShieldBaseAddressEnd = DisplayShieldBaseAddress + 0x100;
    static constexpr Address ST7735DisplayBaseAddress = DisplayShieldBaseAddressEnd;
    static constexpr Address ST7735DisplayBaseAddressEnd = ST7735DisplayBaseAddress + 0x100;
    enum class IOConfigurationSpace0Registers : uint8_t {
#define TwoByteEntry(Prefix) Prefix ## 0, Prefix ## 1
#define FourByteEntry(Prefix) \
        TwoByteEntry(Prefix ## 0), \
        TwoByteEntry(Prefix ## 1), \
        Prefix ## Lower = Prefix ## 00 , \
        Prefix ## Upper = Prefix ## 10

        FourByteEntry(Serial0BaseAddress),
        FourByteEntry(SDCardInterfaceBaseAddress),
        FourByteEntry(SDCardFileBlock0BaseAddress),
        FourByteEntry(DisplayShieldBaseAddress),
        FourByteEntry(ST7735DisplayBaseAddress),
#undef FourByteEntry
#undef TwoByteEntry
        End,
    };
    enum class Registers : uint8_t {
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
    static_assert(static_cast<int>(Registers::End) < 0x100);
    enum class SDCardFileSystemRegisters : uint8_t {
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
        // we ignore the upper half of the register but reserve it to make sure
    };

    enum class DisplayShieldRegisters : uint8_t {
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
        FourByteEntry(RawButtons),
#undef SixteenByteEntry
#undef TwelveByteEntry
#undef EightByteEntry
#undef FourByteEntry
#undef TwoByteEntry
        End,
        Backlight = Backlight0,
        RawButtonsLower = RawButtons00,
        RawButtonsUpper = RawButtons10,
    };

public:
    CoreChipsetFeatures() = delete;
    ~CoreChipsetFeatures() = delete;
    CoreChipsetFeatures(const CoreChipsetFeatures&) = delete;
    CoreChipsetFeatures(CoreChipsetFeatures&&) = delete;
    CoreChipsetFeatures& operator=(const CoreChipsetFeatures&) = delete;
    CoreChipsetFeatures& operator=(CoreChipsetFeatures&&) = delete;
    static void begin() noexcept {
        pinMode(i960Pinout::TFT_CS, OUTPUT);
        pinMode(i960Pinout::SD_EN, OUTPUT);
        digitalWrite<i960Pinout::TFT_CS, HIGH>();
        digitalWrite<i960Pinout::SD_EN, HIGH>();
        Wire.begin();
        if (!displayShield_.begin()) {
           signalHaltState(F("display shield seesaw could not be initialized!")) ;
        }
        Serial.println(F("Display seesaw started"));
        Serial.print("Version: ");
        Serial.println(displayShield_.getVersion(), HEX);

        displayShield_.setBacklight(TFTSHIELD_BACKLIGHT_OFF);
        displayShield_.tftReset();
        tft.initR(INITR_BLACKTAB);

        Serial.println(F("TFT UP AND OK!"));
        tft.fillScreen(ST7735_CYAN);
        delay(1000);
        tft.fillScreen(ST7735_BLACK);
        while (!SD.begin(static_cast<int>(i960Pinout::SD_EN))) {
            Serial.println(F("SD CARD INIT FAILED...WILL RETRY SOON"));
            delay(1000);
        }
        Serial.println(F("SD CARD UP!"));
        timeoutCopy_ = SplitWord32(Serial.getTimeout());
        clusterCount_ = SplitWord32(SD.clusterCount());
        volumeSectorCount_ = SplitWord32(SD.volumeSectorCount());
        bytesPerSector_ = SD.bytesPerSector();
#define X(thing, base, type) Serial.print(F("Address of " #thing ": 0x")); \
        Serial.print(base + static_cast<byte>(type :: thing ), HEX); \
        Serial.print(F(", ("));                                \
        Serial.print(base + static_cast<byte>(type ::thing));        \
        Serial.println(F(")"));
#define P0(thing) X(thing, RegisterPage0BaseAddress, Registers)
#define P1(thing) X(thing, SDCardInterfaceBaseAddress, SDCardFileSystemRegisters)
#define P34(thing) X(thing, DisplayShieldBaseAddress, DisplayShieldRegisters)
        P0(ConsoleIO);
        P0(ConsoleFlush);
        P0(TriggerInterrupt );
        P0(AddressDebuggingFlag );
        P1(PathStart);
        P1(PathEnd);
        P1(OpenPort);
        P1(MakeDirectoryPort);
        P1(ExistsPort );
        P1(RemovePort );
        P1(SDClusterCountLower );
        P1(SDClusterCountUpper );
        P1(SDVolumeSectorCountLower );
        P1(SDVolumeSectorCountUpper );
        P1(SDBytesPerSector);
        P1(NumberOfOpenFiles);
        P1(ErrorCode);
        P1(MakeMissingParentDirectories);
        P1(OpenReadWrite);
        P34(Backlight);
        P34(RawButtonsLower);
        P34(RawButtonsUpper);
#undef P0
#undef P1
#undef P34
#undef X
        for (uint32_t i = SDCardFileInterfaceBlockBaseAddress, j = 0; i < SDCardFileInterfaceBlockEndAddress; i += 0x100, ++j) {
            Serial.print(F("File "));
            Serial.print(j);
            Serial.print(F(" @ 0x"));
            Serial.println(i, HEX);
        }
    }
private:
    static uint16_t findFreeFile() noexcept {
        for (uint16_t i = 0; i < MaximumNumberOfOpenFiles; ++i) {
            if (!files_[i].isOpen()) {
                return i;
            }
        }
        return 0xFFFF;
    }
    static uint16_t tryOpenFile() noexcept {
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
    static bool tryMakeDirectory(bool makeMissingParents = false) noexcept {
        return SD.mkdir(sdCardPath_, makeMissingParents);
    }
    static bool exists() noexcept {
        return SD.exists(sdCardPath_);
    }
    static bool remove() noexcept {
        return SD.remove(sdCardPath_);
    }
    static uint16_t handleSecondPageRegisterReads(uint8_t offset, LoadStoreStyle lss) noexcept {
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
                default:
                    return 0;
            }
        }
    }
    static uint16_t handleFirstPageRegisterReads(uint8_t offset, LoadStoreStyle) noexcept {
        switch (static_cast<Registers>(offset)) {
            case Registers::ConsoleIO:
                return Serial.read();
            case Registers::AddressDebuggingFlag:
                return static_cast<uint16_t>(enableAddressDebugging_);
            default:
                return 0;
        }
    }
    static void handleSecondPageRegisterWrites(uint8_t offset, LoadStoreStyle lss, SplitWord16 value) noexcept {
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
                default:
                    break;
            }
        }
    }
    static void handleDisplayShieldWrites(uint8_t offset, LoadStoreStyle, SplitWord16 value) noexcept {
        switch (static_cast<DisplayShieldRegisters>(offset))  {
            case DisplayShieldRegisters::Backlight:
                backlightIntensity_ = value.getWholeValue();
                displayShield_.setBacklight(backlightIntensity_);
                break;
            default: break;
        }
    }
    static uint16_t handleDisplayShieldReads(uint8_t offset, LoadStoreStyle) noexcept {
        switch (static_cast<DisplayShieldRegisters>(offset)) {
            case DisplayShieldRegisters::Backlight:
                return backlightIntensity_;
            case DisplayShieldRegisters::RawButtonsLower:
                rawButtons_.wholeValue_ = displayShield_.readButtons();
                return rawButtons_.halves[0];
            case DisplayShieldRegisters::RawButtonsUpper:
                return rawButtons_.halves[1];
            default:
                return 0;
        }
    }
    static void handleFirstPageRegisterWrites(uint8_t offset, LoadStoreStyle, SplitWord16 value) noexcept {
        switch (static_cast<Registers>(offset)) {
            case Registers::TriggerInterrupt:
                pulse<i960Pinout::Int0_>();
                break;
            case Registers::ConsoleFlush:
                Serial.flush();
                break;
            case Registers::ConsoleIO:
                Serial.write(static_cast<char>(value.getWholeValue()));
                break;
            case Registers::AddressDebuggingFlag:
                enableAddressDebugging_ = (value.getWholeValue() != 0);
                break;
            default:
                break;
        }
    }
    static uint16_t readIOConfigurationSpace0(uint8_t offset, LoadStoreStyle) noexcept {
            switch (static_cast<IOConfigurationSpace0Registers>(offset)) {
#define X(title, var) \
             case IOConfigurationSpace0Registers:: title ## Lower : return static_cast<uint16_t>(var); \
             case IOConfigurationSpace0Registers:: title ## Upper : return static_cast<uint16_t>(var >> 16)
                X(Serial0BaseAddress, RegisterPage0BaseAddress);
                X(SDCardInterfaceBaseAddress, SDCardInterfaceBaseAddress);
                X(SDCardFileBlock0BaseAddress, SDCardFileInterfaceBlockBaseAddress);
                X(DisplayShieldBaseAddress, DisplayShieldBaseAddress);
                X(ST7735DisplayBaseAddress, ST7735DisplayBaseAddress);
#undef X

                default: return 0; // zero is never an io page!
            }
    }
public:
    [[nodiscard]] static uint16_t read(uint8_t targetPage, uint8_t offset, LoadStoreStyle lss) noexcept {
        // force override the default implementation
        switch (targetPage) {
            case 0:
                return readIOConfigurationSpace0(offset, lss);
            case 1:
            case 2:
            case 3:
            case 4:
            case 5:
            case 6:
            case 7:
            case 8:
            case 9:
            case 10:
            case 11:
            case 12:
            case 13:
            case 14:
            case 15:
                return 0;
            case 16: return handleFirstPageRegisterReads(offset, lss);
            case 17: return handleSecondPageRegisterReads(offset, lss);
            case 18: case 19: case 20: case 21: case 22: case 23: case 24: case 25:
            case 26: case 27: case 28: case 29: case 30: case 31: case 32: case 33:
            case 34: case 35: case 36: case 37: case 38: case 39: case 40: case 41:
            case 42: case 43: case 44: case 45: case 46: case 47: case 48: case 49:
                return files_[targetPage - 18].read(offset, lss);
            case 50:
                return handleDisplayShieldReads(offset, lss);
            default: return 0;
        }
    }
    static void write(uint8_t targetPage, uint8_t offset, LoadStoreStyle lss, SplitWord16 value) noexcept {
        switch (targetPage) {
            // ignore writes to configuration space
            case 16: handleFirstPageRegisterWrites(offset, lss, value); break;
            case 17: handleSecondPageRegisterWrites(offset, lss, value); break;
            case 18: case 19: case 20: case 21: case 22: case 23: case 24: case 25:
            case 26: case 27: case 28: case 29: case 30: case 31: case 32: case 33:
            case 34: case 35: case 36: case 37: case 38: case 39: case 40: case 41:
            case 42: case 43: case 44: case 45: case 46: case 47: case 48: case 49:
                files_[targetPage - 18].write(offset,lss,value);
                break;
            case 50: // here as a placeholder of the next register group
                handleDisplayShieldWrites(offset, lss, value);
                break;
            default: break;
        }
    }
    static bool addressDebuggingEnabled() noexcept { return enableAddressDebugging_; }
private:
    static inline SplitWord32 timeoutCopy_{0};
    static inline SplitWord32 clusterCount_ {0};
    static inline SplitWord32 volumeSectorCount_ {0};
    static inline uint16_t bytesPerSector_ = 0;
    static inline uint16_t numberOfOpenFiles_ = 0;
    // 257th char is always zero and not accessible, prevent crap from going beyond the cache
    static constexpr SplitWord32 clockSpeedHolder{TargetBoard::getCPUFrequency()};
    static inline bool enableAddressDebugging_ = false;
    static inline char sdCardPath_[81] = { 0 };
    static inline OpenFileHandle files_[MaximumNumberOfOpenFiles];
    static inline bool makeMissingParentDirectories_ = false;
    static inline uint16_t filePermissions_ = 0;
    static inline Adafruit_TFTShield18 displayShield_;
    static inline Adafruit_ST7735 tft{static_cast<int>(i960Pinout::TFT_CS),
                                      static_cast<int>(i960Pinout::TFT_DC),
                                      -1};
    static inline uint16_t backlightIntensity_ = 0;
    static inline SplitWord32 rawButtons_{0};
};
#endif //I960SXCHIPSET_CORECHIPSETFEATURES_H
