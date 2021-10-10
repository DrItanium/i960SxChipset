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
    static constexpr Address SDCardInterfaceBaseAddressEnd = SDCardInterfaceBaseAddress+ 0x100;
    static constexpr Address DisplayShieldBaseAddress = SDCardInterfaceBaseAddressEnd;
    static constexpr Address DisplayShieldBaseAddressEnd = DisplayShieldBaseAddress + 0x100;
    static constexpr Address ST7735DisplayBaseAddress = DisplayShieldBaseAddressEnd;
    static constexpr Address ST7735DisplayBaseAddressEnd = ST7735DisplayBaseAddress + 0x100;
    // we have a bunch of pages in here that are useful :)
    static constexpr Address SDCardFileInterfaceBlockBaseAddress = IOConfigurationSpaceEnd + (16 * 0x100);
    static constexpr Address SDCardFileInterfaceBlockEndAddress = SDCardFileInterfaceBlockBaseAddress + (MaximumNumberOfOpenFiles * 0x100);
    enum class IOConfigurationSpace0Registers : uint8_t {
#define TwoByteEntry(Prefix) Prefix ## 0, Prefix ## 1
#define FourByteEntry(Prefix) \
        TwoByteEntry(Prefix ## 0), \
        TwoByteEntry(Prefix ## 1)

        FourByteEntry(Serial0BaseAddress),
        FourByteEntry(SDCardInterfaceBaseAddress),
        FourByteEntry(SDCardFileBlock0BaseAddress),
        FourByteEntry(DisplayShieldBaseAddress),
        FourByteEntry(ST7735DisplayBaseAddress),
#undef FourByteEntry
#undef TwoByteEntry
        End,
        Serial0BaseAddressLower = Serial0BaseAddress00,
        Serial0BaseAddressUpper = Serial0BaseAddress10,
        SDCardInterfaceBaseAddressLower = SDCardInterfaceBaseAddress00,
        SDCardInterfaceBaseAddressUpper = SDCardInterfaceBaseAddress10,
        SDCardFileBlock0BaseAddressLower = SDCardFileBlock0BaseAddress00,
        SDCardFileBlock0BaseAddressUpper = SDCardFileBlock0BaseAddress10,
        DisplayShieldBaseAddressLower = DisplayShieldBaseAddress00,
        DisplayShieldBaseAddressUpper = DisplayShieldBaseAddress10,
        ST7735DisplayBaseAddressLower = ST7735DisplayBaseAddress00,
        ST7735DisplayBaseAddressUpper = ST7735DisplayBaseAddress10,
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
    static void begin() noexcept;
private:
    static uint16_t findFreeFile() noexcept;
    static uint16_t tryOpenFile() noexcept;
    static bool tryMakeDirectory(bool makeMissingParents = false) noexcept;
    static bool exists() noexcept;
    static bool remove() noexcept;
    static uint16_t handleSecondPageRegisterReads(uint8_t offset, LoadStoreStyle lss) noexcept;
    static uint16_t handleFirstPageRegisterReads(uint8_t offset, LoadStoreStyle) noexcept;
    static void handleSecondPageRegisterWrites(uint8_t offset, LoadStoreStyle lss, SplitWord16 value) noexcept;
    static void handleDisplayShieldWrites(uint8_t offset, LoadStoreStyle, SplitWord16 value) noexcept;
    static uint16_t handleDisplayShieldReads(uint8_t offset, LoadStoreStyle) noexcept;
    static void handleFirstPageRegisterWrites(uint8_t offset, LoadStoreStyle, SplitWord16 value) noexcept;
    static uint16_t readIOConfigurationSpace0(uint8_t offset, LoadStoreStyle) noexcept;
public:
    [[nodiscard]] static uint16_t read(uint8_t targetPage, uint8_t offset, LoadStoreStyle lss) noexcept;
    static void write(uint8_t targetPage, uint8_t offset, LoadStoreStyle lss, SplitWord16 value) noexcept;
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
