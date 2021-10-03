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
#include <SdFat.h>
#ifdef USE_DAZZLER
#include <GD2.h>
#endif
extern SdFat SD;
class CoreChipsetFeatures /* : public IOSpaceThing */ {
public:
    static constexpr Address IOBaseAddress = 0xFE00'0000;
    static constexpr Address RegisterPage0BaseAddress = IOBaseAddress;
    static constexpr Address SDCardInterfaceBaseAddress = RegisterPage0BaseAddress + 0x100;
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
        FourByteEntry(ConsoleTimeout),
        TwoByteEntry(ConsoleRXBufferSize),
        TwoByteEntry(ConsoleTXBufferSize),
        FourByteEntry(ChipsetClockSpeed),
        TwoByteEntry(CacheLineCount),
        TwoByteEntry(CacheLineSize),
        TwoByteEntry(NumberOfCacheWays),
        FourByteEntry(Unused0),
        FourByteEntry(Unused1),
        TwoByteEntry(Unused2),
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
        ConsoleTimeoutLower = ConsoleTimeout00,
        ConsoleTimeoutUpper = ConsoleTimeout10,
        ConsoleRXBufferSize = ConsoleRXBufferSize0,
        ConsoleTXBufferSize = ConsoleTXBufferSize0,
        ChipsetClockSpeedLower = ChipsetClockSpeed00,
        ChipsetClockSpeedUpper = ChipsetClockSpeed10,
        CacheLineCount = CacheLineCount0,
        CacheLineSize = CacheLineSize0,
        NumberOfCacheWays = NumberOfCacheWays0,
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
        // we ignore the upper half of the register but reserve it to make sure
    };

public:
    CoreChipsetFeatures() = delete;
    ~CoreChipsetFeatures() = delete;
    CoreChipsetFeatures(const CoreChipsetFeatures&) = delete;
    CoreChipsetFeatures(CoreChipsetFeatures&&) = delete;
    CoreChipsetFeatures& operator=(const CoreChipsetFeatures&) = delete;
    CoreChipsetFeatures& operator=(CoreChipsetFeatures&&) = delete;
    static void begin() noexcept {
        while (!SD.begin(static_cast<int>(i960Pinout::SD_EN))) {
            Serial.println(F("SD CARD INIT FAILED...WILL RETRY SOON"));
            delay(1000);
        }
        Serial.println(F("SD CARD UP!"));
        timeoutCopy_ = SplitWord32(Serial.getTimeout());
        clusterCount_ = SplitWord32(SD.clusterCount());
        volumeSectorCount_ = SplitWord32(SD.volumeSectorCount());
        bytesPerSector_ = SD.bytesPerSector();
#ifdef USE_DAZZLER
        GD.begin(0);
#endif
#define X(thing, base, type) Serial.print(F("Address of " #thing ": 0x")); \
        Serial.print(base + static_cast<byte>(type :: thing ), HEX); \
        Serial.print(F(", ("));                                \
        Serial.print(base + static_cast<byte>(type ::thing));        \
        Serial.println(F(")"));
#define P0(thing) X(thing, RegisterPage0BaseAddress, Registers)
#define P1(thing) X(thing, SDCardInterfaceBaseAddress, SDCardFileSystemRegisters)

        P0(ConsoleIO);
        P0(ConsoleFlush);
        P0(ConsoleTimeoutLower);
        P0(ConsoleTimeoutUpper);
        P0(ConsoleRXBufferSize);
        P0(ConsoleTXBufferSize);
        P0(ChipsetClockSpeedLower);
        P0(ChipsetClockSpeedUpper);
        P0(CacheLineCount );
        P0(CacheLineSize );
        P0(NumberOfCacheWays );
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
#undef P0
#undef P1
#undef X
    }
private:
    static uint16_t handleSecondPageRegisterReads(uint8_t offset, LoadStoreStyle) noexcept {
        using T = SDCardFileSystemRegisters;
        switch (static_cast<T>(offset)) {
#define OneByteEntry(index) case T :: index : return sdCardPath_[static_cast<byte>(T :: index)];
#define TwoByteEntry(Prefix) OneByteEntry(Prefix ## 0) OneByteEntry(Prefix ## 1)
#define FourByteEntry(Prefix) TwoByteEntry(Prefix ## 0) TwoByteEntry(Prefix ## 1)
#define EightByteEntry(Prefix) FourByteEntry(Prefix ## 0) FourByteEntry(Prefix ## 1)
#define SixteenByteEntry(Prefix) EightByteEntry(Prefix ## 0) EightByteEntry(Prefix ## 1)
            SixteenByteEntry(Path0);
            SixteenByteEntry(Path1);
            SixteenByteEntry(Path2);
            SixteenByteEntry(Path3);
            SixteenByteEntry(Path4);
#undef SixteenByteEntry
#undef EightByteEntry
#undef FourByteEntry
#undef TwoByteEntry
#undef OneByteEntry
            case T::OpenPort:
                /// @todo implement open support
                return -1;
            case T::MakeDirectoryPort:
                /// @todo implement mkdir support
                return -1;
            case T::ExistsPort:
                /// @todo implement exists support
                return 0;
            case T::RemovePort:
                /// @todo implement remove support
                return 0;
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
                return 32;
            case T::NumberOfOpenFiles:
                return numberOfOpenFiles_;
            default:
                return 0;
        }
    }
    static uint16_t handleFirstPageRegisterReads(uint8_t offset, LoadStoreStyle) noexcept {
        switch (static_cast<Registers>(offset)) {
            case Registers::ConsoleIO:
                return Serial.read();
            case Registers::ConsoleTimeoutLower:
                return timeoutCopy_.halves[0];
            case Registers::ConsoleTimeoutUpper:
                return timeoutCopy_.halves[1];
            case Registers::ConsoleRXBufferSize:
                return SERIAL_RX_BUFFER_SIZE;
            case Registers::ConsoleTXBufferSize:
                return SERIAL_TX_BUFFER_SIZE;
            case Registers::ChipsetClockSpeedLower:
                return clockSpeedHolder.halves[0];
            case Registers::ChipsetClockSpeedUpper:
                return clockSpeedHolder.halves[1];
            case Registers::CacheLineCount:
                return 256;
            case Registers::CacheLineSize:
                return 16;
            case Registers::NumberOfCacheWays:
                return 2;
            case Registers::AddressDebuggingFlag:
                return static_cast<uint16_t>(enableAddressDebugging_);
            default:
                return 0;
        }
    }
    static void handleSecondPageRegisterWrites(uint8_t offset, LoadStoreStyle, SplitWord16 value) noexcept {
        using T = SDCardFileSystemRegisters;
        switch (static_cast<T>(offset)) {
#define OneByteEntry(index, offset) case T :: index : sdCardPath_[static_cast<byte>(T :: index)] = value.bytes[offset]; break;
#define TwoByteEntry(Prefix) OneByteEntry(Prefix ## 0, 0) OneByteEntry(Prefix ## 1, 1)
#define FourByteEntry(Prefix) TwoByteEntry(Prefix ## 0) TwoByteEntry(Prefix ## 1)
#define EightByteEntry(Prefix) FourByteEntry(Prefix ## 0) FourByteEntry(Prefix ## 1)
#define SixteenByteEntry(Prefix) EightByteEntry(Prefix ## 0) EightByteEntry(Prefix ## 1)
            SixteenByteEntry(Path0);
            SixteenByteEntry(Path1);
            SixteenByteEntry(Path2);
            SixteenByteEntry(Path3);
            SixteenByteEntry(Path4);
#undef SixteenByteEntry
#undef EightByteEntry
#undef FourByteEntry
#undef TwoByteEntry
#undef OneByteEntry
            default:
                break;
        }
    }
    static void handleFirstPageRegisterWrites(uint8_t offset, LoadStoreStyle, SplitWord16 value) noexcept {
        bool updateTimeout = false;
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
            case Registers::ConsoleTimeoutLower:
                timeoutCopy_.halves[0] = value.getWholeValue();
                updateTimeout = true;
                break;
            case Registers::ConsoleTimeoutUpper:
                timeoutCopy_.halves[1] = value.getWholeValue();
                updateTimeout = true;
                break;
            case Registers::AddressDebuggingFlag:
                enableAddressDebugging_ = (value.getWholeValue() != 0);
            default:
                break;
        }
        if (updateTimeout) {
            Serial.setTimeout(timeoutCopy_.getWholeValue());
        }
    }
public:
    static uint16_t read(uint8_t targetPage, uint8_t offset, LoadStoreStyle lss) noexcept {
        // force override the default implementation
        switch (targetPage) {
            case 0: return handleFirstPageRegisterReads(offset, lss);
            case 1: return handleSecondPageRegisterReads(offset, lss);
            default: return 0;
        }
    }
    static void write(uint8_t targetPage, uint8_t offset, LoadStoreStyle lss, SplitWord16 value) noexcept {
        switch (targetPage) {
            case 0: handleFirstPageRegisterWrites(offset, lss, value); break;
            case 1: handleSecondPageRegisterWrites(offset, lss, value); break;
            default: break;
        }
    }
    static bool addressDebuggingEnabled() noexcept { return enableAddressDebugging_; }
private:
    using PageWriteFunction = void (*)(uint8_t, LoadStoreStyle, SplitWord16) noexcept;
    using PageReadFunction = uint16_t (*)(uint8_t, LoadStoreStyle) noexcept;
    static inline SplitWord32 timeoutCopy_{0};
    static inline SplitWord32 clusterCount_ {0};
    static inline SplitWord32 volumeSectorCount_ {0};
    static inline uint16_t bytesPerSector_ = 0;
    static inline uint16_t numberOfOpenFiles_ = 0;
    // 257th char is always zero and not accessible, prevent crap from going beyond the cache
    static constexpr SplitWord32 clockSpeedHolder{TargetBoard::getCPUFrequency()};
    static inline bool enableAddressDebugging_ = false;
    static inline char sdCardPath_[81] = { 0 };
};
#endif //I960SXCHIPSET_CORECHIPSETFEATURES_H
