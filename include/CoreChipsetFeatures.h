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
        FourByteEntry(SDClusterCount),
        FourByteEntry(SDVolumeSectorCount),
        TwoByteEntry(SDBytesPerSector),
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
        SDClusterCountLower = SDClusterCount00,
        SDClusterCountUpper = SDClusterCount10,
        SDVolumeSectorCountLower = SDVolumeSectorCount00,
        SDVolumeSectorCountUpper = SDVolumeSectorCount10,
        SDBytesPerSector = SDBytesPerSector0,
        TriggerInterrupt = TriggerInterrupt0,
        AddressDebuggingFlag = AddressDebuggingFlag00,
        // we ignore the upper half of the register but reserve it to make sure
    };
    static_assert(static_cast<int>(Registers::End) < 0x100);
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
#define X(thing) Serial.print(F("Address of " #thing ": 0x")); \
        Serial.print(RegisterPage0BaseAddress + static_cast<byte>(Registers:: thing ), HEX); \
        Serial.print(F(", ("));                                \
        Serial.print(RegisterPage0BaseAddress + static_cast<byte>(Registers::thing));        \
        Serial.println(F(")"));

        X(ConsoleIO);
        X(ConsoleFlush);
        X(ConsoleTimeoutLower);
        X(ConsoleTimeoutUpper);
        X(ConsoleRXBufferSize);
        X(ConsoleTXBufferSize);
        X(ChipsetClockSpeedLower);
        X(ChipsetClockSpeedUpper);
        X(CacheLineCount );
        X(CacheLineSize );
        X(NumberOfCacheWays );
        X(SDClusterCountLower );
        X(SDClusterCountUpper );
        X(SDVolumeSectorCountLower );
        X(SDVolumeSectorCountUpper );
        X(SDBytesPerSector );
        X(TriggerInterrupt );
        X(AddressDebuggingFlag );
#undef X
    }
private:
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
            case Registers::SDClusterCountLower:
                return clusterCount_.halves[0];
            case Registers::SDClusterCountUpper:
                return clusterCount_.halves[1];
            case Registers::SDVolumeSectorCountLower:
                return volumeSectorCount_.halves[0];
            case Registers::SDVolumeSectorCountUpper:
                return volumeSectorCount_.halves[1];
            case Registers::SDBytesPerSector:
                return bytesPerSector_;
            case Registers::AddressDebuggingFlag:
                return static_cast<uint16_t>(enableAddressDebugging_);
            default:
                return 0;
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
            default: return 0;
        }
    }
    static void write(uint8_t targetPage, uint8_t offset, LoadStoreStyle lss, SplitWord16 value) noexcept {
        switch (targetPage) {
            case 0: handleFirstPageRegisterWrites(offset, lss, value); break;
            default: break;
        }
    }
    static bool addressDebuggingEnabled() noexcept { return enableAddressDebugging_; }
private:
    static inline SplitWord32 timeoutCopy_{0};
    static inline SplitWord32 clusterCount_ {0};
    static inline SplitWord32 volumeSectorCount_ {0};
    static inline uint16_t bytesPerSector_ = 0;
    // 257th char is always zero and not accessible, prevent crap from going beyond the cache
    static constexpr SplitWord32 clockSpeedHolder{TargetBoard::getCPUFrequency()};
    static inline bool enableAddressDebugging_ = false;
};
#endif //I960SXCHIPSET_CORECHIPSETFEATURES_H
