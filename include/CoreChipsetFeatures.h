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
extern SdFat SD;
class CoreChipsetFeatures /* : public IOSpaceThing */ {
public:
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
        timeoutCopy_ = SplitWord32(Serial.getTimeout());
        clusterCount_ = SplitWord32(SD.clusterCount());
        volumeSectorCount_ = SplitWord32(SD.volumeSectorCount());
        bytesPerSector_ = SD.bytesPerSector();
        for (auto& sc : stringCache_) {
            sc = SplitWord16{0};
        }
    }
    static uint16_t read(uint8_t targetPage, uint8_t offset, LoadStoreStyle lss) noexcept {
        static constexpr SplitWord32 clockSpeedHolder{TargetBoard::getCPUFrequency()};
        // force override the default implementation
        if (targetPage == 0) {
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
                    return TargetBoard::numberOfCacheLines();
                case Registers::CacheLineSize:
                    return TargetBoard::cacheLineSize();
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

                default:
                    return 0;
            }
        } else if (targetPage == 0x01) {
            // we are in the string cache so just return the value found at the given offset
            const auto& targetEntry = stringCache_[offset >> 1];
            switch (lss) {
                case LoadStoreStyle::Full16:
                    return targetEntry.getWholeValue();
                case LoadStoreStyle::Upper8:
                    return SplitWord16{0, targetEntry.bytes[1]}.getWholeValue();
                case LoadStoreStyle::Lower8:
                    return static_cast<uint16_t>(targetEntry.bytes[0]);
                default:
                    return 0;
            }
        } else {
            return 0;
        }
    }
    static void write(uint8_t targetPage, uint8_t offset, LoadStoreStyle lss, const SplitWord16& value) noexcept {

        if (targetPage == 0) {
            bool updateTimeout = false;
            switch (static_cast<Registers>(offset)) {
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
                default:
                    break;
            }
            if (updateTimeout) {
                Serial.setTimeout(timeoutCopy_.getWholeValue());
            }
        } else if (targetPage == 0x01) {
            auto& targetEntry = stringCache_[offset >> 1];
            switch (lss) {
                case LoadStoreStyle::Full16:
                    targetEntry.wholeValue_ = value.getWholeValue();
                    break;
                case LoadStoreStyle::Upper8:
                    targetEntry.bytes[1] = value.bytes[1];
                    break;
                case LoadStoreStyle::Lower8:
                    targetEntry.bytes[0] = value.bytes[0];
                    break;
                default:
                    break;
            }
        }
    }
private:
    static inline SplitWord32 timeoutCopy_{0};
    static inline SplitWord32 clusterCount_ {0};
    static inline SplitWord32 volumeSectorCount_ {0};
    static inline uint16_t bytesPerSector_ = 0;
    // 257th char is always zero and not accessible, prevent crap from going beyond the cache
    static inline SplitWord16 stringCache_[258 / sizeof(SplitWord16)];
};
#endif //I960SXCHIPSET_CORECHIPSETFEATURES_H
