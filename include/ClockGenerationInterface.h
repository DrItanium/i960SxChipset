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
// Created by jwscoggins on 10/17/21.
//

#ifndef SXCHIPSET_CLOCKGENERATIONINTERFACE_H
#define SXCHIPSET_CLOCKGENERATIONINTERFACE_H
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SI5351.h>

#include "Pinout.h"

template<Address baseAddress>
class ClockGenerationInterface {
public:
    static constexpr auto StartAddress = baseAddress;
    static constexpr SplitWord32 StartAddressSplit { StartAddress };
    static constexpr auto EndAddress = StartAddress + 0x100;
    static constexpr SplitWord32 EndAddressSplit{EndAddress};
    static constexpr auto StartPage = StartAddressSplit.getTargetPage();
    static constexpr auto EndPage = EndAddressSplit.getTargetPage();
    static constexpr auto SectionID = StartAddressSplit.getMostSignificantByte();
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
        TwoByteEntry(Available),
        TwoByteEntry(Enable),
        /// @todo add PLL A configuration
        /// @todo add PLL B configuration
        /// @todo add Channel 0 Multisynth
        /// @todo add Channel 0 RDIV
        /// @todo add Channel 1 Multisynth
        /// @todo add Channel 1 RDIV
        /// @todo add Channel 2 Multisynth
        /// @todo add Channel 2 RDIV
#undef SixteenByteEntry
#undef TwelveByteEntry
#undef EightByteEntry
#undef FourByteEntry
#undef TwoByteEntry
        End,
        Available = Available0,
        Enable = Enable0,
    };
public:
    ClockGenerationInterface() = delete;
    ~ClockGenerationInterface() = delete;
    static constexpr bool respondsTo(byte targetPage) noexcept {
        return targetPage >= StartPage && targetPage < EndPage;
    }
    static void begin() noexcept {
        clockgenUp_ = clockgen_.begin() == ERROR_NONE;
        if (!clockgenUp_) {
            Serial.println(F("Could not bring up clock gen device, disabling"));
        } else {
            Serial.println(F("Found a clock generation device!"));
            clockgen_.enableOutputs(enabled_);
        }
    }
    static uint16_t read(uint8_t targetPage, uint8_t offset, LoadStoreStyle lss) noexcept {
        switch (static_cast<Registers>(offset)) {
            case Registers::Available:
                return static_cast<uint16_t>(clockgenUp_ ? 0xFFFF : 0);
            case Registers::Enable:
                return static_cast<uint16_t>(enabled_ ? 0xFFFF : 0);
            default:
                return 0;
        }
    }
    static void write(uint8_t targetPage, uint8_t offset, LoadStoreStyle lss, SplitWord16 value) noexcept {
        switch (static_cast<Registers>(offset)) {
            case Registers::Enable: {
                if (clockgenUp_) {
                    enabled_ = value.getWholeValue() != 0;
                    clockgen_.enableOutputs(enabled_);
                }
                break;
            }
            default:
                break;
        }
    }
private:
    static inline Adafruit_SI5351 clockgen_;
    static inline bool clockgenUp_ = false;
    static inline bool enabled_ = false;
};
#endif //SXCHIPSET_CLOCKGENERATIONINTERFACE_H
