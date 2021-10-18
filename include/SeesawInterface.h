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

#ifndef SXCHIPSET_SEESAWINTERFACE_H
#define SXCHIPSET_SEESAWINTERFACE_H
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_seesaw.h>

#include "Pinout.h"

template<Address baseAddress>
class SeesawInterface {
public:
    static constexpr Address Count = 1;
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
        TwoByteEntry(Reset),
        FourByteEntry(Version),
        FourByteEntry(Options),
        TwoByteEntry(I2CAddress),
        TwoByteEntry(EEPROMAddress),
        TwoByteEntry(EEPROMValue),
        TwoByteEntry(EEPROMAddressAutoIncrement),
        /// @todo implement support for the many pins and their corresponding abilities
#undef SixteenByteEntry
#undef TwelveByteEntry
#undef EightByteEntry
#undef FourByteEntry
#undef TwoByteEntry
        End,
        Available = Available0,
        Reset = Reset0,
        I2CAddress = I2CAddress0,
        EEPROMAddress = EEPROMAddress0,
        EEPROMValue = EEPROMValue0,
        EEPROMAddressAutoIncrement = EEPROMAddressAutoIncrement0,
        VersionLower = Version00,
        VersionUpper = Version10,
        OptionsLower = Options00,
        OptionsUpper = Options10,

    };
public:
    SeesawInterface() = delete;
    ~SeesawInterface() = delete;
    static constexpr bool respondsTo(byte targetPage) noexcept {
        return targetPage >= StartPage && targetPage < EndPage;
    }
    static void begin() noexcept {
        initialized_ = seesaw0_.begin();
        if (!initialized_) {
            Serial.println(F("SEESAW0 NOT FOUND! DISABLING!"));
        } else {
            Serial.println(F("SEESAW 0 FOUND!"));
        }

    }
    static uint16_t read(uint8_t, uint8_t offset, LoadStoreStyle) noexcept {
        switch (static_cast<Registers>(offset)) {
            case Registers::Available:
                return static_cast<uint16_t>(initialized_ ? 0xFFFF : 0);
            case Registers::I2CAddress:
                return static_cast<uint16_t>(seesaw0_.getI2CAddr());
            case Registers::EEPROMAddress:
                return static_cast<uint16_t>(eepromAddress_);
            case Registers::EEPROMAddressAutoIncrement:
                return static_cast<uint16_t>(eepromAutoIncrement_);
            case Registers::EEPROMValue:
                return static_cast<uint16_t>(seesaw0_.EEPROMRead8(eepromAddress_));
            case Registers::VersionLower:
                return static_cast<uint16_t>(seesaw0_.getVersion());
            case Registers::VersionUpper:
                return static_cast<uint16_t>(seesaw0_.getVersion() >> 16);
            case Registers::OptionsLower:
                return static_cast<uint16_t>(seesaw0_.getOptions());
            case Registers::OptionsUpper:
                return static_cast<uint16_t>(seesaw0_.getOptions() >> 16);
            default:
                return 0;
        }
    }
    static void write(uint8_t targetPage, uint8_t offset, LoadStoreStyle lss, SplitWord16 value) noexcept {
    }
private:
    static inline Adafruit_seesaw seesaw0_;
    static inline bool initialized_ = false;
    static inline bool eepromAutoIncrement_ = false;
    static inline uint8_t eepromAddress_ = 0;
};
#endif //SXCHIPSET_SEESAWINTERFACE_H
