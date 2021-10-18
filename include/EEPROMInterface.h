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

#ifndef SXCHIPSET_EEPROMINTERFACE_H
#define SXCHIPSET_EEPROMINTERFACE_H
#include <Arduino.h>
#include <EEPROM.h>

#include "Pinout.h"

/**
 * @brief Unlike other core chipset features, the eeprom interface doesn't have "registers", instead it is just 4k of eeprom mapped
 * @tparam baseAddress
 */
template<Address baseAddress>
class EEPROMInterface {
public:
    static constexpr Address Size = E2END + 1;
    static constexpr auto StartAddress = baseAddress;
    static constexpr SplitWord32 StartAddressSplit { StartAddress };
    static constexpr auto StartPage = StartAddressSplit.getTargetPage();
    static constexpr auto EndAddress = StartAddress + Size;
    static constexpr SplitWord32 EndAddressSplit{EndAddress};
    static constexpr auto EndPage = EndAddressSplit.getTargetPage();
    static constexpr auto SectionID = StartAddressSplit.getMostSignificantByte();
public:
    static constexpr bool respondsTo(byte targetPage) noexcept {
        return targetPage >= StartPage && targetPage < EndPage;
    }
    static void begin() noexcept {
        Serial.print(F("EEPROM INTERFACE UP, HAVE ACCESS TO "));
        Serial.print(Size);
        Serial.println(F(" BYTES"));
    }

    static uint16_t read(uint8_t targetPage, uint8_t offset, LoadStoreStyle) noexcept {
        SplitWord16 address{offset, static_cast<byte>(targetPage - StartPage)}; // normalize it
        uint16_t result = 0;
        EEPROM.get(address.getWholeValue(), result);
        return result;
    }

    static void write(uint8_t targetPage, uint8_t offset, LoadStoreStyle lss, SplitWord16 value) noexcept {
        SplitWord16 address{offset, static_cast<byte>(targetPage - StartPage)};
        switch (lss) {
            case LoadStoreStyle::Lower8:
                EEPROM.update(address.getWholeValue(), value.bytes[0]);
                break;
            case LoadStoreStyle::Upper8:
                EEPROM.update(address.getWholeValue() + 1, value.bytes[1]);
                break;
            case LoadStoreStyle::Full16:
                EEPROM.put(address.getWholeValue(), value.getWholeValue());
                break;
            default:
                break;
        }
    }
};

#endif //SXCHIPSET_EEPROMINTERFACE_H
