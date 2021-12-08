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
// Created by jwscoggins on 10/10/21.
//

#ifndef SXCHIPSET_DISPLAYINTERFACE_H
#define SXCHIPSET_DISPLAYINTERFACE_H
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_seesaw.h>
#ifdef CHIPSET_TYPE1
#include <Adafruit_ST7735.h>
#include <Adafruit_TFTShield18.h>
#elif defined(CHIPSET_TYPE2)
#include <Adafruit_ILI9341.h>
#include <Adafruit_FT6206.h>
#endif

#include "Pinout.h"

template<Address baseAddress>
class DisplayInterface {
public:
    enum class SeesawRegisters : uint8_t {
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
    enum class DisplayInterfaceRegisters : uint8_t {
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
public:
    static constexpr auto StartAddress = baseAddress;
    static constexpr SplitWord32 StartAddressSplit { StartAddress};
    static constexpr auto StartPage = StartAddressSplit.getTargetPage();
    static constexpr auto SeesawSectionStart = StartAddress;
    static constexpr auto SeesawSectionEnd = SeesawSectionStart + 0x100;
    static constexpr SplitWord32 SeesawSectionStart_Split {SeesawSectionStart};
    static constexpr auto SeesawPage = SeesawSectionStart_Split.getTargetPage();
    static constexpr auto DisplaySectionStart = SeesawSectionEnd;
    static constexpr SplitWord32 DisplaySectionStart_Split {DisplaySectionStart};
    static constexpr auto DisplayPage = DisplaySectionStart_Split.getTargetPage();
    static constexpr auto DisplaySectionEnd = DisplaySectionStart + 0x100;
    static constexpr auto EndAddress = DisplaySectionEnd;
    static constexpr SplitWord32 EndAddressSplit{EndAddress};
    static constexpr auto EndPage = EndAddressSplit.getTargetPage();
    static constexpr auto SectionID = StartAddressSplit.getMostSignificantByte();

public:
    DisplayInterface() = delete;
    ~DisplayInterface() = delete;
    static void begin() noexcept {
        pinMode(i960Pinout::SD_EN, OUTPUT);
        digitalWrite<i960Pinout::SD_EN, HIGH>();
#ifdef CHIPSET_TYPE1
            pinMode(i960Pinout::TFT_CS, OUTPUT);
            digitalWrite<i960Pinout::TFT_CS, HIGH>();
            Wire.begin();
            if (!displayShield_.begin()) {
                signalHaltState(F("display shield seesaw could not be initialized!"));
            }
            Serial.println(F("Display seesaw started"));
            Serial.print(F("Version: "));
            Serial.println(displayShield_.getVersion(), HEX);

            setBacklightIntensity(TFTSHIELD_BACKLIGHT_OFF);
            displayShield_.tftReset();
            tft.initR(INITR_BLACKTAB);
            setBacklightIntensity(TFTSHIELD_BACKLIGHT_ON);
            Serial.println(F("TFT UP AND OK!"));
            tft.fillScreen(ST7735_CYAN);
            delay(1000);
            tft.fillScreen(ST7735_BLACK);
#elif defined(CHIPSET_TYPE2)
        tft.begin();
        tft.fillScreen(ILI9341_BLACK);
#endif
    }
private:
    static void setBacklightIntensity(uint16_t value) noexcept {
#ifdef CHIPSET_TYPE1
            backlightIntensity_ = value;
            displayShield_.setBacklight(backlightIntensity_);
#endif
    }
public:
    static uint16_t read(uint8_t targetPage, uint8_t offset, LoadStoreStyle lss) noexcept {
        if constexpr (TargetBoard::onAtmega1284p_Type1()) {
            switch (targetPage) {
                case SeesawPage:
                    return handleSeesawReads(offset, lss);
                case DisplayPage:
                    return handleDisplayRead(offset, lss);
                default:
                    return 0;
            }
        } else {
            return 0;
        }
    }
    static void write(uint8_t targetPage, uint8_t offset, LoadStoreStyle lss, SplitWord16 value) noexcept {
        if constexpr (TargetBoard::onAtmega1284p_Type1()) {
            switch (targetPage) {
                case SeesawPage:
                    handleSeesawWrite(offset, lss, value);
                    break;
                case DisplayPage:
                    handleDisplayWrite(offset, lss, value);
                    break;
                default:
                    break;
            }
        }
    }
private:
    enum class InvokeOpcodes : uint8_t {
        // four bit class, four bit kind
        // 0 -> Draw
        // 1 -> Fill
        // 2 -> Other
        // 3-15 -> undefined
        DrawPixel = 0x00,
        DrawLine = 0x01,
        DrawFastVLine = 0x02,
        DrawFastHLine = 0x03,
        DrawRect = 0x04,
        DrawTriangle = 0x05, // technically reserved
        DrawRoundRect = 0x06,
        DrawCircle = 0x07,
        // reserved
        // fill operations
        // first four positions reserved
        FillScreen = 0x10,
        FillRect = 0x14,
        FillTriangle = 0x15, // technically reserved
        FillRoundRect = 0x16,
        FillCircle = 0x17,
    };
    static void invoke(SplitWord16 opcode) noexcept {
            switch (static_cast<InvokeOpcodes>(opcode.getWholeValue())) {
                case InvokeOpcodes::FillScreen:
                    tft.fillScreen(fields_[0]);
                    break;
                case InvokeOpcodes::DrawPixel:
                    tft.drawPixel(fields_[0], fields_[1], fields_[2]);
                    break;
                case InvokeOpcodes::DrawLine:
                    tft.drawLine(fields_[0], fields_[1], fields_[2], fields_[3], fields_[4]);
                    break;
                case InvokeOpcodes::DrawFastVLine:
                    tft.drawFastVLine(fields_[0], fields_[1], fields_[2], fields_[3]);
                    break;
                case InvokeOpcodes::DrawFastHLine:
                    tft.drawFastHLine(fields_[0], fields_[1], fields_[2], fields_[3]);
                    break;
                case InvokeOpcodes::DrawRect:
                    tft.drawRect(fields_[0], fields_[1], fields_[2], fields_[3], fields_[4]);
                    break;
                case InvokeOpcodes::FillRect:
                    tft.fillRect(fields_[0], fields_[1], fields_[2], fields_[3], fields_[4]);
                    break;
                case InvokeOpcodes::FillCircle:
                    tft.fillCircle(fields_[0], fields_[1], fields_[2], fields_[3]);
                    break;
                case InvokeOpcodes::DrawCircle:
                    tft.drawCircle(fields_[0], fields_[1], fields_[2], fields_[3]);
                    break;
                case InvokeOpcodes::FillRoundRect:
                    tft.fillRoundRect(fields_[0], fields_[1], fields_[2], fields_[3], fields_[4], fields_[5]);
                    break;
                case InvokeOpcodes::DrawRoundRect:
                    tft.drawRoundRect(fields_[0], fields_[1], fields_[2], fields_[3], fields_[4], fields_[5]);
                    break;
                case InvokeOpcodes::FillTriangle:
                    tft.fillTriangle(fields_[0], fields_[1], fields_[2], fields_[3], fields_[4], fields_[5], fields_[6]);
                    break;
                case InvokeOpcodes::DrawTriangle:
                    tft.drawTriangle(fields_[0], fields_[1], fields_[2], fields_[3], fields_[4], fields_[5], fields_[6]);
                    break;
                default:
                    break;
            }
    }
    static uint16_t handleDisplayRead(uint8_t offset, LoadStoreStyle) noexcept {
            switch (static_cast<DisplayInterfaceRegisters>(offset)) {
                case DisplayInterfaceRegisters::InstructionField00:
                case DisplayInterfaceRegisters::InstructionField01:
                case DisplayInterfaceRegisters::InstructionField02:
                case DisplayInterfaceRegisters::InstructionField03:
                case DisplayInterfaceRegisters::InstructionField04:
                case DisplayInterfaceRegisters::InstructionField05:
                case DisplayInterfaceRegisters::InstructionField06:
                case DisplayInterfaceRegisters::InstructionField07:
                    return fields_[offset - static_cast<byte>(DisplayInterfaceRegisters::InstructionField00)];
                default:
                    return 0;
            }
    }
    static void handleDisplayWrite(uint8_t offset, LoadStoreStyle lss, SplitWord16 value) noexcept {
            switch (static_cast<DisplayInterfaceRegisters>(offset)) {
                case DisplayInterfaceRegisters::InstructionField00:
                case DisplayInterfaceRegisters::InstructionField01:
                case DisplayInterfaceRegisters::InstructionField02:
                case DisplayInterfaceRegisters::InstructionField03:
                case DisplayInterfaceRegisters::InstructionField04:
                case DisplayInterfaceRegisters::InstructionField05:
                case DisplayInterfaceRegisters::InstructionField06:
                case DisplayInterfaceRegisters::InstructionField07:
                    fields_[offset - static_cast<byte>(DisplayInterfaceRegisters::InstructionField00)] = value.getWholeValue();
                    break;
                case DisplayInterfaceRegisters::Invoke0:
                    invoke(value);
                    break;
                default:
                    break;
            }
    }
    static void handleSeesawWrite(uint8_t offset, LoadStoreStyle, SplitWord16 value) noexcept {
        if constexpr (TargetBoard::onAtmega1284p_Type1()) {
            switch (static_cast<SeesawRegisters>(offset)) {
                case SeesawRegisters::Backlight:
                    setBacklightIntensity(value.getWholeValue());
                    break;
                default:
                    break;
            }
        }
    }
    static uint16_t handleSeesawReads(uint8_t offset, LoadStoreStyle) noexcept {
        if constexpr (TargetBoard::onAtmega1284p_Type1()) {
            switch (static_cast<SeesawRegisters>(offset)) {
                case SeesawRegisters::Backlight:
                    return backlightIntensity_;
                default:
                    return 0;
            }
        } else {
            return 0;
        }

    }
private:
#ifdef CHIPSET_TYPE1
    static inline Adafruit_TFTShield18 displayShield_;
    static inline Adafruit_ST7735 tft{static_cast<int>(i960Pinout::TFT_CS),
                                      static_cast<int>(i960Pinout::TFT_DC),
                                      -1};
#elif defined(CHIPSET_TYPE2)
static inline Adafruit_ILI9341 tft {static_cast<int>(i960Pinout::TFT_CS), static_cast<int>(i960Pinout::TFT_DC) };
static inline Adafruit_FT6206 touchController{};
#endif
    static inline uint16_t backlightIntensity_ = 0;
    static inline uint16_t fields_[8] { 0 };
};
#endif //SXCHIPSET_DISPLAYINTERFACE_H
