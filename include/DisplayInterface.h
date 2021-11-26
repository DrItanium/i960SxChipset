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
#include <Adafruit_ST7735.h>
#include <Adafruit_TFTShield18.h>

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
        TwoByteEntry(PortIO),
        TwoByteEntry(Invoke),

        FourByteEntry(Result),

        TwoByteEntry(X0),
        TwoByteEntry(Y0),

        TwoByteEntry(X1),
        TwoByteEntry(Y1),

        TwoByteEntry(X2),
        TwoByteEntry(Y2),

        TwoByteEntry(SX),
        TwoByteEntry(SY),

        TwoByteEntry(W),
        TwoByteEntry(H),

        TwoByteEntry(R),
        TwoByteEntry(Unused1),

        TwoByteEntry(ForegroundColor),
        TwoByteEntry(BackgroundColor),

        TwoByteEntry(PerformFill),
        TwoByteEntry(Invert),

        TwoByteEntry(Rotation),
        TwoByteEntry(TextWrap),

        TwoByteEntry(DisplayWidth),
        TwoByteEntry(DisplayHeight),

        TwoByteEntry(CursorX),
        TwoByteEntry(CursorY),

        TwoByteEntry(TreatAsSquare),
        TwoByteEntry(CurrentCharacter),

        FourByteEntry(PackedRGB),

        TwoByteEntry(ColorBlack), // fixed color values
        TwoByteEntry(ColorWhite), // fixed color values

        TwoByteEntry(ColorRed), // fixed color values
        TwoByteEntry(ColorGreen), // fixed color values

        TwoByteEntry(ColorBlue), // fixed color values
        TwoByteEntry(ColorCyan), // fixed color values

        TwoByteEntry(ColorMagenta), // fixed color values
        TwoByteEntry(ColorYellow), // fixed color values

        TwoByteEntry(ColorOrange), // fixed color values
        TwoByteEntry(Unused2),
#undef SixteenByteEntry
#undef TwelveByteEntry
#undef EightByteEntry
#undef FourByteEntry
#undef TwoByteEntry
        End,
#define X(component) Color ## component = Color ## component ## 0
X(Black),
X(White),
X(Red),
X(Green),
X(Blue),
X(Cyan),
X(Magenta),
X(Yellow),
X(Orange),
#undef X
        Invoke= Invoke0,
        ResultLower = Result00,
        ResultUpper = Result10,
        X0 = X00,
        Y0 = Y00,
        X1 = X10,
        Y1 = Y10,
        X2 = X20,
        Y2 = Y20,
        W = W0,
        H = H0,
        R = R0,
        ForegroundColor = ForegroundColor0,
        BackgroundColor = BackgroundColor0,
        Invert = Invert0,
        Rotation = Rotation0,
        TextWrap = TextWrap0,
        DisplayWidth = DisplayWidth0,
        DisplayHeight = DisplayHeight0,
        CursorX = CursorX0,
        CursorY = CursorY0,
        PortIO = PortIO0,
        PerformFill = PerformFill0,
        TreatAsSquare = TreatAsSquare0,
        SX = SX0,
        SY = SY0,
        CurrentCharacter = CurrentCharacter0,
        PackedRGBLower = PackedRGB00,
        PackedRGBUpper = PackedRGB10,
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
    static constexpr bool respondsTo(byte targetPage) noexcept {
        return targetPage >= StartPage && targetPage < EndPage;
    }
    static void begin() noexcept {
#ifdef CHIPSET_TYPE1
        pinMode(i960Pinout::TFT_CS, OUTPUT);
        pinMode(i960Pinout::SD_EN, OUTPUT);
        digitalWrite<i960Pinout::TFT_CS, HIGH>();
        digitalWrite<i960Pinout::SD_EN, HIGH>();
        Wire.begin();
        if (!displayShield_.begin()) {
            signalHaltState(F("display shield seesaw could not be initialized!")) ;
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
#endif
    }
private:
    static void turnBacklightOff() noexcept {
        setBacklightIntensity(TFTSHIELD_BACKLIGHT_OFF);
    }
    static void turnBacklightOn() noexcept {
        setBacklightIntensity(TFTSHIELD_BACKLIGHT_ON);
    }
    static void setBacklightIntensity(uint16_t value) noexcept {
        backlightIntensity_ = value;
        displayShield_.setBacklight(backlightIntensity_);
    }
public:
    static uint16_t read(uint8_t targetPage, uint8_t offset, LoadStoreStyle lss) noexcept {
        switch  (targetPage) {
            case SeesawPage:
                return handleSeesawReads(offset, lss);
            case DisplayPage:
                return handleDisplayRead(offset, lss);
            default:
                return 0;
        }
    }
    static void write(uint8_t targetPage, uint8_t offset, LoadStoreStyle lss, SplitWord16 value) noexcept {
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
private:
    enum class InvokeOpcodes : uint8_t {
        DrawPixel,
        DrawFastVLine,
        DrawFastHLine,
        DrawRect, // use the fill flag to use fill rect instead
        FillScreen,
        DrawLine,
        DrawCircle, // use the fill flag to use fillCircle instead
        DrawTriangle, // use the fill flag to use fillTriangle instead
        DrawRoundRect, // use the fill flag to use fillRoundRect instead
        SetCursor,
        SetTextColor,
        SetTextSize,
        DrawChar,
        Color565,
    };
    static void invoke(SplitWord16 opcode) noexcept {
        returnValue_.wholeValue_ = 0;
        switch (static_cast<InvokeOpcodes>(opcode.getWholeValue())) {
            case InvokeOpcodes::Color565:
                returnValue_.wholeValue_ = tft.color565(packedRGB_.bytes[0],
                                                        packedRGB_.bytes[1],
                                                        packedRGB_.bytes[2]);
                break;
            case InvokeOpcodes::FillScreen:
                tft.fillScreen(foregroundColor_);
                break;
            case InvokeOpcodes::DrawPixel:
                tft.drawPixel(x0_, y0_, foregroundColor_);
                break;
            case InvokeOpcodes::DrawLine:
                tft.drawLine(x0_, y0_, x1_, y1_, foregroundColor_);
                break;
            case InvokeOpcodes::DrawFastVLine:
                tft.drawFastVLine(x0_, y0_, h_, foregroundColor_);
                break;
            case InvokeOpcodes::DrawFastHLine:
                tft.drawFastHLine(x0_, y0_, w_, foregroundColor_);
                break;
            case InvokeOpcodes::DrawRect:
                if (performFill_) {
                    tft.fillRect(x0_, y0_, w_, h_, foregroundColor_);
                } else {
                    tft.drawRect(x0_, y0_, w_, h_, foregroundColor_);
                }
                break;
            case InvokeOpcodes::DrawCircle:
                if (performFill_) {
                    tft.fillCircle(x0_, y0_, r_, foregroundColor_);
                } else {
                    tft.drawCircle(x0_, y0_, r_, foregroundColor_);
                }
                break;
            case InvokeOpcodes::DrawTriangle:
                if (performFill_) {
                    tft.fillTriangle(x0_, y0_, x1_, y1_, x2_, y2_, foregroundColor_);
                } else {
                    tft.drawTriangle(x0_, y0_, x1_, y1_, x2_, y2_, foregroundColor_);
                }
                break;
            case InvokeOpcodes::DrawRoundRect:
                if (performFill_) {
                    tft.fillRoundRect(x0_, y0_, w_, h_, r_, foregroundColor_);
                } else {
                    tft.drawRoundRect(x0_, y0_, w_, h_, r_, foregroundColor_);
                }
                break;
            case InvokeOpcodes::SetCursor:
                tft.setCursor(x0_, y0_);
                break;
            case InvokeOpcodes::SetTextColor:
                tft.setTextColor(foregroundColor_, backgroundColor_);
                break;
            case InvokeOpcodes::SetTextSize:
                if (treatAsSquare_) {
                    tft.setTextSize(sx_);
                } else {
                    tft.setTextSize(sx_, sy_);
                }
                break;
            case InvokeOpcodes::DrawChar:
                if (treatAsSquare_) {
                    tft.drawChar(x0_, y0_, currentCharacter_, foregroundColor_, backgroundColor_, sx_);
                } else {
                    tft.drawChar(x0_, y0_, currentCharacter_, foregroundColor_, backgroundColor_, sx_, sy_);
                }
                break;
            default:
                returnValue_.wholeValue_ = 0xFFFF'FFFF;
                break;
        }
    }
    static uint16_t handleDisplayRead(uint8_t offset, LoadStoreStyle) noexcept {
        switch (static_cast<DisplayInterfaceRegisters>(offset)) {
            case DisplayInterfaceRegisters::Invert: return displayIsInverted_ ? 0xFFFF : 0;
            case DisplayInterfaceRegisters::TextWrap: return textWrapOn_ ? 0xFFFF : 0;
            case DisplayInterfaceRegisters::PerformFill: return performFill_ ? 0xFFFF : 0;
            case DisplayInterfaceRegisters::TreatAsSquare: return treatAsSquare_ ? 0xFFFF : 0;
            case DisplayInterfaceRegisters::DisplayWidth: return tft.width();
            case DisplayInterfaceRegisters::DisplayHeight: return tft.height();
            case DisplayInterfaceRegisters::CursorX: return tft.getCursorX();
            case DisplayInterfaceRegisters::CursorY: return tft.getCursorY();
            case DisplayInterfaceRegisters::Rotation: return tft.getRotation();
            case DisplayInterfaceRegisters::ResultLower: return returnValue_.getLowerHalf();
            case DisplayInterfaceRegisters::ResultUpper: return returnValue_.getUpperHalf();
            case DisplayInterfaceRegisters::X0: return x0_ ;
            case DisplayInterfaceRegisters::Y0: return y0_ ;
            case DisplayInterfaceRegisters::X1: return x1_ ;
            case DisplayInterfaceRegisters::Y1: return y1_ ;
            case DisplayInterfaceRegisters::X2: return x2_ ;
            case DisplayInterfaceRegisters::Y2: return y2_ ;
            case DisplayInterfaceRegisters::W: return w_ ;
            case DisplayInterfaceRegisters::H: return h_ ;
            case DisplayInterfaceRegisters::R: return r_ ;
            case DisplayInterfaceRegisters::PackedRGBLower: return packedRGB_.getLowerHalf();
            case DisplayInterfaceRegisters::PackedRGBUpper: return packedRGB_.getUpperHalf();
            case DisplayInterfaceRegisters::BackgroundColor: return backgroundColor_ ;
            case DisplayInterfaceRegisters::ForegroundColor: return foregroundColor_ ;
            case DisplayInterfaceRegisters::SX: return sx_ ;
            case DisplayInterfaceRegisters::SY: return sy_ ;
            case DisplayInterfaceRegisters::CurrentCharacter: return currentCharacter_;
            case DisplayInterfaceRegisters::ColorBlack: return ST7735_BLACK;
            case DisplayInterfaceRegisters::ColorWhite: return ST7735_WHITE;
            case DisplayInterfaceRegisters::ColorRed: return ST7735_RED;
            case DisplayInterfaceRegisters::ColorGreen: return ST7735_GREEN;
            case DisplayInterfaceRegisters::ColorBlue: return ST7735_BLUE;
            case DisplayInterfaceRegisters::ColorMagenta: return ST7735_MAGENTA;
            case DisplayInterfaceRegisters::ColorYellow: return ST7735_YELLOW;
            case DisplayInterfaceRegisters::ColorOrange: return ST7735_ORANGE;
            default: break;
        }
        return 0;
    }
    static void handleDisplayWrite(uint8_t offset, LoadStoreStyle lss, SplitWord16 value) noexcept {
            switch (static_cast<DisplayInterfaceRegisters>(offset)) {
                case DisplayInterfaceRegisters::Invert:
                    displayIsInverted_ = value.getWholeValue() != 0;
                    tft.invertDisplay(displayIsInverted_);
                    break;
                case DisplayInterfaceRegisters::TextWrap:
                    textWrapOn_ = value.getWholeValue() != 0;
                    tft.setTextWrap(textWrapOn_);
                    break;
                case DisplayInterfaceRegisters::Rotation: tft.setRotation(value.getLowerHalf()); break;
                case DisplayInterfaceRegisters::CursorX: tft.setCursor(value.getWholeValue(), tft.getCursorY()); break;
                case DisplayInterfaceRegisters::CursorY: tft.setCursor(tft.getCursorX(), value.getWholeValue()); break;
                case DisplayInterfaceRegisters::Invoke: invoke(value); break;
                case DisplayInterfaceRegisters::PerformFill: performFill_ = value.getWholeValue() != 0; break;
                case DisplayInterfaceRegisters::TreatAsSquare: treatAsSquare_ = value.getWholeValue() != 0; break;
                case DisplayInterfaceRegisters::X0: x0_ = value.wholeValue_; break;
                case DisplayInterfaceRegisters::Y0: y0_ = value.wholeValue_; break;
                case DisplayInterfaceRegisters::X1: x1_ = value.wholeValue_; break;
                case DisplayInterfaceRegisters::Y1: y1_ = value.wholeValue_; break;
                case DisplayInterfaceRegisters::X2: x2_ = value.wholeValue_; break;
                case DisplayInterfaceRegisters::Y2: y2_ = value.wholeValue_; break;
                case DisplayInterfaceRegisters::SX: sx_ = value.wholeValue_; break;
                case DisplayInterfaceRegisters::SY: sy_ = value.wholeValue_; break;
                case DisplayInterfaceRegisters::W: w_ = value.wholeValue_; break;
                case DisplayInterfaceRegisters::H: h_ = value.wholeValue_; break;
                case DisplayInterfaceRegisters::R: r_ = value.wholeValue_; break;
                case DisplayInterfaceRegisters::PackedRGBLower: packedRGB_.setLowerHalf(value); break;
                case DisplayInterfaceRegisters::PackedRGBUpper: packedRGB_.setUpperHalf(value); break;
                case DisplayInterfaceRegisters::BackgroundColor: backgroundColor_ = value.wholeValue_; break;
                case DisplayInterfaceRegisters::ForegroundColor: foregroundColor_ = value.wholeValue_; break;
                case DisplayInterfaceRegisters::ResultLower: returnValue_.setLowerHalf(value); break;
                case DisplayInterfaceRegisters::ResultUpper: returnValue_.setUpperHalf(value); break;
                case DisplayInterfaceRegisters::CurrentCharacter: currentCharacter_ = value.wholeValue_; break;
                case DisplayInterfaceRegisters::PortIO: tft.write(value.getLowerHalf()); break;
                default: break;

            }
    }
    static void handleSeesawWrite(uint8_t offset, LoadStoreStyle, SplitWord16 value) noexcept {
#ifdef CHIPSET_TYPE1
        switch (static_cast<SeesawRegisters>(offset))  {
            case SeesawRegisters::Backlight:
                setBacklightIntensity(value.getWholeValue());
                break;
            default: break;
        }
#endif
    }
    static uint16_t handleSeesawReads(uint8_t offset, LoadStoreStyle) noexcept {
#ifdef CHIPSET_TYPE1
        switch (static_cast<SeesawRegisters>(offset)) {
            case SeesawRegisters::Backlight:
                return backlightIntensity_;
            case SeesawRegisters::RawButtonsLower:
                rawButtons_.wholeValue_ = displayShield_.readButtons();
                return rawButtons_.halves[0];
            case SeesawRegisters::RawButtonsUpper:
                return rawButtons_.halves[1];
            default:
                return 0;
        }
#else
        return 0;
#endif

    }
private:
#ifdef CHIPSET_TYPE1
    static inline Adafruit_TFTShield18 displayShield_;
    static inline Adafruit_ST7735 tft{static_cast<int>(i960Pinout::TFT_CS),
                                      static_cast<int>(i960Pinout::TFT_DC),
                                      -1};
    static inline uint16_t backlightIntensity_ = 0;
    static inline SplitWord32 rawButtons_{0};
    static inline SplitWord32 returnValue_{0};
    static inline SplitWord32 packedRGB_ { 0};
#define X(type, name, defaultValue) static inline type name = defaultValue;
#include "InternalDisplayRegisters.def"
#undef X
#endif
};
#endif //SXCHIPSET_DISPLAYINTERFACE_H
