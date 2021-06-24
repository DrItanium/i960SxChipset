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
// Created by jwscoggins on 6/23/21.
//

#ifndef I960SXCHIPSET_TFTDISPLAYTHING_H
#define I960SXCHIPSET_TFTDISPLAYTHING_H
#include <Adafruit_TFTShield18.h>
#include <Adafruit_ST7735.h>
#include "Device.h"
class TFTDisplayThing : public IOSpaceThing {
public:
    enum class Registers : uint32_t {
        Flush = 0,
        IO,
        Available,
        AvailableForWrite,
        Command,
        X,
        Y,
        W,
        H,
        Radius,
        Color,
        BGColor,
        X0,
        Y0,
        X1,
        Y1,
        X2,
        Y2,
        R,
        G,
        B,
        Doorbell,
        Backlight,
        BacklightFrequency,
        ButtonsLower,
        ButtonsUpper,
        ButtonsQuery,

    };
    enum class Opcodes : uint16_t {
        None = 0,
        SetRotation,
        InvertDisplay,
        FillRect,
        FillScreen,
        DrawLine,
        DrawRect,
        DrawCircle,
        FillCircle,
        DrawTriangle,
        FillTriangle,
        SetTextSizeSquare,
        SetTextSizeRectangle,
        SetCursor,
        SetTextColor0,
        SetTextColor1,
        SetTextWrap,
        GetWidth,
        GetHeight,
        GetRotation,
        GetCursorX,
        GetCursorY,
        DrawPixel,
        Color565,
        DrawRoundRect,
        FillRoundRect,
    };
public:
    explicit TFTDisplayThing(Address base) : IOSpaceThing(base, base + 0x100),
                                            display_(static_cast<int>(i960Pinout::DISPLAY_EN),
                                                     static_cast<int>(i960Pinout::DC),
                                                     -1) { }
    ~TFTDisplayThing() override = default;
    /**
     * @brief Invoke on doorbell write
     * @param value the value written to the doorbell
     * @return the value to return to the i960 if it makes sense (otherwise it will be zero)
     */
    uint16_t invoke(uint16_t /* unused */) {
        // perhaps we'll do nothing with the value but hold onto it for now
        switch (command_) {
            case Opcodes::SetRotation:
                display_.setRotation(x_);
                break;
            case Opcodes::InvertDisplay:
                display_.invertDisplay(x_ != 0);
                break;
            case Opcodes::DrawPixel:
                display_.drawPixel(x_, y_, color_);
                break;
            case Opcodes::FillRect:
                display_.fillRect(x_, y_, w_, h_, color_);
                break;
            case Opcodes::FillScreen:
                display_.fillScreen(color_);
                break;
            case Opcodes::Color565:
                return display_.color565(r_, g_, b_);
            case Opcodes::DrawLine:
                display_.drawLine(x0_, y0_, x1_, y1_, color_);
                break;
            case Opcodes::DrawRect:
                display_.drawRect(x_, y_, w_, h_, color_);
                break;
            case Opcodes::DrawCircle:
                display_.drawCircle(x0_, y0_, r_, color_);
                break;
            case Opcodes::FillCircle:
                display_.fillCircle(x0_, y0_, r_, color_);
                break;
            case Opcodes::DrawTriangle:
                display_.drawTriangle(x0_, y0_, x1_, y1_, x2_, y2_, color_);
                break;
            case Opcodes::FillTriangle:
                display_.fillCircle(x0_, y0_, r_, color_);
                break;
            case Opcodes::DrawRoundRect:
                display_.drawRoundRect(x_, y_, w_, h_, r_, color_);
                break;
            case Opcodes::FillRoundRect:
                display_.fillRoundRect(x_, y_, w_, h_, r_, color_);
                break;
            case Opcodes::SetTextSizeSquare:
                display_.setTextSize(x_);
                break;
            case Opcodes::SetTextSizeRectangle:
                display_.setTextSize(x_, y_);
                break;
            case Opcodes::SetCursor:
                display_.setCursor(x_, y_);
                break;
            case Opcodes::SetTextColor0:
                display_.setTextColor(color_);
                break;
            case Opcodes::SetTextColor1:
                display_.setTextColor(color_, bgcolor_);
                break;
            case Opcodes::SetTextWrap:
                display_.setTextWrap(x_ != 0);
                break;
            case Opcodes::GetWidth:
                return display_.width();
            case Opcodes::GetHeight:
                return display_.height();
            case Opcodes::GetRotation:
                return display_.getRotation();
            case Opcodes::GetCursorX:
                return display_.getCursorX();
            case Opcodes::GetCursorY:
                return display_.getCursorY();
            default:
                return 0;
        }
        return 0;
    }
public:
    [[nodiscard]] constexpr auto getCommand() const noexcept { return command_; }
    [[nodiscard]] constexpr auto getX() const noexcept { return x_; }
    [[nodiscard]] constexpr auto getY() const noexcept { return y_; }
    [[nodiscard]] constexpr auto getW() const noexcept { return w_; }
    [[nodiscard]] constexpr auto getH() const noexcept { return h_; }
    [[nodiscard]] constexpr auto getRadius() const noexcept { return radius_; }
    [[nodiscard]] constexpr uint16_t getColor() const noexcept { return color_; }
    [[nodiscard]] constexpr uint16_t getBackgroundColor() const noexcept { return bgcolor_; }
    [[nodiscard]] constexpr auto getX0() const noexcept { return x0_; }
    [[nodiscard]] constexpr auto getY0() const noexcept { return y0_; }
    [[nodiscard]] constexpr auto getX1() const noexcept { return x1_; }
    [[nodiscard]] constexpr auto getY1() const noexcept { return y1_; }
    [[nodiscard]] constexpr auto getX2() const noexcept { return x2_; }
    [[nodiscard]] constexpr auto getY2() const noexcept { return y2_; }
    [[nodiscard]] constexpr auto getRed() const noexcept { return r_; }
    [[nodiscard]] constexpr auto getGreen() const noexcept { return g_; }
    [[nodiscard]] constexpr auto getBlue() const noexcept { return b_; }
    void setCommand(Opcodes command) noexcept { command_ = command; }
    void setX(int16_t x) noexcept { x_ = x; }
    void setY(int16_t y) noexcept { y_ = y; }
    void setW(int16_t w) noexcept { w_ = w; }
    void setH(int16_t h) noexcept { h_ = h; }
    void setRadius(int16_t radius) noexcept { radius_ = radius; }
    void setColor(uint16_t color) noexcept { color_ = color; }
    void setBackgroundColor(uint16_t color) noexcept { bgcolor_ = color; }
    void setX0(int16_t value) noexcept { x0_ = value; }
    void setY0(int16_t value) noexcept { y0_ = value; }
    void setX1(int16_t value) noexcept { x1_ = value; }
    void setY1(int16_t value) noexcept { y1_ = value; }
    void setX2(int16_t value) noexcept { x2_ = value; }
    void setY2(int16_t value) noexcept { y2_ = value; }
    void setR(int16_t value) noexcept { r_ = value; }
    void setG(int16_t value) noexcept { g_ = value; }
    void setB(int16_t value) noexcept { b_ = value; }
    void flush() {
        display_.flush();
    }
    void print(char c) { display_.print(c); }
    [[nodiscard]] bool available() noexcept { return true; }
    [[nodiscard]] bool availableForWriting() noexcept { return display_.availableForWrite(); }
    uint16_t read16(Address address) noexcept override {
            switch (address) {
#define X(title) case (static_cast<Address>(Registers:: title) * sizeof(uint16_t))
                X(Available) : return available();
                X(AvailableForWrite) : return availableForWriting();
                X(Command) : return static_cast<uint16_t>(getCommand());
                X(X) : return getX();
                X(Y) : return getY();
                X(W) : return getW();
                X(H) : return getH();
                X(Radius) : return getRadius();
                X(Color) : return getColor();
                X(BGColor) : return getBackgroundColor();
                X(X0) : return getX0();
                X(Y0) : return getY0();
                X(X1) : return getX1();
                X(Y1) : return getY1();
                X(X2) : return getX2();
                X(Y2) : return getY2();
                X(R) : return getRed();
                X(G) : return getGreen();
                X(B) : return getBlue();
                X(Doorbell) : return invoke(0);
                X(Backlight) : return backlightStatus_;
                X(BacklightFrequency) : return backlightFrequency_;
                X(ButtonsLower) : return buttonsCache_ & 0xFFFF;
                X(ButtonsUpper) : return (buttonsCache_ >> 16) & 0xFFFF;
#undef X
                default: return 0;
            }
    }
    void write16(Address address, uint16_t value) noexcept override {
            switch (address) {
#define X(title) case (static_cast<Address>(Registers:: title) * sizeof(uint16_t))
                X(Command) :
                    setCommand(static_cast<Opcodes>(value));
                break;
                X(X) : setX(static_cast<int16_t>(value)); break;
                X(Y) : setY(static_cast<int16_t>(value)); break;
                X(W) : setW(static_cast<int16_t>(value)); break;
                X(H) : setH(static_cast<int16_t>(value)); break;
                X(Radius) : setRadius(static_cast<int16_t>(value)); break;
                X(Color) : setColor(value); break;
                X(BGColor) : setBackgroundColor(value); break;
                X(X0) : setX0(static_cast<int16_t>(value)); break;
                X(Y0) : setY0(static_cast<int16_t>(value)); break;
                X(X1) : setX1(static_cast<int16_t>(value)); break;
                X(Y1) : setY1(static_cast<int16_t>(value)); break;
                X(X2) : setX2(static_cast<int16_t>(value)); break;
                X(Y2) : setY2(static_cast<int16_t>(value)); break;
                X(R) : setR(static_cast<int16_t>(value)); break;
                X(G) : setG(static_cast<int16_t>(value)); break;
                X(B) : setB(static_cast<int16_t>(value)); break;
                X(Doorbell) :
                    resultLower_ = invoke(value);
                break;
                X(Backlight) :
                    backlightStatus_ = value != 0 ? TFTSHIELD_BACKLIGHT_ON : TFTSHIELD_BACKLIGHT_OFF;
                ss.setBacklight(backlightStatus_);
                break;
                X(BacklightFrequency) :
                    backlightFrequency_ = value;
                ss.setBacklightFreq(backlightFrequency_);
                break;
                X(ButtonsQuery):
                    buttonsCache_ = ss.readButtons();
                break;
#undef X
                default: break;
            }
    }
    inline void fillScreen(uint16_t value) noexcept { display_.fillScreen(value); }
    inline void setCursor(int16_t x, int16_t y) noexcept { display_.setCursor(x, y); }
    inline void setTextColor(uint16_t value) noexcept { display_.setTextColor(value); }
    inline void setTextSize(uint16_t size) noexcept { display_.setTextSize(size); }
    template<typename T, typename ... Args>
    inline void println(T msg, Args&& ... args) noexcept {
        display_.println(msg, args...);
    }
    void
    begin() noexcept override {
            Serial.println(F("Setting up the seesaw"));
            if (!ss.begin()) {
                signalHaltState(F("NO SEESAW"));
            }
            Serial.println(F("seesaw started"));
            Serial.print(F("Version: "));
            Serial.println(ss.getVersion(), HEX);
            ss.setBacklight(TFTSHIELD_BACKLIGHT_OFF);
            ss.tftReset();
            display_.initR(INITR_BLACKTAB); // initialize a ST7735S, black tab
            ss.setBacklight(TFTSHIELD_BACKLIGHT_ON);
            Serial.println(F("TFT OK!"));
            display_.fillScreen(ST77XX_BLACK);
            display_.setCursor(0, 0);
            display_.setTextColor(ST77XX_WHITE);
            display_.setTextSize(3);
            display_.println(F("i960Sx!"));
    }
    void
    clearScreen() {
        display_.fillScreen(ST7735_BLACK);
    }
private:
    Adafruit_ST7735 display_;
    Opcodes command_;
    int16_t x_ = 0;
    int16_t y_ = 0;
    int16_t w_ = 0;
    int16_t h_ = 0;
    int16_t radius_ = 0;
    uint16_t color_ = 0;
    uint16_t bgcolor_ = 0;
    int16_t x0_ = 0;
    int16_t y0_ = 0;
    int16_t x1_ = 0;
    int16_t y1_ = 0;
    int16_t x2_ = 0;
    int16_t y2_ = 0;
    int16_t r_ = 0;
    int16_t g_ = 0;
    int16_t b_ = 0;
    uint16_t resultLower_ = 0;
    uint16_t resultUpper_ = 0;
    uint16_t backlightStatus_ = TFTSHIELD_BACKLIGHT_ON;
    uint16_t backlightFrequency_ = 0;
    uint32_t buttonsCache_ = 0;
    Adafruit_TFTShield18 ss;

};
#endif //I960SXCHIPSET_TFTDISPLAYTHING_H
