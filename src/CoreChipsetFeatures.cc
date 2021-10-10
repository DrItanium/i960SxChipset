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

#include "CoreChipsetFeatures.h"

void
CoreChipsetFeatures::begin() noexcept {
    pinMode(i960Pinout::TFT_CS, OUTPUT);
    pinMode(i960Pinout::SD_EN, OUTPUT);
    digitalWrite<i960Pinout::TFT_CS, HIGH>();
    digitalWrite<i960Pinout::SD_EN, HIGH>();
    Wire.begin();
    if (!displayShield_.begin()) {
        signalHaltState(F("display shield seesaw could not be initialized!")) ;
    }
    Serial.println(F("Display seesaw started"));
    Serial.print("Version: ");
    Serial.println(displayShield_.getVersion(), HEX);

    displayShield_.setBacklight(TFTSHIELD_BACKLIGHT_OFF);
    displayShield_.tftReset();
    tft.initR(INITR_BLACKTAB);

    Serial.println(F("TFT UP AND OK!"));
    tft.fillScreen(ST7735_CYAN);
    delay(1000);
    tft.fillScreen(ST7735_BLACK);
}
uint16_t
CoreChipsetFeatures::handleFirstPageRegisterReads(uint8_t offset, LoadStoreStyle) noexcept {
    switch (static_cast<Registers>(offset)) {
        case Registers::ConsoleIO:
            return Serial.read();
        case Registers::AddressDebuggingFlag:
            return static_cast<uint16_t>(enableAddressDebugging_);
        default:
            return 0;
    }
}
void
CoreChipsetFeatures::handleDisplayShieldWrites(uint8_t offset, LoadStoreStyle, SplitWord16 value) noexcept {
    switch (static_cast<DisplayShieldRegisters>(offset))  {
        case DisplayShieldRegisters::Backlight:
            backlightIntensity_ = value.getWholeValue();
            displayShield_.setBacklight(backlightIntensity_);
            break;
        default: break;
    }
}
uint16_t
CoreChipsetFeatures::handleDisplayShieldReads(uint8_t offset, LoadStoreStyle) noexcept {
    switch (static_cast<DisplayShieldRegisters>(offset)) {
        case DisplayShieldRegisters::Backlight:
            return backlightIntensity_;
        case DisplayShieldRegisters::RawButtonsLower:
            rawButtons_.wholeValue_ = displayShield_.readButtons();
            return rawButtons_.halves[0];
        case DisplayShieldRegisters::RawButtonsUpper:
            return rawButtons_.halves[1];
        default:
            return 0;
    }
}
void
CoreChipsetFeatures::handleFirstPageRegisterWrites(uint8_t offset, LoadStoreStyle, SplitWord16 value) noexcept {
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
        case Registers::AddressDebuggingFlag:
            enableAddressDebugging_ = (value.getWholeValue() != 0);
            break;
        default:
            break;
    }
}
uint16_t
CoreChipsetFeatures::readIOConfigurationSpace0(uint8_t offset, LoadStoreStyle) noexcept {
    switch (static_cast<IOConfigurationSpace0Registers>(offset)) {
#define X(title, var) \
             case IOConfigurationSpace0Registers:: title ## Lower : return static_cast<uint16_t>(var); \
             case IOConfigurationSpace0Registers:: title ## Upper : return static_cast<uint16_t>(var >> 16)
        X(Serial0BaseAddress, RegisterPage0BaseAddress);
        X(SDCardInterfaceBaseAddress, SDCardInterfaceBaseAddress);
        X(SDCardFileBlock0BaseAddress, SDCardFileInterfaceBlockBaseAddress);
        X(DisplayShieldBaseAddress, DisplayShieldBaseAddress);
        X(ST7735DisplayBaseAddress, ST7735DisplayBaseAddress);
#undef X

        default: return 0; // zero is never an io page!
    }
}
uint16_t
CoreChipsetFeatures::read(uint8_t targetPage, uint8_t offset, LoadStoreStyle lss) noexcept {
    // force override the default implementation
    switch (targetPage) {
        case 0: return readIOConfigurationSpace0(offset, lss);
        case 16: return handleFirstPageRegisterReads(offset, lss);
        case 17: return SDCardInterface::read(offset, lss);
        case 18: return handleDisplayShieldReads(offset, lss);
        case 32: case 33: case 34: case 35: case 36: case 37: case 38: case 39:
        case 40: case 41: case 42: case 43: case 44: case 45: case 46: case 47:
        case 48: case 49: case 50: case 51: case 52: case 53: case 54: case 55:
        case 56: case 57: case 58: case 59: case 60: case 61: case 62: case 63:
            return SDCardInterface::fileRead(targetPage - 32, offset, lss);
        default: return 0;
    }
}
void
CoreChipsetFeatures::write(uint8_t targetPage, uint8_t offset, LoadStoreStyle lss, SplitWord16 value) noexcept {
    switch (targetPage) {
        // ignore writes to configuration space
        case 16: handleFirstPageRegisterWrites(offset, lss, value); break;
        case 17: SDCardInterface::write(offset, lss, value); break;
        case 18: handleDisplayShieldWrites(offset, lss, value); break;
        case 32: case 33: case 34: case 35: case 36: case 37: case 38: case 39:
        case 40: case 41: case 42: case 43: case 44: case 45: case 46: case 47:
        case 48: case 49: case 50: case 51: case 52: case 53: case 54: case 55:
        case 56: case 57: case 58: case 59: case 60: case 61: case 62: case 63:
            SDCardInterface::fileWrite(targetPage - 32, offset, lss, value);
            break;
        default: break;
    }
}
