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
    while (!SD.begin(static_cast<int>(i960Pinout::SD_EN))) {
        Serial.println(F("SD CARD INIT FAILED...WILL RETRY SOON"));
        delay(1000);
    }
    Serial.println(F("SD CARD UP!"));
    timeoutCopy_ = SplitWord32(Serial.getTimeout());
    clusterCount_ = SplitWord32(SD.clusterCount());
    volumeSectorCount_ = SplitWord32(SD.volumeSectorCount());
    bytesPerSector_ = SD.bytesPerSector();
#define X(thing, base, type) Serial.print(F("Address of " #thing ": 0x")); \
        Serial.print(base + static_cast<byte>(type :: thing ), HEX); \
        Serial.print(F(", ("));                                \
        Serial.print(base + static_cast<byte>(type ::thing));        \
        Serial.println(F(")"));
#define P0(thing) X(thing, RegisterPage0BaseAddress, Registers)
#define P1(thing) X(thing, SDCardInterfaceBaseAddress, SDCardFileSystemRegisters)
#define P34(thing) X(thing, DisplayShieldBaseAddress, DisplayShieldRegisters)
    P0(ConsoleIO);
    P0(ConsoleFlush);
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
    P1(ErrorCode);
    P1(MakeMissingParentDirectories);
    P1(OpenReadWrite);
    P34(Backlight);
    P34(RawButtonsLower);
    P34(RawButtonsUpper);
#undef P0
#undef P1
#undef P34
#undef X
    for (uint32_t i = SDCardFileInterfaceBlockBaseAddress, j = 0; i < SDCardFileInterfaceBlockEndAddress; i += 0x100, ++j) {
        Serial.print(F("File "));
        Serial.print(j);
        Serial.print(F(" @ 0x"));
        Serial.println(i, HEX);
    }
}
