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
// Created by jwscoggins on 3/30/21.
//

#ifndef ARDUINO_BOARD_GRANDCENTRALM4_H
#define ARDUINO_BOARD_GRANDCENTRALM4_H

#include <Arduino.h>
#include <SdFat.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_SPIFlash.h>
#include "BoardSpecific.h"

constexpr auto OnboardNeoPixelPin = 88;
constexpr auto OnboardNeoPixelCount = 1;
enum class i960Pinout : decltype(A0) {
    Ready = 0,
    Int0_,
    DEN_,
    SPI_BUS_EN,
    W_R_,
    Reset960,
    // Software Serial, can be reused
    RX0,
    TX0,
    Pin_MOSI,
    Pin_MISO,
    Pin_SCK,
    GPIOSelect,
    BLAST_,
    FAIL,
    AS_,
    MCU_RESET,
    Count,
};
static_assert(static_cast<int>(i960Pinout::Count) == 16);
extern Adafruit_NeoPixel onboardPixel;
#define SOFTWARE_IS_SERIAL
#endif //ARDUINO_BOARD_GRANDCENTRALM4_H
