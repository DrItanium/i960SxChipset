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

constexpr auto OnboardNeoPixelPin = PIN_NEOPIXEL;
constexpr auto OnboardNeoPixelCount = 1;
/// @todo implement proper pinout

enum class i960Pinout : decltype(A0) {
    RX0 = 0,
    TX0,
    Digital2,
    Digital3,
    Digital4,
    Digital5,
    Digital6,
    Digital7,
    Digital8,
    Digital9,
    Digital10,
    Digital11,
    Digital12,
    Digital13,
    Digital14,
    Digital15,
    Digital16,
    Digital17,
    Digital18,
    Digital19,
    Digital20,
    Digital21,
    Digital22,
    Digital23,
    Digital24,
    Digital25,
    Digital26,
    Digital27,
    Digital28,
    Digital29,
    Digital30,
    Digital31,
    Digital32,
    Digital33,
    Digital34,
    Digital35,
    Digital36,
    Digital37,
    Digital38,
    Digital39,
    Digital40,
    Digital41,
    Digital42,
    Digital43,
    Digital44,
    Digital45,
    Digital46,
    Digital47,
    Digital48,
    Digital49,
    MISO,
    MOSI,
    SCK,
    GPIOSelect,
    Analog8, // 54
    Analog9,
    Analog10,
    Analog11,
    Analog12,
    Analog13,
    Analog14,
    Analog15,
    SDA,
    SCL,
    ICSP_MISO,
    ICSP_MOSI,
    ICSP_SCK,
    Analog0, // 67
    Analog1,
    Analog2,
    Analog3,
    Analog4,
    Analog5,
    Analog6,
    Analog7,
    RXLed,
    TXLed,
    USBHostEnable,
    USB_DM,
    USB_DP,
    SD_SPI_INTERFACE0,
    SD_SPI_INTERFACE1,
    SD_SPI_INTERFACE2,
    SD_SPI_INTERFACE3,
    DAC_AREF,
    DAC_VOUT0,
    DAC_VOUT1,
    LED_DUPLICATE,
    InternalNeoPixel,
    QSPI0,
    QSPI1,
    QSPI2,
    QSPI3,
    QSPI4,
    QSPI5,
    SD_Detect,
    SWO,
    Count,

    AS_,
    DEN_,
    Ready,
    Int0_,
    W_R_,
    Reset960,
    BLAST_,
    FAIL,
    SPI_BUS_EN,
    Led = LED_BUILTIN,
};
static_assert(static_cast<int>(i960Pinout::Count) == 97);
extern Adafruit_NeoPixel onboardPixel;
extern Adafruit_FlashTransport_QSPI flashTransport;
extern Adafruit_SPIFlash onboardFlash;
extern SdFat onboardSDCard;
#define SOFTWARE_IS_SERIAL
#endif //ARDUINO_BOARD_GRANDCENTRALM4_H
