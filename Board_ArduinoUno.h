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

#ifndef ARDUINO_BOARD_ARDUINOUNO_H
#define ARDUINO_BOARD_ARDUINOUNO_H
#include <Arduino.h>
#include "BoardSpecific.h"
#include <Adafruit_GFX.h>
#include <SD.h>
enum class i960Pinout : decltype(A0) {
    RX0 = 0,
    TX0,
    AS_,
    DEN_,
    Ready,
    Int0_,
    W_R_,
    Reset960,
    BLAST_,
    FAIL,
    GPIOSelect,
    MOSI,
    MISO,
    SCK,
    SPI_BUS_EN,
    Analog1,
    Analog2,
    Analog3,
    Analog4,
    Analog5,
    Count,
    SCL = A5,
    SDA = A4,
    Led = LED_BUILTIN,
};
static_assert(static_cast<int>(i960Pinout::Count) == 20);
#define AS_ISR INT0_vect
#define DEN_ISR INT1_vect
#endif //ARDUINO_BOARD_ARDUINOUNO_H
