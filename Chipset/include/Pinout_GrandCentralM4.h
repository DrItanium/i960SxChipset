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
// Created by jwscoggins on 7/19/21.
//

#ifndef CHIPSET_PINOUT_GRANDCENTRALM4_H
#define CHIPSET_PINOUT_GRANDCENTRALM4_H
#include <Arduino.h>
#ifdef ARDUINO_GRAND_CENTRAL_M4
enum class i960Pinout {
    /// @todo fix this
    AS_,     // input, AVR Int2
    DEN_,      // AVR Interrupt INT0
    Int0_,          // output
    SPI_BUS_EN, // output
    Reset960,          // output
    BLAST_,     // input
    Ready,      // output
    GPIOSelect = ::SS,        // output
    MOSI = ::MOSI,          // reserved
    MISO = ::MOSI,          // reserved
    SCK = ::SCK,          // reserved
    AIRLIFT_SD_CS = A5,
    AIRLIFT_RESET = 5,
    AIRLIFT_BUSY = A4,
    AIRLIFT_CS = A3,
    BLUEFRUIT_RESET = A2,
    BLUEFRUIT_IRQ = A1,
    BLUEFRUIT_CS = A0,
    SD_EN = SDCARD_SS_PIN,
    DISPLAY_SD_EN = 4,
    DISPLAY_DC = 8,
    DISPLAY_EN = 10,

// PORT C
    SCL = ::SCL,          // reserved
    SDA = ::SDA,          // reserved
    W_R_,          // input
    FAIL,         // input
    BA1,
    BA2,
    BA3,
    BE0,
    BE1,
    Count, // must be last
};
template<i960Pinout pin>
inline void pulse() noexcept {
    /// @todo implement this
#warning "PULSE DOES NOTHING!"
}
template<i960Pinout pin>
inline void toggle() noexcept {
#warning "TOGGLE DOES NOTHING!"
}
template<i960Pinout pin, decltype(HIGH) value>
inline void digitalWrite() noexcept {
#warning "digitalWrite<pin,value>() routed to normal arduino function!"
    digitalWrite(static_cast<int>(pin), value);
}
template<i960Pinout pin>
inline void digitalWrite(decltype(HIGH) value) noexcept {
#warning "digitalWrite<pin>(value) routed to normal arduino function!"
    digitalWrite(static_cast<int>(pin), value);

}
template<i960Pinout pin>
inline auto digitalRead() noexcept {
#warning "digitalRead<pin>() routed to normal arduino function!"
    return digitalRead(static_cast<int>(pin));
}
#endif
#endif //CHIPSET_PINOUT_GRANDCENTRALM4_H
