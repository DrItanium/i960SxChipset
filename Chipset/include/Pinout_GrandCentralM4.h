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
    // PORT B
    CACHE_A0 = 0,      // output
    CLOCK_OUT, // output, unusable
    AS_,     // input, AVR Int2
    CACHE_A1, // output
    GPIOSelect,        // output
    MOSI,          // reserved
    MISO,          // reserved
    SCK,          // reserved
// PORT D
    RX0,          // reserved
    TX0,          // reserved
    DEN_,      // AVR Interrupt INT0
    CACHE_A2,        // Output, AVR Interrupt INT1
    SPI_BUS_EN, // output
    DC,     // output
    DISPLAY_EN, // output
    SD_EN,      // output
// PORT C
    SCL,          // reserved
    SDA,          // reserved
    Ready,      // output
    Int0_,          // output
    W_R_,          // input
    Reset960,          // output
    BLAST_,     // input
    FAIL,         // input
// PORT A, used to select the spi bus address (not directly used)
    WR2,
    BA1,
    BA2,
    BA3,
    BE0,
    BE1,
    BLAST2,
    SPI_BUS_A7,
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
inline void digitalWrite() {
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
