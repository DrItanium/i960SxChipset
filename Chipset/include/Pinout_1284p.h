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

#ifndef CHIPSET_PINOUT_1284P_H
#define CHIPSET_PINOUT_1284P_H
#include <Arduino.h>
#ifdef ARDUINO_AVR_ATmega1284
/// @todo fix this pinout for different targets
enum class i960Pinout {
    PORT_B0 = 0,
    PORT_B1,
    PORT_B2,
    PORT_B3,
    PORT_B4,
    PORT_B5,
    PORT_B6,
    PORT_B7,
    PORT_D0,
    PORT_D1,
    PORT_D2,
    PORT_D3,
    PORT_D4,
    PORT_D5,
    PORT_D6,
    PORT_D7,
    PORT_C0,
    PORT_C1,
    PORT_C2,
    PORT_C3,
    PORT_C4,
    PORT_C5,
    PORT_C6,
    PORT_C7,
    PORT_A0,
    PORT_A1,
    PORT_A2,
    PORT_A3,
    PORT_A4,
    PORT_A5,
    PORT_A6,
    PORT_A7,
    Count,
    // this is described in digial pin order!
    // leave this one alone
    // PORT B
    Led = PORT_B0,
    Ready = PORT_B3,
    GPIOSelect = PORT_B4,        // output
    MOSI = PORT_B5,          // reserved
    MISO = PORT_B6,          // reserved
    SCK = PORT_B7,          // reserved
// PORT D
    RX0 = PORT_D0,          // reserved
    TX0 = PORT_D1,          // reserved
    NEW_REQUEST_ = PORT_D2,      // AVR Interrupt INT0
    //BOOT_NORMAL_ = PORT_D3,     // AVR Interrupt INT1
    SYSTEM_FAIL_ = PORT_D4,
// PORT C
    SCL = PORT_C0,          // reserved
    SDA = PORT_C1,          // reserved
    DISPLAY_EN = PORT_C2,
    DC = PORT_C3,
    SD_EN = PORT_C4,
    PSRAM_EN = PORT_C5,
// PORT A, used to select the spi bus address (not directly used)
    W_R_ = PORT_A0,
    BA1 = PORT_A1,
    BA2 = PORT_A2,
    BA3 = PORT_A3,
    BE0 = PORT_A4,
    BE1 = PORT_A5,
    NEW_ADDRESS_ = PORT_A6,
};
template<i960Pinout pin>
[[nodiscard]] inline volatile unsigned char& getAssociatedOutputPort() noexcept {
    //static_assert(isValidPin<pin>, "INVALID PIN PROVIDED");
    switch (pin) {
        case i960Pinout::PORT_A0:
        case i960Pinout::PORT_A1:
        case i960Pinout::PORT_A2:
        case i960Pinout::PORT_A3:
        case i960Pinout::PORT_A4:
        case i960Pinout::PORT_A5:
        case i960Pinout::PORT_A6:
        case i960Pinout::PORT_A7:
            return PORTA;
        case i960Pinout::PORT_C0:
        case i960Pinout::PORT_C1:
        case i960Pinout::PORT_C2:
        case i960Pinout::PORT_C3:
        case i960Pinout::PORT_C4:
        case i960Pinout::PORT_C5:
        case i960Pinout::PORT_C6:
        case i960Pinout::PORT_C7:
            return PORTC;
        case i960Pinout::PORT_D0:
        case i960Pinout::PORT_D1:
        case i960Pinout::PORT_D2:
        case i960Pinout::PORT_D3:
        case i960Pinout::PORT_D4:
        case i960Pinout::PORT_D5:
        case i960Pinout::PORT_D6:
        case i960Pinout::PORT_D7:
            return PORTD;
        case i960Pinout::PORT_B0:
        case i960Pinout::PORT_B1:
        case i960Pinout::PORT_B2:
        case i960Pinout::PORT_B3:
        case i960Pinout::PORT_B4:
        case i960Pinout::PORT_B5:
        case i960Pinout::PORT_B6:
        case i960Pinout::PORT_B7:
            return PORTB;
        default:
            return PORTA;
    }
}
template<i960Pinout pin>
[[nodiscard]] inline volatile unsigned char& getAssociatedInputPort() noexcept {
    //static_assert(isValidPin<pin>, "INVALID PIN PROVIDED");
    switch (pin) {
        case i960Pinout::PORT_A0:
        case i960Pinout::PORT_A1:
        case i960Pinout::PORT_A2:
        case i960Pinout::PORT_A3:
        case i960Pinout::PORT_A4:
        case i960Pinout::PORT_A5:
        case i960Pinout::PORT_A6:
        case i960Pinout::PORT_A7:
            return PINA;
        case i960Pinout::PORT_C0:
        case i960Pinout::PORT_C1:
        case i960Pinout::PORT_C2:
        case i960Pinout::PORT_C3:
        case i960Pinout::PORT_C4:
        case i960Pinout::PORT_C5:
        case i960Pinout::PORT_C6:
        case i960Pinout::PORT_C7:
            return PINC;
        case i960Pinout::PORT_D0:
        case i960Pinout::PORT_D1:
        case i960Pinout::PORT_D2:
        case i960Pinout::PORT_D3:
        case i960Pinout::PORT_D4:
        case i960Pinout::PORT_D5:
        case i960Pinout::PORT_D6:
        case i960Pinout::PORT_D7:
            return PIND;
        case i960Pinout::PORT_B0:
        case i960Pinout::PORT_B1:
        case i960Pinout::PORT_B2:
        case i960Pinout::PORT_B3:
        case i960Pinout::PORT_B4:
        case i960Pinout::PORT_B5:
        case i960Pinout::PORT_B6:
        case i960Pinout::PORT_B7:
            return PINB;
        default:
            return PINA;
    }
}
template<i960Pinout pin>
[[nodiscard]] constexpr decltype(auto) getPinMask() noexcept {
#if 0
    static_assert(isValidPin<pin>, "INVALID PIN PROVIDED");
#endif
    switch (pin) {
        case i960Pinout::PORT_A0: return _BV(PA0);
        case i960Pinout::PORT_A1: return _BV(PA1);
        case i960Pinout::PORT_A2: return _BV(PA2);
        case i960Pinout::PORT_A3: return _BV(PA3);
        case i960Pinout::PORT_A4: return _BV(PA4);
        case i960Pinout::PORT_A5: return _BV(PA5);
        case i960Pinout::PORT_A6: return _BV(PA6);
        case i960Pinout::PORT_A7: return _BV(PA7);
        case i960Pinout::PORT_C0: return _BV(PC0);
        case i960Pinout::PORT_C1: return _BV(PC1);
        case i960Pinout::PORT_C2: return _BV(PC2);
        case i960Pinout::PORT_C3: return _BV(PC3);
        case i960Pinout::PORT_C4: return _BV(PC4);
        case i960Pinout::PORT_C5: return _BV(PC5);
        case i960Pinout::PORT_C6: return _BV(PC6);
        case i960Pinout::PORT_C7: return _BV(PC7);
        case i960Pinout::PORT_D0: return _BV(PD0);
        case i960Pinout::PORT_D1: return _BV(PD1);
        case i960Pinout::PORT_D2: return _BV(PD2);
        case i960Pinout::PORT_D3: return _BV(PD3);
        case i960Pinout::PORT_D4: return _BV(PD4);
        case i960Pinout::PORT_D5: return _BV(PD5);
        case i960Pinout::PORT_D6: return _BV(PD6);
        case i960Pinout::PORT_D7: return _BV(PD7);
        case i960Pinout::PORT_B0: return _BV(PB0);
        case i960Pinout::PORT_B1: return _BV(PB1);
        case i960Pinout::PORT_B2: return _BV(PB2);
        case i960Pinout::PORT_B3: return _BV(PB3);
        case i960Pinout::PORT_B4: return _BV(PB4);
        case i960Pinout::PORT_B5: return _BV(PB5);
        case i960Pinout::PORT_B6: return _BV(PB6);
        case i960Pinout::PORT_B7: return _BV(PB7);
        default:
            return 0xFF;
    }
}
template<i960Pinout pin>
inline void pulse() noexcept {
        // save registers and do the pulse
        uint8_t theSREG = SREG;
        cli();
        auto &thePort = getAssociatedOutputPort<pin>();
        thePort ^= getPinMask<pin>();
        thePort ^= getPinMask<pin>();
        SREG = theSREG;
}
template<i960Pinout pin, decltype(HIGH) value>
inline void digitalWrite() noexcept {
    uint8_t theSREG = SREG;
    cli();
    auto& thePort = getAssociatedOutputPort<pin>();
    if constexpr (value == LOW) {
        thePort &= ~getPinMask<pin>();
    } else {
        thePort |= getPinMask<pin>();
    }
    SREG = theSREG;
}
template<i960Pinout pin>
inline void digitalWrite(decltype(HIGH) value) noexcept {
    uint8_t theSREG = SREG;
    cli();
    auto& thePort = getAssociatedOutputPort<pin>();
    if (value == LOW) {
        thePort &= ~getPinMask<pin>();
    } else {
        thePort |= getPinMask<pin>();
    }
    SREG = theSREG;
}
template<i960Pinout pin>
inline auto digitalRead() noexcept {
    return (getAssociatedInputPort<pin>() & getPinMask<pin>()) ? HIGH : LOW;
}
#endif
#endif //CHIPSET_PINOUT_1284P_H
