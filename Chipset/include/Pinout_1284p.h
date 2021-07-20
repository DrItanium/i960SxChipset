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
    // this is described in digial pin order!
    // leave this one alone
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
    Count,          // special, must be last
};
static_assert(isValidPin<i960Pinout::CACHE_A0>, "The CACHE_A0 pin should be a valid pin!");
template<i960Pinout pin>
[[nodiscard]] inline volatile unsigned char& getAssociatedOutputPort() noexcept {
    static_assert(isValidPin<pin>, "INVALID PIN PROVIDED");
    switch (pin) {
        case i960Pinout::WR2:
        case i960Pinout::BA1:
        case i960Pinout::BA2:
        case i960Pinout::BA3:
        case i960Pinout::BE0:
        case i960Pinout::BE1:
        case i960Pinout::BLAST2:
        case i960Pinout::SPI_BUS_A7:
            return PORTA;
        case i960Pinout::SCL:          // reserved
        case i960Pinout::SDA:          // reserved
        case i960Pinout::Ready:      // output
        case i960Pinout::Int0_:          // output
        case i960Pinout::W_R_:          // input
        case i960Pinout::Reset960:          // output
        case i960Pinout::BLAST_:     // input
        case i960Pinout::FAIL:         // input
            return PORTC;
        case i960Pinout::CACHE_A0:      // output
        case i960Pinout::CLOCK_OUT: // output: unusable
        case i960Pinout::AS_:     // input: AVR Int2
        case i960Pinout::CACHE_A1: // unused
        case i960Pinout::GPIOSelect:        // output
        case i960Pinout::MOSI:          // reserved
        case i960Pinout::MISO:          // reserved
        case i960Pinout::SCK:          // reserved
            return PORTB;
        case i960Pinout::RX0:          // reserved
        case i960Pinout::TX0:          // reserved
        case i960Pinout::DEN_:      // AVR Interrupt INT0
        case i960Pinout::CACHE_A2:        // AVR Interrupt INT1
        case i960Pinout::SPI_BUS_EN: // output
        case i960Pinout::DC:     // output
        case i960Pinout::DISPLAY_EN: // output
        case i960Pinout::SD_EN:      // output
            return PORTD;
        default:
            return PORTA;
    }
}

template<i960Pinout pin>
[[nodiscard]] inline volatile unsigned char& getAssociatedInputPort() noexcept {
    static_assert(isValidPin<pin>, "INVALID PIN PROVIDED");
    switch (pin) {
        case i960Pinout::WR2:
        case i960Pinout::BA1:
        case i960Pinout::BA2:
        case i960Pinout::BA3:
        case i960Pinout::BE0:
        case i960Pinout::BE1:
        case i960Pinout::BLAST2:
        case i960Pinout::SPI_BUS_A7:
            return PINA;
        case i960Pinout::SCL:          // reserved
        case i960Pinout::SDA:          // reserved
        case i960Pinout::Ready:      // output
        case i960Pinout::Int0_:          // output
        case i960Pinout::W_R_:          // input
        case i960Pinout::Reset960:          // output
        case i960Pinout::BLAST_:     // input
        case i960Pinout::FAIL:         // input
            return PINC;
        case i960Pinout::CACHE_A0:      // output
        case i960Pinout::CLOCK_OUT: // output: unusable
        case i960Pinout::AS_:     // input: AVR Int2
        case i960Pinout::CACHE_A1: // unused
        case i960Pinout::GPIOSelect:        // output
        case i960Pinout::MOSI:          // reserved
        case i960Pinout::MISO:          // reserved
        case i960Pinout::SCK:          // reserved
            return PINB;
        case i960Pinout::RX0:          // reserved
        case i960Pinout::TX0:          // reserved
        case i960Pinout::DEN_:      // AVR Interrupt INT0
        case i960Pinout::CACHE_A2:        // AVR Interrupt INT1
        case i960Pinout::SPI_BUS_EN: // output
        case i960Pinout::DC:     // output
        case i960Pinout::DISPLAY_EN: // output
        case i960Pinout::SD_EN:      // output
            return PIND;
        default:
            return PORTA;
    }
}
template<i960Pinout pin>
[[nodiscard]] constexpr decltype(auto) getPinMask() noexcept {
    static_assert(isValidPin<pin>, "INVALID PIN PROVIDED");
    switch (pin) {
        case i960Pinout::WR2: return _BV(PA0) ;
        case i960Pinout::BA1: return _BV(PA1);
        case i960Pinout::BA2: return _BV(PA2);
        case i960Pinout::BA3: return _BV(PA3);
        case i960Pinout::BE0: return _BV(PA4);
        case i960Pinout::BE1: return _BV(PA5);
        case i960Pinout::BLAST2: return _BV(PA6);
        case i960Pinout::SPI_BUS_A7: return _BV(PA7);
        case i960Pinout::SCL:        return _BV(PC0);
        case i960Pinout::SDA:        return _BV(PC1);
        case i960Pinout::Ready:      return _BV(PC2);
        case i960Pinout::Int0_:      return _BV(PC3);
        case i960Pinout::W_R_:       return _BV(PC4);
        case i960Pinout::Reset960:   return _BV(PC5);
        case i960Pinout::BLAST_:     return _BV(PC6);
        case i960Pinout::FAIL:       return _BV(PC7);

        case i960Pinout::CACHE_A0:      return _BV(PB0);
        case i960Pinout::CLOCK_OUT:  return _BV(PB1);
        case i960Pinout::AS_:     return _BV(PB2);
        case i960Pinout::CACHE_A1:    return _BV(PB3);
        case i960Pinout::GPIOSelect:        return _BV(PB4);
        case i960Pinout::MOSI:         return _BV(PB5) ;
        case i960Pinout::MISO:         return _BV(PB6) ;
        case i960Pinout::SCK:          return _BV(PB7);

        case i960Pinout::RX0:          return _BV(PD0);
        case i960Pinout::TX0:          return _BV(PD1);
        case i960Pinout::DEN_:          return _BV(PD2);
        case i960Pinout::CACHE_A2:        return _BV(PD3);
        case i960Pinout::SPI_BUS_EN: return _BV(PD4);
        case i960Pinout::DC:     return _BV(PD5);
        case i960Pinout::DISPLAY_EN: return _BV(PD6);
        case i960Pinout::SD_EN:     return _BV(PD7);
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
template<i960Pinout pin>
inline void toggle() noexcept {
    auto& thePort = getAssociatedInputPort<pin>();
    thePort |= getPinMask<pin>();
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
