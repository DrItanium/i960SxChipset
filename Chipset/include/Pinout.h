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

#ifndef ARDUINO_PINOUT_H
#define ARDUINO_PINOUT_H
#include <Arduino.h>
#include "MCUPlatform.h"
using Address = uint32_t;
/**
 * @brief Sx Load/Store styles that the processor will request
 */
enum class LoadStoreStyle : uint8_t {
    // based off of BE0,BE1 pins
    Full16 = 0b00,
    Upper8 = 0b01,
    Lower8 = 0b10,
    None = 0b11,
};
#include "Pinout_1284p.h"
#include "Pinout_GrandCentralM4.h"
template<i960Pinout pin>
constexpr bool isValidPin = static_cast<byte>(pin) < static_cast<byte>(i960Pinout::Count);
static_assert(!isValidPin<i960Pinout::Count>, "The Count \"pin\" should be an invalid pin!");
#ifdef __AVR__
static_assert(isValidPin<i960Pinout::CACHE_A0>, "The CACHE_A0 pin should be a valid pin!");
#endif
#ifdef __AVR__
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
#endif

template<i960Pinout pin>
inline void pulse() noexcept {
#ifdef __AVR__
        // save registers and do the pulse
        uint8_t theSREG = SREG;
        cli();
        auto &thePort = getAssociatedOutputPort<pin>();
        thePort ^= getPinMask<pin>();
        thePort ^= getPinMask<pin>();
        SREG = theSREG;
#else
#warning "PULSE DOES NOTHING!"
#endif

}
template<i960Pinout pin>
inline void toggle() noexcept {
#ifdef __AVR__
    auto& thePort = getAssociatedInputPort<pin>();
    thePort |= getPinMask<pin>();
#else
#warning "TOGGLE DOES NOTHING!"
#endif
}

template<i960Pinout pin, decltype(HIGH) value>
inline void digitalWrite() {
#ifdef __AVR__
    uint8_t theSREG = SREG;
    cli();
    auto& thePort = getAssociatedOutputPort<pin>();
    if constexpr (value == LOW) {
        thePort &= ~getPinMask<pin>();
    } else {
        thePort |= getPinMask<pin>();
    }
    SREG = theSREG;
#else
#warning "digitalWrite<pin,value>() routed to normal arduino function!"
    digitalWrite(static_cast<int>(pin), value);
#endif
}
template<i960Pinout pin>
inline void digitalWrite(decltype(HIGH) value) noexcept {
#ifdef __AVR__
    uint8_t theSREG = SREG;
    cli();
    auto& thePort = getAssociatedOutputPort<pin>();
    if (value == LOW) {
        thePort &= ~getPinMask<pin>();
    } else {
        thePort |= getPinMask<pin>();
    }
    SREG = theSREG;
#else
#warning "digitalWrite<pin>(value) routed to normal arduino function!"
    digitalWrite(static_cast<int>(pin), value);
#endif

}

inline void digitalWrite(i960Pinout ip, decltype(HIGH) value) {
    digitalWrite(static_cast<int>(ip), value);
}

inline void pinMode(i960Pinout ip, decltype(INPUT) value) {
    pinMode(static_cast<int>(ip), value);
}
template<i960Pinout pin>
inline auto digitalRead() noexcept {
#ifdef __AVR__
    return (getAssociatedInputPort<pin>() & getPinMask<pin>()) ? HIGH : LOW;
#else
#warning "digitalRead<pin>() routed to normal arduino function!"
    return digitalRead(static_cast<int>(pin));
#endif
}
inline auto digitalRead(i960Pinout ip) {
    return digitalRead(static_cast<int>(ip));
}

template<i960Pinout pin>
struct DigitalPin {
    DigitalPin() = delete;
    ~DigitalPin() = delete;
    DigitalPin(const DigitalPin&) = delete;
    DigitalPin(DigitalPin&&) = delete;
    DigitalPin& operator=(const DigitalPin&) = delete;
    DigitalPin& operator=(DigitalPin&&) = delete;
    static constexpr bool isInputPin() noexcept { return false; }
    static constexpr bool isOutputPin() noexcept { return false; }
    static constexpr bool getDirection() noexcept { return false; }
    static constexpr auto getPin() noexcept { return pin; }
};

#define DefOutputPin(pin, asserted, deasserted) \
    template<> \
    struct DigitalPin< pin > { \
    static_assert(asserted != deasserted, "Asserted and deasserted must not be equal!"); \
        DigitalPin() = delete; \
        ~DigitalPin() = delete; \
        DigitalPin(const DigitalPin&) = delete; \
        DigitalPin(DigitalPin&&) = delete; \
        DigitalPin& operator=(const DigitalPin&) = delete; \
        DigitalPin& operator=(DigitalPin&&) = delete; \
        static constexpr auto isInputPin() noexcept { return false; } \
        static constexpr auto isOutputPin() noexcept { return true; } \
        static constexpr auto getPin() noexcept { return pin; } \
        static constexpr auto getDirection() noexcept { return OUTPUT; } \
        static constexpr auto getAssertionState() noexcept { return asserted; } \
        static constexpr auto getDeassertionState() noexcept { return deasserted; } \
        inline static void assertPin() noexcept { digitalWrite<pin,getAssertionState()>(); } \
        inline static void deassertPin() noexcept { digitalWrite<pin,getDeassertionState()>(); } \
        inline static void write(decltype(LOW) value) noexcept { digitalWrite<pin>(value); } \
        inline static void pulse() noexcept {   \
            ::pulse<pin>();                                     \
        }                                       \
        inline static void togglePin() noexcept { \
            ::toggle<pin>(); \
        } \
    }
#define DefInputPin(pin, asserted, deasserted) \
    template<> \
    struct DigitalPin< pin > { \
        static_assert(asserted != deasserted, "Asserted and deasserted must not be equal!"); \
        DigitalPin() = delete; \
        ~DigitalPin() = delete; \
        DigitalPin(const DigitalPin&) = delete; \
        DigitalPin(DigitalPin&&) = delete; \
        DigitalPin& operator=(const DigitalPin&) = delete; \
        DigitalPin& operator=(DigitalPin&&) = delete; \
        static constexpr auto isInputPin() noexcept { return true; } \
        static constexpr auto isOutputPin() noexcept { return false; } \
        static constexpr auto getPin() noexcept { return pin; } \
        static constexpr auto getDirection() noexcept { return INPUT; } \
        static constexpr auto getAssertionState() noexcept { return asserted; } \
        static constexpr auto getDeassertionState() noexcept { return deasserted; } \
        inline static auto read() noexcept { return digitalRead<pin>(); } \
        inline static bool isAsserted() noexcept { return read() == getAssertionState(); } \
        inline static bool isDeasserted() noexcept { return read() == getDeassertionState(); } \
    }

DefOutputPin(i960Pinout::GPIOSelect, LOW, HIGH);
DefOutputPin(i960Pinout::Reset960, LOW, HIGH);
DefOutputPin(i960Pinout::Ready, LOW, HIGH);
DefOutputPin(i960Pinout::SPI_BUS_EN, LOW, HIGH);
DefOutputPin(i960Pinout::DISPLAY_EN, LOW, HIGH);
DefOutputPin(i960Pinout::SD_EN, LOW, HIGH);
DefInputPin(i960Pinout::FAIL, HIGH, LOW);
DefInputPin(i960Pinout::DEN_, LOW, HIGH);
DefInputPin(i960Pinout::AS_, LOW, HIGH);
DefInputPin(i960Pinout::BLAST_, LOW, HIGH);
DefInputPin(i960Pinout::W_R_, LOW, HIGH);
#undef DefInputPin
#undef DefOutputPin

template<typename ... Pins>
inline void setupPins(decltype(OUTPUT) direction, Pins ... pins) {
    (pinMode(pins, direction), ...);
}

template<typename ... Pins>
inline void digitalWriteBlock(decltype(HIGH) value, Pins ... pins) {
    (digitalWrite(pins, value), ...);
}

template<i960Pinout pinId>
class PinAsserter {
public:
    static_assert(DigitalPin<pinId>::isOutputPin());
    PinAsserter() { DigitalPin<pinId>::assertPin(); }
    ~PinAsserter() { DigitalPin<pinId>::deassertPin(); }
};


#endif //ARDUINO_PINOUT_H
