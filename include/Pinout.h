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
#if 0
    Full16 = 0b00,
    Upper8 = 0b01,
    Lower8 = 0b10,
    None = 0b11,
#else
// no need to shift
    Full16 = 0b0000'0000,
    Upper8 = 0b0001'0000,
    Lower8 = 0b0010'0000,
    None = 0b0011'0000,
#endif
};
/// @todo fix this pinout for different targets
enum class i960Pinout : int {
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
    // current type 1 pinout that will evolve into type 3 once the hardware design is complete
#ifdef CHIPSET_TYPE1
    Ready = PORT_B0,
    CLOCK_OUT= PORT_B1,
    AS_ = PORT_B2,
    PSRAM_EN = PORT_B3,
    GPIOSelect = PORT_B4,
    MOSI = PORT_B5,
    MISO = PORT_B6,
    SCK = PORT_B7,
    RX0 = PORT_D0,
    TX0 = PORT_D1,
    DEN_ = PORT_D2,
    Reset960 = PORT_D5,
    Int0_ = PORT_D6,
    SCL = PORT_C0,
    SDA = PORT_C1,
    SPI_OFFSET0 = PORT_C2,
    SPI_OFFSET1 = PORT_C3,
    SPI_OFFSET2 = PORT_C4,
    SD_EN = PORT_C7,
    W_R_ = PORT_A0,
    BA1 = PORT_A1,
    BA2 = PORT_A2,
    BA3 = PORT_A3,
    BE0 = PORT_A4,
    BE1 = PORT_A5,
    BLAST_ = PORT_A6,
    FAIL = PORT_A7,
#elif defined(CHIPSET_TYPE3)
    FAIL = PORT_B0,
    CLOCK_OUT = PORT_B1,
    DAZZLER_INT = PORT_B2,
    // PB3 free for use
    GPIO_CS0 = PORT_B4,
    MOSI0 = PORT_B5,
    MISO0 = PORT_B6,
    SCK0 = PORT_B7,
    RX0 = PORT_D0,
    TX0 = PORT_D1,
    MISO1 = PORT_D2,
    MOSI1 = PORT_D3,
    SCK1 = PORT_D4,
    GPIO_CS1 = PORT_D5,
    // PD6 and PD7 are connected to the arduino breakout
    SCL = PORT_C0,
    SDA = PORT_C1,
    PSRAM_EN0 = PORT_C2,
    PSRAM_EN1 = PORT_C3,
    SD_EN = PORT_C4,
    GPU_EN = PORT_C5,
    DAZZLER_EN = PORT_C6,
    Ready = PORT_C7,
    // PA0-PA3 free for use
    BE0 = PORT_A4,
    BE1 = PORT_A5,
    BLAST_ = PORT_A6,
    DEN_ = PORT_A7,
    // compat
    GPIOSelect = GPIO_CS0,
#else
#error "Unknown pinout type!"
#endif
};

inline void digitalWrite(i960Pinout ip, decltype(HIGH) value) {
    digitalWrite(static_cast<int>(ip), value);
}

inline void pinMode(i960Pinout ip, decltype(INPUT) value) {
    pinMode(static_cast<int>(ip), value);
}
inline auto digitalRead(i960Pinout ip) {
    return digitalRead(static_cast<int>(ip));
}
template<i960Pinout pin>
constexpr auto isValidPin960_v = static_cast<int>(pin) < static_cast<int>(i960Pinout::Count);
//static_assert(isValidPin<i960Pinout::CACHE_A0>, "The CACHE_A0 pin should be a valid pin!");
template<i960Pinout pin>
[[nodiscard]] inline volatile unsigned char& getAssociatedOutputPort() noexcept {
    static_assert(isValidPin960_v<pin>, "INVALID PIN PROVIDED");
    switch (pin) {
#define X(id, number) case i960Pinout:: PORT_ ## id ## number
#define Y(id) \
    X(id, 0): \
    X(id, 1): \
    X(id, 2): \
    X(id, 3): \
    X(id, 4): \
    X(id, 5): \
    X(id, 6): \
    X(id, 7)
        Y(A): return PORTA;
        Y(C): return PORTC;
        Y(D): return PORTD;
        Y(B): return PORTB;
#undef Y
#undef X

        default:
            return PORTA;
    }
}

template<i960Pinout pin>
[[nodiscard]] inline volatile unsigned char& getAssociatedDirectionPort() noexcept {
    static_assert(isValidPin960_v<pin>, "INVALID PIN PROVIDED");
    switch (pin) {
#define X(id, number) case i960Pinout:: PORT_ ## id ## number
#define Y(id) \
    X(id, 0): \
    X(id, 1): \
    X(id, 2): \
    X(id, 3): \
    X(id, 4): \
    X(id, 5): \
    X(id, 6): \
    X(id, 7)
        Y(A): return DDRA;
        Y(C): return DDRC;
        Y(D): return DDRD;
        Y(B): return DDRB;
#undef Y
#undef X

        default:
            return DDRA;
    }
}

template<i960Pinout pin>
[[nodiscard]] inline volatile unsigned char& getAssociatedInputPort() noexcept {
    static_assert(isValidPin960_v<pin>, "INVALID PIN PROVIDED");
    switch (pin) {
#define X(id, number) case i960Pinout:: PORT_ ## id ## number
#define Y(id) \
    X(id, 0): \
    X(id, 1): \
    X(id, 2): \
    X(id, 3): \
    X(id, 4): \
    X(id, 5): \
    X(id, 6): \
    X(id, 7)
        Y(A): return PINA;
        Y(C): return PINC;
        Y(D): return PIND;
        Y(B): return PINB;
#undef Y
#undef X
        default:
            return PINA;
    }
}
template<i960Pinout pin>
[[nodiscard]] constexpr decltype(auto) getPinMask() noexcept {
    static_assert(isValidPin960_v<pin>, "INVALID PIN PROVIDED");
    switch (pin) {
#define X(id, number) case i960Pinout:: PORT_ ## id ## number : return _BV ( P ## id ## number )
#define Y(id) \
    X(id, 0); \
    X(id, 1); \
    X(id, 2); \
    X(id, 3); \
    X(id, 4); \
    X(id, 5); \
    X(id, 6); \
    X(id, 7)
        Y(A);
        Y(C);
        Y(D);
        Y(B);
#undef Y
#undef X
        default:
            return 0xFF;
    }
}

class InterruptDisabler final {
public:
    InterruptDisabler() noexcept {
        storage_ = SREG;
        cli();
    }
    ~InterruptDisabler() noexcept {
        SREG = storage_;
    }
private:
    uint8_t storage_ = 0;
};


template<i960Pinout pin, decltype(HIGH) value>
inline void digitalWrite() noexcept {
    if constexpr (auto& thePort = getAssociatedOutputPort<pin>(); value == LOW) {
        thePort &= ~getPinMask<pin>();
    } else {
        thePort |= getPinMask<pin>();
    }
}
template<i960Pinout pin>
inline void digitalWrite(decltype(HIGH) value) noexcept {
    // don't disable interrupts, that should be done outside this method
    if (auto& thePort = getAssociatedOutputPort<pin>(); value == LOW) {
        thePort &= ~getPinMask<pin>();
    } else {
        thePort |= getPinMask<pin>();
    }
}

template<i960Pinout pin, decltype(HIGH) switchTo = LOW>
inline void pulse() noexcept {
    // use the switch to value to compute what to revert to
    digitalWrite<pin, switchTo>();
    digitalWrite<pin, ((switchTo == LOW) ? HIGH : LOW)>();
}

template<i960Pinout pin>
inline auto digitalRead() noexcept {
    return (getAssociatedInputPort<pin>() & getPinMask<pin>()) ? HIGH : LOW;
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
    static constexpr auto valid() noexcept { return isValidPin960_v<pin>; }
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
        static constexpr auto getDeassertionState() noexcept { return deasserted; }      \
        inline static void assertPin() noexcept { digitalWrite<pin,getAssertionState()>(); } \
        inline static void deassertPin() noexcept { digitalWrite<pin,getDeassertionState()>(); } \
        inline static void write(decltype(LOW) value) noexcept { digitalWrite<pin>(value); } \
        static constexpr auto valid() noexcept { return isValidPin960_v<pin>; }          \
        template<decltype(LOW) switchTo = LOW>  \
        inline static void pulse() noexcept {   \
            ::pulse<pin, switchTo>();           \
        }                                       \
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
        static constexpr auto valid() noexcept { return isValidPin960_v<pin>; } \
    }
#define DefSPICSPin(pin) DefOutputPin(pin, LOW, HIGH)

#ifdef CHIPSET_TYPE1
DefSPICSPin(i960Pinout::GPIOSelect);
DefInputPin(i960Pinout::W_R_, LOW, HIGH);
DefOutputPin(i960Pinout::Reset960, LOW, HIGH);
DefSPICSPin(i960Pinout::PSRAM_EN);
#elif defined(CHIPSET_TYPE3)
DefSPICSPin(i960Pinout::GPIO_CS0);
DefSPICSPin(i960Pinout::GPIO_CS1);
DefSPICSPin(i960Pinout::PSRAM_EN0);
DefSPICSPin(i960Pinout::PSRAM_EN1);
DefSPICSPin(i960Pinout::GPU_EN);
DefSPICSPin(i960Pinout::DAZZLER_EN);
#endif

DefSPICSPin(i960Pinout::SD_EN);
DefOutputPin(i960Pinout::Ready, LOW, HIGH);
DefInputPin(i960Pinout::FAIL, HIGH, LOW);
DefInputPin(i960Pinout::DEN_, LOW, HIGH);
DefInputPin(i960Pinout::BLAST_, LOW, HIGH);
#undef DefSPICSPin
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
