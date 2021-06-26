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
/// @todo fix this pinout for different targets
enum class i960Pinout : decltype(A0) {
    // this is described in digial pin order!
    // leave this one alone
// PORT B
    Led = 0,      // output
    CLOCK_OUT, // output, unusable
    AS_,     // input, AVR Int2
    PWM4, // unused
    GPIOSelect,        // output
    MOSI,          // reserved
    MISO,          // reserved
    SCK,          // reserved
// PORT D
    RX0,          // reserved
    TX0,          // reserved
    DEN_,      // AVR Interrupt INT0
    AVR_INT1,        // AVR Interrupt INT1
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
// PORT A, used as part of servicing address requests for speed purposes
    WriteReadCheck,
    BurstAddress1,
    BurstAddress2,
    BurstAddress3,
    ByteEnable0,
    ByteEnable1,
    BurstLast,
    FAIL_INPUT,
    Count,
};
static_assert(static_cast<int>(i960Pinout::Count) == 32);
/**
 * @brief If both the display en and dc pins are bound to sda or scl then communication is taking place over i2c
 */
[[nodiscard]] constexpr auto talksToDisplayOverI2C() noexcept {
    return (i960Pinout::DISPLAY_EN == i960Pinout::SDA || i960Pinout::DISPLAY_EN == i960Pinout::SCL) &&
            (i960Pinout::DC == i960Pinout::SDA || i960Pinout::DC == i960Pinout::SCL );
}

/**
 * @brief If both the display and dc pins are bound to non i2c pins then assume it is talking over spi
 */
[[nodiscard]] constexpr auto talksToDisplayOverSPI() noexcept {
    return (i960Pinout::DISPLAY_EN != i960Pinout::SDA) &&
           (i960Pinout::DISPLAY_EN != i960Pinout::SCL) &&
           (i960Pinout::DC != i960Pinout::SDA) &&
           (i960Pinout::DC != i960Pinout::SCL);
}

inline void digitalWrite(i960Pinout ip, decltype(HIGH) value) {
    digitalWrite(static_cast<int>(ip), value);
}

inline void pinMode(i960Pinout ip, decltype(INPUT) value) {
    pinMode(static_cast<int>(ip), value);
}

inline auto digitalRead(i960Pinout ip) {
    return digitalRead(static_cast<int>(ip));
}
constexpr auto isValidPin(i960Pinout p) noexcept {
    switch (p) {
        case i960Pinout::Led:      // output
        case i960Pinout::CLOCK_OUT: // output: unusable
        case i960Pinout::AS_:     // input: AVR Int2
        case i960Pinout::PWM4: // unused
        case i960Pinout::GPIOSelect:        // output
        case i960Pinout::MOSI:          // reserved
        case i960Pinout::MISO:          // reserved
        case i960Pinout::SCK:          // reserved
        case i960Pinout::RX0:          // reserved
        case i960Pinout::TX0:          // reserved
        case i960Pinout::DEN_:      // AVR Interrupt INT0
        case i960Pinout::AVR_INT1:        // AVR Interrupt INT1
        case i960Pinout::SPI_BUS_EN: // output
        case i960Pinout::DC:     // output
        case i960Pinout::DISPLAY_EN: // output
        case i960Pinout::SD_EN:      // output
        case i960Pinout::SCL:          // reserved
        case i960Pinout::SDA:          // reserved
        case i960Pinout::Ready:      // output
        case i960Pinout::Int0_:          // output
        case i960Pinout::W_R_:          // input
        case i960Pinout::Reset960:          // output
        case i960Pinout::BLAST_:     // input
        case i960Pinout::FAIL:         // input
        case i960Pinout::WriteReadCheck:
        case i960Pinout::BurstAddress1:
        case i960Pinout::BurstAddress2:
        case i960Pinout::BurstAddress3:
        case i960Pinout::ByteEnable0:
        case i960Pinout::ByteEnable1:
        case i960Pinout::BurstLast:
        case i960Pinout::FAIL_INPUT:
            return true;
        default:
            return false;
    }
}
template<i960Pinout targetPin>
volatile uint8_t& getPort() noexcept {
    static_assert(isValidPin(targetPin), "Invalid Pin found, cannot bind to a port!");
    switch (targetPin) {
        case i960Pinout::Led:      // output
        case i960Pinout::CLOCK_OUT: // output: unusable
        case i960Pinout::AS_:     // input: AVR Int2
        case i960Pinout::PWM4: // unused
        case i960Pinout::GPIOSelect:        // output
        case i960Pinout::MOSI:          // reserved
        case i960Pinout::MISO:          // reserved
        case i960Pinout::SCK:          // reserved
            return PORTB;
// PORT D
        case i960Pinout::RX0:          // reserved
        case i960Pinout::TX0:          // reserved
        case i960Pinout::DEN_:      // AVR Interrupt INT0
        case i960Pinout::AVR_INT1:        // AVR Interrupt INT1
        case i960Pinout::SPI_BUS_EN: // output
        case i960Pinout::DC:     // output
        case i960Pinout::DISPLAY_EN: // output
        case i960Pinout::SD_EN:      // output
        return PORTD;
// PORT C
        case i960Pinout::SCL:          // reserved
        case i960Pinout::SDA:          // reserved
        case i960Pinout::Ready:      // output
        case i960Pinout::Int0_:          // output
        case i960Pinout::W_R_:          // input
        case i960Pinout::Reset960:          // output
        case i960Pinout::BLAST_:     // input
        case i960Pinout::FAIL:         // input
        return PORTC;
// PORT A: used to select the spi bus address (not directly used)
        case i960Pinout::WriteReadCheck:
        case i960Pinout::BurstAddress1:
        case i960Pinout::BurstAddress2:
        case i960Pinout::BurstAddress3:
        case i960Pinout::ByteEnable0:
        case i960Pinout::ByteEnable1:
        case i960Pinout::BurstLast:
        case i960Pinout::FAIL_INPUT:
            return PORTD;
        default:
            // force a crash by dereferencing 0
            return *(reinterpret_cast<volatile uint8_t*>(0));
    }
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
        inline static void assertPin() noexcept { digitalWrite(pin, getAssertionState()); } \
        inline static void deassertPin() noexcept { digitalWrite(pin, getDeassertionState()); } \
        inline static void write(decltype(LOW) value) noexcept { digitalWrite(pin, value); } \
        inline static void pulse() noexcept { \
            assertPin(); \
            deassertPin(); \
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
        inline static auto read() noexcept { return digitalRead(pin); } \
        inline static bool isAsserted() noexcept { return read() == getAssertionState(); } \
        inline static bool isDeasserted() noexcept { return read() == getDeassertionState(); } \
    }

#define DefInputPullupPin(pin, asserted, deasserted) \
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
        static constexpr auto getDirection() noexcept { return INPUT_PULLUP; } \
        static constexpr auto getAssertionState() noexcept { return asserted; } \
        static constexpr auto getDeassertionState() noexcept { return deasserted; } \
        inline static bool isAsserted() noexcept { return digitalRead(pin) == getAssertionState(); } \
        inline static bool isDeasserted() noexcept { return digitalRead(pin) == getDeassertionState(); } \
        inline static auto read() noexcept { return digitalRead(pin); } \
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
#undef DefInputPullupPin

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
