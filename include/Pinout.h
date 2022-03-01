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
    Full16 = 0,
    Upper8,
    Lower8,
    None,
};
enum class i960Pinout : int {
#ifdef CHIPSET_TYPE1
#include "Type1Pinout.def"
#else
#error "Target Chipset Hardware has no pinout defined"
#endif
};

[[gnu::always_inline]]
inline void digitalWrite(i960Pinout ip, decltype(HIGH) value) {
    digitalWrite(static_cast<int>(ip), value);
}

[[gnu::always_inline]]
inline void pinMode(i960Pinout ip, decltype(INPUT) value) {
    pinMode(static_cast<int>(ip), value);
}
[[gnu::always_inline]]
inline auto digitalRead(i960Pinout ip) {
    return digitalRead(static_cast<int>(ip));
}
template<i960Pinout pin>
constexpr auto isValidPin960_v = static_cast<int>(pin) < static_cast<int>(i960Pinout::Count);

/**
 * @brief Get the output port associated with a given pin. A compile-time error will be generated if the pin is not valid
 * @tparam pin The pin whose output port will be retrieved
 * @return A reference to the output port address tied to the given pin
 */
template<i960Pinout pin>
[[gnu::always_inline]]
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

/**
 * @brief Get the direction register associated with a given pin. A compile-time error will be generated if the pin is not valid
 * @tparam pin The pin whose direction register will be retrieved
 * @return A reference to the direction register address tied to the given pin
 */
template<i960Pinout pin>
[[gnu::always_inline]]
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

/**
 * @brief Get the input port associated with a given pin. A compile-time error will be generated if the pin is not valid
 * @tparam pin The pin whose input port will be retrieved
 * @return A reference to the input port address tied to the given pin
 */
template<i960Pinout pin>
[[gnu::always_inline]]
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
/**
 * @brief Get the pin mask associated with a digital pin in a constexpr context. Will generate compile-time error if pin is not a valid one
 * @tparam pin The pin to get the proper pin mask for
 * @return The constant pin mask pattern for the target pin
 */
template<i960Pinout pin>
[[gnu::always_inline]]
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

/**
 * @brief RAII class which turns off interrupts during construction and restores them on destruction
 */
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

/**
 * @brief Condense digitalWrite calls down into a single instruction. Use this method if the value of a pin will be known at compile time.
 * @tparam pin the pin to modify
 * @tparam value the value to set the pin to
 */
template<i960Pinout pin, decltype(HIGH) value>
[[gnu::always_inline]]
inline void digitalWrite() noexcept {
    if constexpr (auto& thePort = getAssociatedOutputPort<pin>(); value == LOW) {
        // generates a warning in C++20 mode if you do &= due to changes in volatile
        thePort = thePort & ~getPinMask<pin>();
    } else {
        // generates a warning in C++20 mode if you do |= due to changes in volatile
        thePort = thePort | getPinMask<pin>();
    }
}
/**
 * @brief Condense digitalWrite calls down into a simpler form
 * @tparam pin The target pin to modify
 * @param value The value to set the given pin to
 */
template<i960Pinout pin>
[[gnu::always_inline]]
inline void digitalWrite(decltype(HIGH) value) noexcept {
    // don't disable interrupts, that should be done outside this method
    if (auto& thePort = getAssociatedOutputPort<pin>(); value == LOW) {
        thePort = thePort & ~getPinMask<pin>();
    } else {
        thePort = thePort | getPinMask<pin>();
    }
}
/**
 * @brief Convert a boolean value into a fast digitalWrite call
 * @tparam pin The pin to modify
 * @param level True -> HIGH , False -> LOW
 */
template<i960Pinout pin>
[[gnu::always_inline]] inline void digitalWrite(bool level) noexcept {
    digitalWrite<pin>(level ? HIGH : LOW);
}

/**
 * @brief Switch a given pin from one state to the other as fast as possible and then back again. It is the equivalent of:
 * digitalWrite(pin, LOW);
 * digitalWrite(pin, HIGH);
 *
 * The function will determine (at compile-time) the value to switch to and back. This function does not perform any error checking so if you do:
 *
 * digitalWrite<i960Pinout::PORT_D0, LOW>();
 * pulse<i960Pinout::PORT_D0, LOW>();
 *
 * Then you will see that the cpu holds the pin low for two iterations instead of one before pulling it high again.
 * @tparam pin The pin to pulse
 * @tparam switchTo The value to set the pin to, this is used to determine what the original state was.
 */
template<i960Pinout pin, decltype(HIGH) switchTo = LOW>
[[gnu::always_inline]]
inline void pulse() noexcept {
    // use the switch to value to compute what to revert to
    digitalWrite<pin, switchTo>();
    digitalWrite<pin, ((switchTo == LOW) ? HIGH : LOW)>();
}

/**
 * @brief Condense digitalRead into a single instruction (or as close as possible)
 * @tparam pin The pin to query
 * @return The pin value as HIGH or LOW depending on what it is set to
 */
template<i960Pinout pin>
[[gnu::always_inline]]
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
        [[gnu::always_inline]] inline static void assertPin() noexcept { digitalWrite<pin,getAssertionState()>(); } \
        [[gnu::always_inline]] inline static void deassertPin() noexcept { digitalWrite<pin,getDeassertionState()>(); } \
        [[gnu::always_inline]] inline static void write(decltype(LOW) value) noexcept { digitalWrite<pin>(value); } \
        static constexpr auto valid() noexcept { return isValidPin960_v<pin>; }          \
        template<decltype(LOW) switchTo = LOW>  \
        [[gnu::always_inline]]                  \
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
        [[gnu::always_inline]] inline static auto read() noexcept { return digitalRead<pin>(); } \
        [[gnu::always_inline]] inline static bool isAsserted() noexcept { return read() == getAssertionState(); } \
        [[gnu::always_inline]] inline static bool isDeasserted() noexcept { return read() == getDeassertionState(); } \
        static constexpr auto valid() noexcept { return isValidPin960_v<pin>; } \
    }
#define DefSPICSPin(pin) DefOutputPin(pin, LOW, HIGH)

DefSPICSPin(i960Pinout::GPIOSelect);
DefSPICSPin(i960Pinout::SD_EN);
DefSPICSPin(i960Pinout::PSRAM_EN);
DefSPICSPin(i960Pinout::MEMBLK0_);
DefSPICSPin(i960Pinout::TFT_CS);


DefOutputPin(i960Pinout::Ready, LOW, HIGH);
DefOutputPin(i960Pinout::SPI_OFFSET0, LOW, HIGH);
DefOutputPin(i960Pinout::SPI_OFFSET1, LOW, HIGH);
DefOutputPin(i960Pinout::SPI_OFFSET2, LOW, HIGH);
DefOutputPin(i960Pinout::MEMBLK0_A0, LOW, HIGH);
DefOutputPin(i960Pinout::MEMBLK0_A1, LOW, HIGH);
DefOutputPin(i960Pinout::TFT_DC, LOW, HIGH);

DefInputPin(i960Pinout::FAIL, HIGH, LOW);
DefInputPin(i960Pinout::DEN_, LOW, HIGH);
DefInputPin(i960Pinout::BLAST_, LOW, HIGH);
DefInputPin(i960Pinout::W_R_, LOW, HIGH);
DefInputPin(i960Pinout::RAM_SPACE_, LOW, HIGH);
DefInputPin(i960Pinout::IO_SPACE_, LOW, HIGH);
DefInputPin(i960Pinout::BE0, LOW, HIGH);
DefInputPin(i960Pinout::BE1, LOW, HIGH);
DefInputPin(i960Pinout::IOEXP_INT0, LOW, HIGH);
DefInputPin(i960Pinout::IOEXP_INT1, LOW, HIGH);
DefInputPin(i960Pinout::IOEXP_INT2, LOW, HIGH);
DefInputPin(i960Pinout::IOEXP_INT3, LOW, HIGH);
#undef DefSPICSPin
#undef DefInputPin
#undef DefOutputPin

/**
 * @brief Template parameter pack version of pinMode which sets a block of pins to a given direction.
 * This is designed to cut down on typing as much as possible
 * @tparam Pins The type of the pins to set their pinMode to (this is implicitly populated)
 * @param direction The direction that all specified pins will be set to
 * @param pins The pins to set
 */
template<typename ... Pins>
[[gnu::always_inline]]
inline void setupPins(decltype(OUTPUT) direction, Pins ... pins) {
    (pinMode(pins, direction), ...);
}

/**
 * @brief RAII-class which asserts a pin (direction defined by the DigitalPin class for a given pin) on construction.
 * It deasserts the pin on destruction. Very useful with scopes to make sure that you never forget to assert and deassert a pin.
 * Cleans up SPI transactions significantly. The compiler generally inlines the bodies of this class when used.
 * @tparam pinId The pin to affect for the lifetime of the object
 */
template<i960Pinout pinId>
class PinAsserter final {
public:
    static_assert(DigitalPin<pinId>::isOutputPin());
    PinAsserter() { DigitalPin<pinId>::assertPin(); }
    ~PinAsserter() { DigitalPin<pinId>::deassertPin(); }
};

#endif //ARDUINO_PINOUT_H
