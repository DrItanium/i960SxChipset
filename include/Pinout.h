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
#include "MCP23S17.h"

using Address = uint32_t;
/**
 * @brief Sx Load/Store styles that the processor will request
 */
enum class i960Pinout : int {
#ifdef CHIPSET_TYPE1
#include "Type1Pinout.def"
#elif defined(CHIPSET_TYPE_MEGA)
#include "TypeMegaPinout.def"
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
 * @brief Represents a type safe look at the ports available on the 1284p
 */
enum class PortId
{
    None,
    PortA,
    PortB,
    PortC,
    PortD,
#ifdef CHIPSET_TYPE_MEGA
    PortE,
    PortF,
    PortG,
    PortH,
    PortJ,
    PortK,
    PortL,
#endif
    // more ports go here
};
template<i960Pinout pin>
[[gnu::always_inline]]
[[nodiscard]] constexpr PortId portFromPin() noexcept {
    switch (pin) {
        case i960Pinout::PORT_A0:
        case i960Pinout::PORT_A1:
        case i960Pinout::PORT_A2:
        case i960Pinout::PORT_A3:
        case i960Pinout::PORT_A4:
        case i960Pinout::PORT_A5:
        case i960Pinout::PORT_A6:
        case i960Pinout::PORT_A7:
            return PortId::PortA;
        case i960Pinout::PORT_B0:
        case i960Pinout::PORT_B1:
        case i960Pinout::PORT_B2:
        case i960Pinout::PORT_B3:
        case i960Pinout::PORT_B4:
        case i960Pinout::PORT_B5:
        case i960Pinout::PORT_B6:
        case i960Pinout::PORT_B7:
            return PortId::PortB;
        case i960Pinout::PORT_C0:
        case i960Pinout::PORT_C1:
        case i960Pinout::PORT_C2:
        case i960Pinout::PORT_C3:
        case i960Pinout::PORT_C4:
        case i960Pinout::PORT_C5:
        case i960Pinout::PORT_C6:
        case i960Pinout::PORT_C7:
            return PortId::PortC;
        case i960Pinout::PORT_D0:
        case i960Pinout::PORT_D1:
        case i960Pinout::PORT_D2:
        case i960Pinout::PORT_D3:
        case i960Pinout::PORT_D4:
        case i960Pinout::PORT_D5:
        case i960Pinout::PORT_D6:
        case i960Pinout::PORT_D7:
            return PortId::PortD;
#ifdef CHIPSET_TYPE_MEGA
#define X(letter, index) case i960Pinout:: PORT_ ## letter ## index
#define Y(letter) X(letter, 0): X(letter, 1): X(letter, 2): X(letter, 3): X(letter, 4): X(letter, 5): X(letter, 6): X(letter, 7)
            Y(E): return PortId::PortE;
            Y(F): return PortId::PortF;
            Y(H): return PortId::PortH;
            Y(J): return PortId::PortJ;
            Y(K): return PortId::PortK;
            Y(L): return PortId::PortL;
#undef Y
#undef X
        case i960Pinout::PORT_G0:
        case i960Pinout::PORT_G1:
        case i960Pinout::PORT_G2:
        case i960Pinout::PORT_G3:
        case i960Pinout::PORT_G4:
        case i960Pinout::PORT_G5:
            return PortId::PortG;
#endif

        default:
            return PortId::None;
    }
}
template<i960Pinout pin>
[[gnu::always_inline]]
[[nodiscard]] constexpr byte pinToPortBit() noexcept {
    static_assert(isValidPin960_v<pin>);
    switch (pin) {
        case i960Pinout::PORT_A0:
        case i960Pinout::PORT_B0:
        case i960Pinout::PORT_C0:
        case i960Pinout::PORT_D0:
#ifdef CHIPSET_TYPE_MEGA
        case i960Pinout::PORT_E0:
        case i960Pinout::PORT_F0:
        case i960Pinout::PORT_G0:
        case i960Pinout::PORT_H0:
        case i960Pinout::PORT_J0:
        case i960Pinout::PORT_K0:
        case i960Pinout::PORT_L0:
#endif
            return 0b0000'0001;
        case i960Pinout::PORT_A1:
        case i960Pinout::PORT_B1:
        case i960Pinout::PORT_C1:
        case i960Pinout::PORT_D1:
#ifdef CHIPSET_TYPE_MEGA
        case i960Pinout::PORT_E1:
        case i960Pinout::PORT_F1:
        case i960Pinout::PORT_G1:
        case i960Pinout::PORT_H1:
        case i960Pinout::PORT_J1:
        case i960Pinout::PORT_K1:
        case i960Pinout::PORT_L1:
#endif
            return 0b0000'0010;
        case i960Pinout::PORT_A2:
        case i960Pinout::PORT_B2:
        case i960Pinout::PORT_C2:
        case i960Pinout::PORT_D2:
#ifdef CHIPSET_TYPE_MEGA
        case i960Pinout::PORT_E2:
        case i960Pinout::PORT_F2:
        case i960Pinout::PORT_G2:
        case i960Pinout::PORT_H2:
        case i960Pinout::PORT_J2:
        case i960Pinout::PORT_K2:
        case i960Pinout::PORT_L2:
#endif
            return 0b0000'0100;
        case i960Pinout::PORT_A3:
        case i960Pinout::PORT_B3:
        case i960Pinout::PORT_C3:
        case i960Pinout::PORT_D3:
#ifdef CHIPSET_TYPE_MEGA
        case i960Pinout::PORT_E3:
        case i960Pinout::PORT_F3:
        case i960Pinout::PORT_G3:
        case i960Pinout::PORT_H3:
        case i960Pinout::PORT_J3:
        case i960Pinout::PORT_K3:
        case i960Pinout::PORT_L3:
#endif
            return 0b0000'1000;
        case i960Pinout::PORT_A4:
        case i960Pinout::PORT_B4:
        case i960Pinout::PORT_C4:
        case i960Pinout::PORT_D4:
#ifdef CHIPSET_TYPE_MEGA
        case i960Pinout::PORT_E4:
        case i960Pinout::PORT_F4:
        case i960Pinout::PORT_G4:
        case i960Pinout::PORT_H4:
        case i960Pinout::PORT_J4:
        case i960Pinout::PORT_K4:
        case i960Pinout::PORT_L4:
#endif
            return 0b0001'0000;
        case i960Pinout::PORT_A5:
        case i960Pinout::PORT_B5:
        case i960Pinout::PORT_C5:
        case i960Pinout::PORT_D5:
#ifdef CHIPSET_TYPE_MEGA
        case i960Pinout::PORT_E5:
        case i960Pinout::PORT_F5:
        case i960Pinout::PORT_G5:
        case i960Pinout::PORT_H5:
        case i960Pinout::PORT_J5:
        case i960Pinout::PORT_K5:
        case i960Pinout::PORT_L5:
#endif
            return 0b0010'0000;
        case i960Pinout::PORT_A6:
        case i960Pinout::PORT_B6:
        case i960Pinout::PORT_C6:
        case i960Pinout::PORT_D6:
#ifdef CHIPSET_TYPE_MEGA
        case i960Pinout::PORT_E6:
        case i960Pinout::PORT_F6:
        case i960Pinout::PORT_H6:
        case i960Pinout::PORT_J6:
        case i960Pinout::PORT_K6:
        case i960Pinout::PORT_L6:
#endif
            return 0b0100'0000;
        case i960Pinout::PORT_A7:
        case i960Pinout::PORT_B7:
        case i960Pinout::PORT_C7:
        case i960Pinout::PORT_D7:
#ifdef CHIPSET_TYPE_MEGA
        case i960Pinout::PORT_E7:
        case i960Pinout::PORT_F7:
        case i960Pinout::PORT_H7:
        case i960Pinout::PORT_J7:
        case i960Pinout::PORT_K7:
        case i960Pinout::PORT_L7:
#endif
            return 0b1000'0000;
        default:
            return 0b1111'1111;
    }
}
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
#ifdef CHIPSET_TYPE_MEGA
        Y(E): return PORTE;
        Y(F): return PORTF;
        //Y(G): return PORTG;
        Y(H): return PORTH;
        Y(J): return PORTJ;
        Y(K): return PORTK;
        Y(L): return PORTL;
        X(G, 0): X(G, 1): X(G, 2): X(G, 3): X(G, 4): X(G, 5): return PORTG;
#endif
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
#ifdef CHIPSET_TYPE_MEGA
        Y(E): return DDRE;
        Y(F): return DDRF;
        Y(H): return DDRH;
        Y(J): return DDRJ;
        Y(K): return DDRK;
        Y(L): return DDRL;
        X(G, 0): X(G, 1): X(G, 2): X(G, 3): X(G, 4): X(G, 5): return DDRG;
#endif
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
#ifdef CHIPSET_TYPE_MEGA
        Y(E): return PINE;
        Y(F): return PINF;
        X(G, 0): X(G, 1): X(G, 2): X(G, 3): X(G, 4): X(G, 5): return PING;
        Y(H): return PINH;
        Y(J): return PINJ;
        Y(K): return PINK;
        Y(L): return PINL;
#endif
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
#ifdef CHIPSET_TYPE_MEGA
        Y(E);
        Y(F);
        X(G, 0);
        X(G, 1);
        X(G, 2);
        X(G, 3);
        X(G, 4);
        X(G, 5);
        Y(H);
        Y(J);
        Y(K);
        Y(L);
#endif
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

template<i960Pinout pin, decltype(INPUT) defaultDirection>
struct BackingDigitalPin {
    BackingDigitalPin() = delete;
    ~BackingDigitalPin() = delete;
    BackingDigitalPin(const BackingDigitalPin&) = delete;
    BackingDigitalPin(BackingDigitalPin&&) = delete;
    BackingDigitalPin& operator=(const BackingDigitalPin&) = delete;
    BackingDigitalPin& operator=(BackingDigitalPin&&) = delete;
    static decltype(HIGH) read() noexcept { return digitalRead<pin>(); }
    static void write(decltype(HIGH) value) noexcept { digitalWrite<pin>(value); }
    template<decltype(HIGH) value>
    static void write() noexcept  { digitalWrite<pin, value>(); }
    static void configure(decltype(INPUT) mode = defaultDirection) {
        direction_ = mode;
        pinMode(pin, direction_);
    }
    static constexpr bool isInputPin() noexcept { return direction_ == INPUT || direction_ == INPUT_PULLUP; }
    static constexpr bool isOutputPin() noexcept { return direction_ == OUTPUT; }
    static constexpr decltype(INPUT) mode() noexcept { return direction_; }
    static constexpr auto getPin() noexcept { return pin; }
    static constexpr auto valid() noexcept { return isValidPin960_v<pin>; }
    template<decltype(HIGH) to = LOW>
    [[gnu::always_inline]] static void pulse() {
        ::pulse<pin, to>();
    }
private:
    static inline decltype(INPUT) direction_ = defaultDirection;
};
#if 0
template<i960Pinout pin, decltype(INPUT) defaultDirection, byte index>
struct BackingIOExpanderPin {
    BackingIOExpanderPin() = delete;
    ~BackingIOExpanderPin() = delete;
    BackingIOExpanderPin(const BackingIOExpanderPin&)  = delete;
    BackingIOExpanderPin& operator=(const BackingIOExpanderPin&)  = delete;
    BackingIOExpanderPin(BackingIOExpanderPin&&)  = delete;
    BackingIOExpanderPin& operator=(BackingIOExpanderPin&&)  = delete;
    static decltype(HIGH) read() noexcept { return digitalRead<pin>(); }
    static void write(decltype(HIGH) value) noexcept { digitalWrite<pin>(value); }
    template<decltype(HIGH) value>
    static void write() noexcept  { digitalWrite<pin, value>(); }
    static void configure(decltype(INPUT) mode = defaultDirection) {
        direction_ = mode;
        pinMode(pin, direction_);
    }
    static constexpr bool isInputPin() noexcept { return direction_ == INPUT || direction_ == INPUT_PULLUP; }
    static constexpr bool isOutputPin() noexcept { return direction_ == OUTPUT; }
    static constexpr decltype(INPUT) mode() noexcept { return direction_; }
    static constexpr auto getPin() noexcept { return pin; }
    static constexpr auto valid() noexcept { return isValidPin960_v<pin>; }
    template<decltype(HIGH) to = LOW>
    [[gnu::always_inline]] static void pulse() {
        ::pulse<pin, to>();
    }
};
#endif
template<typename T>
struct DigitalPin2 {
    using BackingThing = typename T::BackingImplementation;
    DigitalPin2() = delete;
    ~DigitalPin2() = delete;
    DigitalPin2(const DigitalPin2&) = delete;
    DigitalPin2(DigitalPin2&&) = delete;
    DigitalPin2& operator=(const DigitalPin2&) = delete;
    DigitalPin2& operator=(DigitalPin2&&) = delete;
    static void setup(decltype(OUTPUT) direction = T::Direction) noexcept { BackingThing::configure(direction); }
    static constexpr auto isInputPin() noexcept { return BackingThing::isInputPin(); }
    static constexpr auto isOutputPin() noexcept { return BackingThing::isOutputPin(); }
    static constexpr auto getPin() noexcept { return BackingThing ::getPin(); }
    static constexpr auto getDirection() noexcept { return BackingThing::mode(); }
    static constexpr auto getAssertionState() noexcept { return T::Assert; }
    static constexpr auto getDeassertionState() noexcept { return T::Deassert; }
    static constexpr auto getExpectedDirection() noexcept { return T::Direction; }
    static constexpr auto valid() noexcept { return BackingThing::valid(); }
    [[gnu::always_inline]] inline static void assertPin() noexcept { BackingThing::template write<getAssertionState()>(); }
    [[gnu::always_inline]] inline static void deassertPin() noexcept { BackingThing::template write<getDeassertionState()>(); }
    [[gnu::always_inline]] inline static void write(decltype(LOW) value) noexcept { BackingThing::write(value); }
    template<decltype(LOW) switchTo = LOW>
    [[gnu::always_inline]] inline static void pulse() noexcept {
        BackingThing::template pulse<switchTo>();
    }
    [[gnu::always_inline]] inline static auto read() noexcept { return BackingThing ::read(); }
    [[gnu::always_inline]] inline static bool isAsserted() noexcept { return read() == getAssertionState(); }
    [[gnu::always_inline]] inline static bool isDeasserted() noexcept { return read() == getDeassertionState(); }
};
template<i960Pinout pinout>
struct DigitalPinDescription {
    DigitalPinDescription() = delete;
    ~DigitalPinDescription() = delete;
    DigitalPinDescription(const DigitalPinDescription&) = delete;
    DigitalPinDescription(DigitalPinDescription&&) = delete;
    DigitalPinDescription& operator=(const DigitalPinDescription&) = delete;
    DigitalPinDescription& operator=(DigitalPinDescription&&) = delete;
    static constexpr auto Pin = pinout;
};
template<i960Pinout pin>
using DigitalPin = DigitalPin2<DigitalPinDescription<pin>>;
#define DefInputPin(pin, asserted, deasserted) \
template<> struct DigitalPinDescription <pin> {   \
    DigitalPinDescription() = delete;           \
    ~DigitalPinDescription() = delete;          \
    DigitalPinDescription(const DigitalPinDescription&) = delete; \
    DigitalPinDescription(DigitalPinDescription&&) = delete;      \
    DigitalPinDescription& operator=(const DigitalPinDescription&) = delete; \
    DigitalPinDescription& operator=(DigitalPinDescription&&) = delete;      \
    static constexpr auto Assert = asserted;    \
    static constexpr auto Deassert = deasserted;\
    static constexpr auto Direction = INPUT;   \
    static constexpr auto Pin = pin;           \
    using BackingImplementation = BackingDigitalPin<Pin, Direction>;                                            \
    }
#define DefOutputPin(pin, asserted, deasserted) \
    template<> struct DigitalPinDescription <pin> {   \
    DigitalPinDescription() = delete;           \
    ~DigitalPinDescription() = delete;          \
    DigitalPinDescription(const DigitalPinDescription&) = delete; \
    DigitalPinDescription(DigitalPinDescription&&) = delete;      \
    DigitalPinDescription& operator=(const DigitalPinDescription&) = delete; \
    DigitalPinDescription& operator=(DigitalPinDescription&&) = delete;      \
    static constexpr auto Assert = asserted;    \
    static constexpr auto Deassert = deasserted;\
    static constexpr auto Direction = OUTPUT;   \
    static constexpr auto Pin = pin; \
    using BackingImplementation = BackingDigitalPin<Pin, Direction>;                                            \
    }


#define DefSPICSPin(pin) DefOutputPin(pin, LOW, HIGH)



DefSPICSPin(i960Pinout::GPIOSelect);
DefSPICSPin(i960Pinout::SD_EN);
DefInputPin(i960Pinout::DEN_, LOW, HIGH);
DefOutputPin(i960Pinout::Ready, LOW, HIGH);
DefInputPin(i960Pinout::IOEXP_INT0, LOW, HIGH);
DefInputPin(i960Pinout::IOEXP_INT1, LOW, HIGH);
DefInputPin(i960Pinout::IOEXP_INT2, LOW, HIGH);
DefInputPin(i960Pinout::IOEXP_INT3, LOW, HIGH);

#ifdef CHIPSET_TYPE_MEGA
union CTL0Register {
    static constexpr size_t Address = 0xA104;
    byte reg;
    struct {
       byte byteEnable : 2;
       byte blast : 1;
       byte fail : 1;
       byte den : 1;
       byte burstAddress : 3;
    } bits;
};
union CTL1Register {
    static constexpr size_t Address = 0xA105;
    byte reg;
    struct {
        byte inRamSpace : 1;
        byte inIOSpace : 1;
        byte unused : 6;
    } bits;
};
static_assert (sizeof(CTL0Register) == sizeof(byte));
union AddressRegister {
    static constexpr size_t Address = 0xA100;
    explicit AddressRegister(uint32_t f) : full(f) { }
    explicit AddressRegister(bool wr, uint32_t addr) : wr_(wr), address(addr) { }
    AddressRegister(const AddressRegister& other) : full(other.full) { }
    uint32_t full;
    struct {
        uint32_t wr_ : 1;
        uint32_t address : 31;
    };
    struct {
        uint32_t offset : (32 - 3);
        uint32_t space : 3;
    } space;
};
static_assert(sizeof (AddressRegister) == sizeof(uint32_t));
template<typename T>
inline volatile T& getRegister() noexcept {
    return memory<T>(T::Address);
}
inline volatile CTL0Register& getCTL0() noexcept { return getRegister<CTL0Register>(); }
inline volatile AddressRegister& getAddressRegister() noexcept { return getRegister<AddressRegister>(); }
template<>
struct DigitalPin< i960Pinout::FAIL > {
        DigitalPin() = delete;
        ~DigitalPin() = delete;
        DigitalPin(const DigitalPin&) = delete;
        DigitalPin(DigitalPin&&) = delete;
        DigitalPin& operator=(const DigitalPin&) = delete;
        DigitalPin& operator=(DigitalPin&&) = delete;
        static constexpr auto isInputPin() noexcept { return true; }
        static constexpr auto isOutputPin() noexcept { return false; }
        static constexpr auto getPin() noexcept { return i960Pinout::FAIL; }
        static constexpr auto getDirection() noexcept { return INPUT; }
        static constexpr auto getAssertionState() noexcept { return HIGH; }
        static constexpr auto getDeassertionState() noexcept { return LOW; }
        [[gnu::always_inline]] inline static auto read() noexcept { return getCTL0().bits.fail; }
        [[gnu::always_inline]] inline static bool isAsserted() noexcept { return read() == getAssertionState(); }
        [[gnu::always_inline]] inline static bool isDeasserted() noexcept { return read() == getDeassertionState(); }
        static constexpr auto valid() noexcept { return isValidPin960_v<i960Pinout::FAIL>; }
};

template<>
struct DigitalPin< i960Pinout::BLAST_> {
    DigitalPin() = delete;
    ~DigitalPin() = delete;
    DigitalPin(const DigitalPin&) = delete;
    DigitalPin(DigitalPin&&) = delete;
    DigitalPin& operator=(const DigitalPin&) = delete;
    DigitalPin& operator=(DigitalPin&&) = delete;
    static constexpr auto isInputPin() noexcept { return true; }
    static constexpr auto isOutputPin() noexcept { return false; }
    static constexpr auto getPin() noexcept { return i960Pinout::BLAST_; }
    static constexpr auto getDirection() noexcept { return INPUT; }
    static constexpr auto getAssertionState() noexcept { return LOW; }
    static constexpr auto getDeassertionState() noexcept { return HIGH; }
    [[gnu::always_inline]] inline static auto read() noexcept { return getCTL0().bits.blast; }
    [[gnu::always_inline]] inline static bool isAsserted() noexcept { return read() == getAssertionState(); }
    [[gnu::always_inline]] inline static bool isDeasserted() noexcept { return read() == getDeassertionState(); }
    static constexpr auto valid() noexcept { return isValidPin960_v<i960Pinout::BLAST_>; }
};

template<>
struct DigitalPin< i960Pinout::BE0> {
    DigitalPin() = delete;
    ~DigitalPin() = delete;
    DigitalPin(const DigitalPin&) = delete;
    DigitalPin(DigitalPin&&) = delete;
    DigitalPin& operator=(const DigitalPin&) = delete;
    DigitalPin& operator=(DigitalPin&&) = delete;
    static constexpr auto isInputPin() noexcept { return true; }
    static constexpr auto isOutputPin() noexcept { return false; }
    static constexpr auto getPin() noexcept { return i960Pinout::BE0; }
    static constexpr auto getDirection() noexcept { return INPUT; }
    static constexpr auto getAssertionState() noexcept { return LOW; }
    static constexpr auto getDeassertionState() noexcept { return HIGH; }
    [[gnu::always_inline]] inline static auto read() noexcept { return getCTL0().bits.byteEnable & 0b01; }
    [[gnu::always_inline]] inline static bool isAsserted() noexcept { return read() == getAssertionState(); }
    [[gnu::always_inline]] inline static bool isDeasserted() noexcept { return read() == getDeassertionState(); }
    static constexpr auto valid() noexcept { return isValidPin960_v<i960Pinout::BE0>; }
};
template<>
struct DigitalPin< i960Pinout::BE1> {
    DigitalPin() = delete;
    ~DigitalPin() = delete;
    DigitalPin(const DigitalPin&) = delete;
    DigitalPin(DigitalPin&&) = delete;
    DigitalPin& operator=(const DigitalPin&) = delete;
    DigitalPin& operator=(DigitalPin&&) = delete;
    static constexpr auto isInputPin() noexcept { return true; }
    static constexpr auto isOutputPin() noexcept { return false; }
    static constexpr auto getPin() noexcept { return i960Pinout::BE1; }
    static constexpr auto getDirection() noexcept { return INPUT; }
    static constexpr auto getAssertionState() noexcept { return LOW; }
    static constexpr auto getDeassertionState() noexcept { return HIGH; }
    [[gnu::always_inline]] inline static auto read() noexcept { return getCTL0().bits.byteEnable & 0b10; }
    [[gnu::always_inline]] inline static bool isAsserted() noexcept { return read() == getAssertionState(); }
    [[gnu::always_inline]] inline static bool isDeasserted() noexcept { return read() == getDeassertionState(); }
    static constexpr auto valid() noexcept { return isValidPin960_v<i960Pinout::BE1>; }
};
template<>
struct DigitalPin< i960Pinout::RAM_SPACE_> {
    DigitalPin() = delete;
    ~DigitalPin() = delete;
    DigitalPin(const DigitalPin&) = delete;
    DigitalPin(DigitalPin&&) = delete;
    DigitalPin& operator=(const DigitalPin&) = delete;
    DigitalPin& operator=(DigitalPin&&) = delete;
    static constexpr auto isInputPin() noexcept { return true; }
    static constexpr auto isOutputPin() noexcept { return false; }
    static constexpr auto getPin() noexcept { return i960Pinout::RAM_SPACE_; }
    static constexpr auto getDirection() noexcept { return INPUT; }
    static constexpr auto getAssertionState() noexcept { return LOW; }
    static constexpr auto getDeassertionState() noexcept { return HIGH; }
    [[gnu::always_inline]] inline static auto read() noexcept { return getAddressRegister().space.space == 0 ? getAssertionState() : getDeassertionState(); }
    [[gnu::always_inline]] inline static bool isAsserted() noexcept { return read() == getAssertionState(); }
    [[gnu::always_inline]] inline static bool isDeasserted() noexcept { return read() == getDeassertionState(); }
    static constexpr auto valid() noexcept { return isValidPin960_v<i960Pinout::RAM_SPACE_>; }
};
template<>
struct DigitalPin< i960Pinout::IO_SPACE_> {
    DigitalPin() = delete;
    ~DigitalPin() = delete;
    DigitalPin(const DigitalPin&) = delete;
    DigitalPin(DigitalPin&&) = delete;
    DigitalPin& operator=(const DigitalPin&) = delete;
    DigitalPin& operator=(DigitalPin&&) = delete;
    static constexpr auto isInputPin() noexcept { return true; }
    static constexpr auto isOutputPin() noexcept { return false; }
    static constexpr auto getPin() noexcept { return i960Pinout::IO_SPACE_; }
    static constexpr auto getDirection() noexcept { return INPUT; }
    static constexpr auto getAssertionState() noexcept { return LOW; }
    static constexpr auto getDeassertionState() noexcept { return HIGH; }
    [[gnu::always_inline]] inline static auto read() noexcept { return getAddressRegister().space.space == 0b111 ? getAssertionState() : getDeassertionState(); }
    [[gnu::always_inline]] inline static bool isAsserted() noexcept { return read() == getAssertionState(); }
    [[gnu::always_inline]] inline static bool isDeasserted() noexcept { return read() == getDeassertionState(); }
    static constexpr auto valid() noexcept { return isValidPin960_v<i960Pinout::IO_SPACE_>; }
};
template<>
struct DigitalPin< i960Pinout::W_R_> {
    DigitalPin() = delete;
    ~DigitalPin() = delete;
    DigitalPin(const DigitalPin&) = delete;
    DigitalPin(DigitalPin&&) = delete;
    DigitalPin& operator=(const DigitalPin&) = delete;
    DigitalPin& operator=(DigitalPin&&) = delete;
    static constexpr auto isInputPin() noexcept { return true; }
    static constexpr auto isOutputPin() noexcept { return false; }
    static constexpr auto getPin() noexcept { return i960Pinout::BLAST_; }
    static constexpr auto getDirection() noexcept { return INPUT; }
    static constexpr auto getAssertionState() noexcept { return LOW; }
    static constexpr auto getDeassertionState() noexcept { return HIGH; }
    [[gnu::always_inline]] inline static auto read() noexcept { return getAddressRegister().wr_; }
    [[gnu::always_inline]] inline static bool isAsserted() noexcept { return read() == getAssertionState(); }
    [[gnu::always_inline]] inline static bool isDeasserted() noexcept { return read() == getDeassertionState(); }
    static constexpr auto valid() noexcept { return isValidPin960_v<i960Pinout::BLAST_>; }
};
#else
DefInputPin(i960Pinout::RAM_SPACE_, LOW, HIGH);
DefInputPin(i960Pinout::IO_SPACE_, LOW, HIGH);
DefInputPin(i960Pinout::FAIL, HIGH, LOW);
DefInputPin(i960Pinout::BLAST_, LOW, HIGH);
DefInputPin(i960Pinout::W_R_, LOW, HIGH);
DefInputPin(i960Pinout::BE0, LOW, HIGH);
DefInputPin(i960Pinout::BE1, LOW, HIGH);
DefSPICSPin(i960Pinout::PSRAM_EN);
DefSPICSPin(i960Pinout::MEMBLK0_);
DefSPICSPin(i960Pinout::TFT_CS);


DefOutputPin(i960Pinout::SPI_OFFSET0, LOW, HIGH);
DefOutputPin(i960Pinout::SPI_OFFSET1, LOW, HIGH);
DefOutputPin(i960Pinout::SPI_OFFSET2, LOW, HIGH);
DefOutputPin(i960Pinout::MEMBLK0_A0, LOW, HIGH);
DefOutputPin(i960Pinout::MEMBLK0_A1, LOW, HIGH);
DefOutputPin(i960Pinout::TFT_DC, LOW, HIGH);
#endif
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
enum class LoadStoreStyle : uint8_t {
    // based off of BE0,BE1 pins
    Full16 = 0,
#ifdef CHIPSET_TYPE1
    Upper8 = pinToPortBit<i960Pinout::BE0>(),
    Lower8 = pinToPortBit<i960Pinout::BE1>(),
    None = pinToPortBit<i960Pinout::BE0>() | pinToPortBit<i960Pinout::BE1>(),
#else
    Upper8 = 0b01,
    Lower8 = 0b10,
    None = 0b11,
#endif
};



#endif //ARDUINO_PINOUT_H
