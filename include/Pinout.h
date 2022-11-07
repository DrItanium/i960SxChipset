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
#define Entry(name) name,
#define PIN(name, port, offset)  Entry(name)
#define PORT(name, size)
#include "TargetMicrocontroller.def"
#undef PORT
#undef PIN
#undef Entry
#define X(name, pin, direction, assert, deassert, addr, haddr, offset) name = pin,
#define GPIO(name, pin, direction, assert, deassert) X(name, pin, direction, assert, deassert, 0, 0, 0)
#define IOEXP(name, pin, direction, assert, deassert, index, offset) X(name, pin, direction, assert, deassert, 0, index, offset)
#define SINK(name, pin, direction, assert, deassert) X(name, pin, direction, assert, deassert, 0, 0, 0)
#define ALIAS(name, pin) X(name, pin, INPUT, LOW, HIGH, 0, 0, 0)
#include "MappedPinouts.def"
#undef GPIO
#undef IOEXP
#undef SINK
#undef ALIAS
#undef X
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
#define Entry(name)
#define PIN(name, port, offset)
#define PORT(name, size) Port ## name ,
#include "TargetMicrocontroller.def"
#undef PIN
#undef PORT
#undef Entry
};
template<i960Pinout pin>
[[gnu::always_inline]]
[[nodiscard]] constexpr PortId portFromPin() noexcept {
    switch (pin) {
#define Entry(name)
#define PIN(name, port, offset) case i960Pinout:: name : return PortId :: Port ## port ;
#define PORT(name, size)
#include "TargetMicrocontroller.def"
#undef PIN
#undef PORT
#undef Entry
        default:
            return PortId::None;
    }
}
template<i960Pinout pin>
[[gnu::always_inline]]
[[nodiscard]] constexpr byte pinToPortBit() noexcept {
    static_assert(isValidPin960_v<pin>);
    switch (pin) {
#define Entry(name)
#define PIN(name, port, offset) case i960Pinout:: name : return static_cast<uint8_t>(1 << offset) ;
#define PORT(name, size)
#include "TargetMicrocontroller.def"
#undef PIN
#undef PORT
#undef Entry
        default:
            return 0b1111'1111;
    }
}
static_assert(pinToPortBit<i960Pinout::PORT_A0>() == 0b0000'0001);
static_assert(pinToPortBit<i960Pinout::PORT_A1>() == 0b0000'0010);
static_assert(pinToPortBit<i960Pinout::PORT_A2>() == 0b0000'0100);
static_assert(pinToPortBit<i960Pinout::PORT_A3>() == 0b0000'1000);
static_assert(pinToPortBit<i960Pinout::PORT_A4>() == 0b0001'0000);
static_assert(pinToPortBit<i960Pinout::PORT_A5>() == 0b0010'0000);
static_assert(pinToPortBit<i960Pinout::PORT_A6>() == 0b0100'0000);
static_assert(pinToPortBit<i960Pinout::PORT_A7>() == 0b1000'0000);
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
#define Entry(name)
#define PIN(name, port, offset) case i960Pinout:: name : return PORT ## port ;
#define PORT(name, size)
#include "TargetMicrocontroller.def"
#undef PIN
#undef PORT
#undef Entry
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
#define Entry(name)
#define PIN(name, port, offset) case i960Pinout:: name : return DDR ## port ;
#define PORT(name, size)
#include "TargetMicrocontroller.def"
#undef PIN
#undef PORT
#undef Entry
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
#define Entry(name)
#define PIN(name, port, offset) case i960Pinout:: name : return PIN ## port ;
#define PORT(name, size)
#include "TargetMicrocontroller.def"
#undef PIN
#undef PORT
#undef Entry
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
#define Entry(name)
#define PIN(name, port, offset) case i960Pinout:: name : return _BV ( P ## port ## offset ) ;
#define PORT(name, size)
#include "TargetMicrocontroller.def"
#undef PIN
#undef PORT
#undef Entry
        default:
            return 0xFF;
    }
}
static_assert(getPinMask<i960Pinout::PORT_A0>() == 0b0000'0001);
static_assert(getPinMask<i960Pinout::PORT_A1>() == 0b0000'0010);
static_assert(getPinMask<i960Pinout::PORT_A2>() == 0b0000'0100);
static_assert(getPinMask<i960Pinout::PORT_A3>() == 0b0000'1000);
static_assert(getPinMask<i960Pinout::PORT_A4>() == 0b0001'0000);
static_assert(getPinMask<i960Pinout::PORT_A5>() == 0b0010'0000);
static_assert(getPinMask<i960Pinout::PORT_A6>() == 0b0100'0000);
static_assert(getPinMask<i960Pinout::PORT_A7>() == 0b1000'0000);

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
#define ALIAS(name, pin_index)
#define SINK(name, pin_index, direction, assert, deassert)

#define GPIO(name, pin_index, direction, assert, deassert) \
template<> struct DigitalPinDescription < i960Pinout:: name > {          \
    DigitalPinDescription() = delete;           \
    ~DigitalPinDescription() = delete;          \
    DigitalPinDescription(const DigitalPinDescription&) = delete; \
    DigitalPinDescription(DigitalPinDescription&&) = delete;      \
    DigitalPinDescription& operator=(const DigitalPinDescription&) = delete; \
    DigitalPinDescription& operator=(DigitalPinDescription&&) = delete;      \
    static constexpr auto Assert = assert;                 \
    static constexpr auto Deassert = deassert;             \
    static constexpr auto Direction = direction;           \
    static constexpr auto Pin = i960Pinout:: name ;         \
    using BackingImplementation = BackingDigitalPin<Pin, Direction>; \
};

#define IOEXP(name, pin_index, direction, assert, deassert, index, offset) \
template<> \
struct DigitalPinDescription < i960Pinout:: name > {\
    DigitalPinDescription() = delete; \
    ~DigitalPinDescription() = delete; \
    DigitalPinDescription(const DigitalPinDescription&) = delete;\
    DigitalPinDescription(DigitalPinDescription&&) = delete;\
    DigitalPinDescription& operator=(const DigitalPinDescription&) = delete; \
    DigitalPinDescription& operator=(DigitalPinDescription&&) = delete; \
    static constexpr auto Assert = assert; \
    static constexpr auto Deassert = deassert;\
    static constexpr auto Direction = direction;\
    static constexpr auto Pin =  i960Pinout:: name ;                                      \
    static constexpr auto DeviceID  = MCP23S17::HardwareDeviceAddress::Device ## index; \
    static constexpr auto Offset = MCP23S17::PinIndex::Pin ## offset; \
    using CSPin = DigitalPin<i960Pinout::GPIOSelect>;                     \
    using BackingImplementation = MCP23S17::BackingPin<Pin, DeviceID, Offset, Direction, CSPin>;\
};

#include "MappedPinouts.def"
#undef GPIO
#undef SINK
#undef ALIAS
#undef IOEXP


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

enum class LoadStoreStyle : uint8_t {
    // based off of BE0,BE1 pins
    Full16 = 0,
    Upper8 = pinToPortBit<i960Pinout::BE0>(),
    Lower8 = pinToPortBit<i960Pinout::BE1>(),
    None = pinToPortBit<i960Pinout::BE0>() | pinToPortBit<i960Pinout::BE1>(),
};

#endif //ARDUINO_PINOUT_H
