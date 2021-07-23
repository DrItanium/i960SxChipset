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
enum class i960Pinout;
template<i960Pinout pin> inline void pulse() noexcept;
template<i960Pinout pin> inline void toggle() noexcept;
template<i960Pinout pin, decltype(HIGH) value> inline void digitalWrite() noexcept;
template<i960Pinout pin> inline void digitalWrite(decltype(HIGH) value) noexcept;
template<i960Pinout pin> inline auto digitalRead() noexcept;
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

#include "Pinout_1284p.h"
#include "Pinout_GrandCentralM4.h"
#include "Pinout_Mega2560.h"
#if 0
static_assert(!isValidPin<i960Pinout::Count>, "The Count \"pin\" should be an invalid pin!");
#endif



inline void digitalWrite(i960Pinout ip, decltype(HIGH) value) {
    digitalWrite(static_cast<int>(ip), value);
}
inline void pinMode(i960Pinout ip, decltype(INPUT) value) {
    pinMode(static_cast<int>(ip), value);
}
inline auto digitalRead(i960Pinout ip) {
    return digitalRead(static_cast<int>(ip));
}


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
DefOutputPin(i960Pinout::Ready, LOW, HIGH);
DefOutputPin(i960Pinout::DISPLAY_EN, LOW, HIGH);
DefOutputPin(i960Pinout::SD_EN, LOW, HIGH);
DefInputPin(i960Pinout::SYSTEM_FAIL_, LOW, HIGH);
DefInputPin(i960Pinout::NEW_REQUEST_, LOW, HIGH);
DefInputPin(i960Pinout::W_R_, LOW, HIGH);
DefInputPin(i960Pinout::BE0, LOW, HIGH);
DefInputPin(i960Pinout::BE1, LOW, HIGH);
DefInputPin(i960Pinout::BA1, LOW, HIGH);
DefInputPin(i960Pinout::BA2, LOW, HIGH);
DefInputPin(i960Pinout::BA3, LOW, HIGH);
DefInputPin(i960Pinout::BOOT_NORMAL_, LOW, HIGH);
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
