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
#include <libbonuspin.h>
using Address = uint32_t;
enum class i960Pinout : decltype(A0) {
    // PORT B
    Led = 0, 	  // output
    CLOCK_OUT, // output, unusable
    AS_,     // input, AVR Int2
    SPI_BUS_EN, // output
    GPIOSelect,		// output
    MOSI,		  // reserved
    MISO,		  // reserved
    SCK, 		  // reserved
// PORT D
    RX0, 		  // reserved
    TX0, 		  // reserved
    DEN_,	  // AVR Interrupt INT0
    AVR_INT1, 		// AVR Interrupt INT1
    PWM0,	       //
    PWM1, 		  // unused
    PWM2, 		  // unused
    PWM3, 		  // unused
// PORT C
    SCL,		  // reserved
    SDA, 		  // reserved
    Ready, 	  // output
    Int0_,		  // output
    W_R_, 		  // input
    Reset960,		  // output
    BLAST_, 	 // input
    FAIL, 	     // input
// PORT A
    DC, 	 // output
    DISPLAY_EN, // output
    SD_EN,      // output
    BLE_EN,
    BLE_IRQ,
    WIFI_EN,
    WIFI_BUSY,
    WIFI_RST,
    Count,		  // special
};
static_assert(static_cast<decltype(HIGH)>(i960Pinout::Count) <= 32);

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
		inline static void assert() noexcept { digitalWrite(pin, getAssertionState()); } \
		inline static void deassert() noexcept { digitalWrite(pin, getDeassertionState()); } \
		inline static void write(decltype(LOW) value) noexcept { digitalWrite(pin, value); } \
		inline static void pulse() noexcept { \
			assert(); \
			deassert(); \
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
void setupPins(decltype(OUTPUT) direction, Pins ... pins) {
    (pinMode(pins, direction), ...);
}

template<typename ... Pins>
void digitalWriteBlock(decltype(HIGH) value, Pins ... pins) {
    (digitalWrite(pins, value), ...);
}

template<i960Pinout pinId, decltype(HIGH) onConstruction, decltype(LOW) onDestruction>
class PinToggler {
public:
    PinToggler() { digitalWrite(pinId, onConstruction); }
    ~PinToggler() { digitalWrite(pinId, onDestruction); }
};

template<i960Pinout pinId>
using HoldPinLow = PinToggler<pinId, LOW, HIGH>;

template<i960Pinout pinId>
using HoldPinHigh = PinToggler<pinId, HIGH, LOW>;

template<i960Pinout pinId>
class PinAsserter {
public:
    static_assert(DigitalPin<pinId>::isOutputPin());
    PinAsserter() { DigitalPin<pinId>::assert(); }
    ~PinAsserter() { DigitalPin<pinId>::deassert(); }
};

#endif //ARDUINO_PINOUT_H
