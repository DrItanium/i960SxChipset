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
    // PORT B
    Ready = TargetBoard::getReadyPin(),
    CLOCK_OUT = TargetBoard::getClockOutPin(),
    PSRAM_EN = TargetBoard::getPsramEnPin(),
    GPIOSelect = TargetBoard::getGpioSelectPin(),
    MOSI = TargetBoard::getMosiPin(),          // reserved
    MISO = TargetBoard::getMisoPin(),          // reserved
    SCK = TargetBoard::getSckPin(),          // reserved
    DEN_ = TargetBoard::getDenPin(),      // AVR Interrupt INT0
    CACHE_EN_ = TargetBoard::getCacheEnPin(),
    // PD4 is not used by simple management card
    Reset960= TargetBoard::getReset960Pin(),
    Int0_ = TargetBoard::getInt0Pin(),
    // PD7 is not used by simple management card
    SCL = TargetBoard::getSclPin(),
    SDA = TargetBoard::getSdaPin(),
    SPI_OFFSET0 = TargetBoard::getSpiOffset0Pin(),
    SPI_OFFSET1 = TargetBoard::getSpiOffset1Pin(),
    SPI_OFFSET2 = TargetBoard::getSpiOffset2Pin(),
    DISPLAY_EN = TargetBoard::getDisplayEnPin(),
    DC = TargetBoard::getDcPin(),     // output
    SD_EN = TargetBoard::getSdEnablePin(),      // output
    W_R_ = TargetBoard::getWrPin(),
    BA1 = TargetBoard::getBurstAddress1Pin(),
    BA2 = TargetBoard::getBurstAddress2Pin(),
    BA3 = TargetBoard::getBurstAddress3Pin(),
    BE0 = TargetBoard::getByteEnable0Pin(),
    BE1 = TargetBoard::getByteEnable1Pin(),
    BLAST_ = TargetBoard::getBlastPin(),     // input
    FAIL = TargetBoard::getFailPin(),         // input
    AS_ = TargetBoard::getAddressStatePin(),
};
constexpr bool isValidPin(i960Pinout pin) noexcept {
    return isValidPin<UnderlyingPinoutType>(static_cast<UnderlyingPinoutType>(pin));
}
constexpr bool CacheActive_v = isValidPin(i960Pinout::CACHE_EN_);
constexpr bool DisplayActive_v = isValidPin(i960Pinout::DISPLAY_EN);
constexpr auto attachedToIOExpander(i960Pinout pinout) noexcept {
    return attachedToIOExpander<UnderlyingPinoutType>(static_cast<UnderlyingPinoutType>(pinout));
}
inline void digitalWrite(i960Pinout ip, decltype(HIGH) value) {
    if (isValidPin(ip)) {
        digitalWrite(static_cast<int>(ip), value);
    }
}

inline void pinMode(i960Pinout ip, decltype(INPUT) value) {
    if (isValidPin(ip)) {
        pinMode(static_cast<int>(ip), value);
    }
}
inline auto digitalRead(i960Pinout ip) {
    if (isValidPin(ip)) {
        return digitalRead(static_cast<int>(ip));
    } else {
        return LOW;
    }
}
template<i960Pinout pin>
constexpr auto isValidPin960_v = isValidPin_v<static_cast<UnderlyingPinoutType >(pin)>;
#ifdef ARDUINO_AVR_ATmega1284
//static_assert(isValidPin<i960Pinout::CACHE_A0>, "The CACHE_A0 pin should be a valid pin!");
template<i960Pinout pin>
[[nodiscard]] inline volatile unsigned char& getAssociatedOutputPort() noexcept {
    static_assert(isValidPin960_v<pin>, "INVALID PIN PROVIDED");
    switch (static_cast<UnderlyingPinoutType >(pin)) {
#define X(id, number) case UnderlyingPinoutType:: PORT_ ## id ## number
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
[[nodiscard]] inline volatile unsigned char& getAssociatedInputPort() noexcept {
    static_assert(isValidPin960_v<pin>, "INVALID PIN PROVIDED");
    switch (static_cast<UnderlyingPinoutType >(pin)) {
#define X(id, number) case UnderlyingPinoutType:: PORT_ ## id ## number
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
    switch (static_cast<UnderlyingPinoutType >(pin)) {
#define X(id, number) case UnderlyingPinoutType:: PORT_ ## id ## number : return _BV ( P ## id ## number )
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

#endif
template<i960Pinout pin>
inline void pulse(decltype(HIGH) from = HIGH, decltype(LOW) to = LOW) noexcept {
#ifdef ARDUINO_AVR_ATmega1284
    // save registers and do the pulse
    uint8_t theSREG = SREG;
    cli();
    auto& thePort = getAssociatedOutputPort<pin>();
    thePort ^= getPinMask<pin>();
    thePort ^= getPinMask<pin>();
    SREG = theSREG;
#else
    // make sure that we stay at the high signal for long enough to matter
    digitalWrite(pin, from);
#ifdef ARDUINO_ARCH_RP2040
    asm volatile ("nop");
    asm volatile ("nop");
    asm volatile ("nop");
    asm volatile ("nop");
#endif
    digitalWrite(pin, to);
#ifdef ARDUINO_ARCH_RP2040
    asm volatile ("nop");
    asm volatile ("nop");
    asm volatile ("nop");
    asm volatile ("nop");
#endif
    // then come back up after four cycles or so (tweak this as needed)
    digitalWrite(pin, from);
#ifdef ARDUINO_ARCH_RP2040
    asm volatile ("nop");
    asm volatile ("nop");
    asm volatile ("nop");
    asm volatile ("nop");
#endif
#endif
}

template<i960Pinout pin, decltype(HIGH) value>
inline void digitalWrite() {
#ifdef ARDUINO_AVR_ATmega1284
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
    digitalWrite(pin, value);
#endif
}
template<i960Pinout pin>
inline void digitalWrite(decltype(HIGH) value) noexcept {
#ifdef ARDUINO_AVR_ATmega1284
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
    digitalWrite(pin, value);
#endif
}

template<i960Pinout pin>
inline auto digitalRead() noexcept {
#ifdef ARDUINO_AVR_ATmega1284
    return (getAssociatedInputPort<pin>() & getPinMask<pin>()) ? HIGH : LOW;
#else
    return digitalRead(pin);
#endif
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
        static constexpr auto getDeassertionState() noexcept { return deasserted; } \
        inline static void assertPin() noexcept { digitalWrite<pin,getAssertionState()>(); } \
        inline static void deassertPin() noexcept { digitalWrite<pin,getDeassertionState()>(); } \
        inline static void write(decltype(LOW) value) noexcept { digitalWrite<pin>(value); } \
        static constexpr auto valid() noexcept { return isValidPin960_v<pin>; } \
        inline static void pulse() noexcept {   \
            ::pulse<pin>();                                     \
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

DefSPICSPin(i960Pinout::GPIOSelect);
#ifdef ALLOW_SRAM_CACHE
DefSPICSPin(i960Pinout::CACHE_EN_);
#endif
DefSPICSPin(i960Pinout::SD_EN);
DefSPICSPin(i960Pinout::PSRAM_EN);
DefSPICSPin(i960Pinout::DISPLAY_EN);
DefOutputPin(i960Pinout::Reset960, LOW, HIGH);
DefOutputPin(i960Pinout::Ready, LOW, HIGH);
DefInputPin(i960Pinout::FAIL, HIGH, LOW);
DefInputPin(i960Pinout::DEN_, LOW, HIGH);
DefInputPin(i960Pinout::BLAST_, LOW, HIGH);
DefInputPin(i960Pinout::W_R_, LOW, HIGH);
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
