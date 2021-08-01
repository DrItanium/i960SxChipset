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
/**
 * @brief Describes the directly pulled bits from PORTA that describe a unique transaction that the chipset will carry out
 */
enum class TransactionDescription : uint8_t {
    Last16BitValueReadOffset0 = 0b0000'0000,
    Last16BitValueWriteOffset0 = 0b0000'0001,
    Last16BitValueReadOffset1 = 0b0000'0010,
    Last16BitValueWriteOffset1 = 0b0000'0011,
    Last16BitValueReadOffset2 = 0b0000'0100,
    Last16BitValueWriteOffset2 = 0b0000'0101,
    Last16BitValueReadOffset3 = 0b0000'0110,
    Last16BitValueWriteOffset3 = 0b0000'0111,
    Last16BitValueReadOffset4 = 0b0000'1000,
    Last16BitValueWriteOffset4 = 0b0000'1001,
    Last16BitValueReadOffset5 = 0b0000'1010,
    Last16BitValueWriteOffset5 = 0b0000'1011,
    Last16BitValueReadOffset6 = 0b0000'1100,
    Last16BitValueWriteOffset6 = 0b0000'1101,
    Last16BitValueReadOffset7 = 0b0000'1110,
    Last16BitValueWriteOffset7 = 0b0000'1111,
    LastUpper8BitValueReadOffset0 = 0b0001'0000,
    LastUpper8BitValueWriteOffset0 = 0b0001'0001,
    LastUpper8BitValueReadOffset1 = 0b0001'0010,
    LastUpper8BitValueWriteOffset1 = 0b0001'0011,
    LastUpper8BitValueReadOffset2 = 0b0001'0100,
    LastUpper8BitValueWriteOffset2 = 0b0001'0101,
    LastUpper8BitValueReadOffset3 = 0b0001'0110,
    LastUpper8BitValueWriteOffset3 = 0b0001'0111,
    LastUpper8BitValueReadOffset4 = 0b0001'1000,
    LastUpper8BitValueWriteOffset4 = 0b0001'1001,
    LastUpper8BitValueReadOffset5 = 0b0001'1010,
    LastUpper8BitValueWriteOffset5 = 0b0001'1011,
    LastUpper8BitValueReadOffset6 = 0b0001'1100,
    LastUpper8BitValueWriteOffset6 = 0b0001'1101,
    LastUpper8BitValueReadOffset7 = 0b0001'1110,
    LastUpper8BitValueWriteOffset7 = 0b0001'1111,
    LastLower8BitValueReadOffset0  = 0b0010'0000,
    LastLower8BitValueWriteOffset0 = 0b0010'0001,
    LastLower8BitValueReadOffset1  = 0b0010'0010,
    LastLower8BitValueWriteOffset1 = 0b0010'0011,
    LastLower8BitValueReadOffset2  = 0b0010'0100,
    LastLower8BitValueWriteOffset2 = 0b0010'0101,
    LastLower8BitValueReadOffset3  = 0b0010'0110,
    LastLower8BitValueWriteOffset3 = 0b0010'0111,
    LastLower8BitValueReadOffset4  = 0b0010'1000,
    LastLower8BitValueWriteOffset4 = 0b0010'1001,
    LastLower8BitValueReadOffset5  = 0b0010'1010,
    LastLower8BitValueWriteOffset5 = 0b0010'1011,
    LastLower8BitValueReadOffset6  = 0b0010'1100,
    LastLower8BitValueWriteOffset6 = 0b0010'1101,
    LastLower8BitValueReadOffset7  = 0b0010'1110,
    LastLower8BitValueWriteOffset7 = 0b0010'1111,
    ChecksumFailureValueReadOffset0  = 0b0011'0000,
    ChecksumFailureValueWriteOffset0 = 0b0011'0001,
    ChecksumFailureValueReadOffset1  = 0b0011'0010,
    ChecksumFailureValueWriteOffset1 = 0b0011'0011,
    ChecksumFailureValueReadOffset2  = 0b0011'0100,
    ChecksumFailureValueWriteOffset2 = 0b0011'0101,
    ChecksumFailureValueReadOffset3  = 0b0011'0110,
    ChecksumFailureValueWriteOffset3 = 0b0011'0111,
    ChecksumFailureValueReadOffset4  = 0b0011'1000,
    ChecksumFailureValueWriteOffset4 = 0b0011'1001,
    ChecksumFailureValueReadOffset5  = 0b0011'1010,
    ChecksumFailureValueWriteOffset5 = 0b0011'1011,
    ChecksumFailureValueReadOffset6  = 0b0011'1100,
    ChecksumFailureValueWriteOffset6 = 0b0011'1101,
    ChecksumFailureValueReadOffset7  = 0b0011'1110,
    ChecksumFailureValueWriteOffset7 = 0b0011'1111,
    Burst16BitValueReadOffset0       = 0b0100'0000,
    Burst16BitValueWriteOffset0      = 0b0100'0001,
    Burst16BitValueReadOffset1       = 0b0100'0010,
    Burst16BitValueWriteOffset1      = 0b0100'0011,
    Burst16BitValueReadOffset2       = 0b0100'0100,
    Burst16BitValueWriteOffset2      = 0b0100'0101,
    Burst16BitValueReadOffset3       = 0b0100'0110,
    Burst16BitValueWriteOffset3      = 0b0100'0111,
    Burst16BitValueReadOffset4       = 0b0100'1000,
    Burst16BitValueWriteOffset4      = 0b0100'1001,
    Burst16BitValueReadOffset5       = 0b0100'1010,
    Burst16BitValueWriteOffset5      = 0b0100'1011,
    Burst16BitValueReadOffset6       = 0b0100'1100,
    Burst16BitValueWriteOffset6      = 0b0100'1101,
    Burst16BitValueReadOffset7       = 0b0100'1110,
    Burst16BitValueWriteOffset7      = 0b0100'1111,
    BurstUpper8BitValueReadOffset0   = 0b0101'0000,
    BurstUpper8BitValueWriteOffset0  = 0b0101'0001,
    BurstUpper8BitValueReadOffset1   = 0b0101'0010,
    BurstUpper8BitValueWriteOffset1  = 0b0101'0011,
    BurstUpper8BitValueReadOffset2   = 0b0101'0100,
    BurstUpper8BitValueWriteOffset2  = 0b0101'0101,
    BurstUpper8BitValueReadOffset3   = 0b0101'0110,
    BurstUpper8BitValueWriteOffset3  = 0b0101'0111,
    BurstUpper8BitValueReadOffset4   = 0b0101'1000,
    BurstUpper8BitValueWriteOffset4  = 0b0101'1001,
    BurstUpper8BitValueReadOffset5   = 0b0101'1010,
    BurstUpper8BitValueWriteOffset5  = 0b0101'1011,
    BurstUpper8BitValueReadOffset6   = 0b0101'1100,
    BurstUpper8BitValueWriteOffset6  = 0b0101'1101,
    BurstUpper8BitValueReadOffset7   = 0b0101'1110,
    BurstUpper8BitValueWriteOffset7  = 0b0101'1111,
    BurstLower8BitValueReadOffset0   = 0b0110'0000,
    BurstLower8BitValueWriteOffset0  = 0b0110'0001,
    BurstLower8BitValueReadOffset1   = 0b0110'0010,
    BurstLower8BitValueWriteOffset1  = 0b0110'0011,
    BurstLower8BitValueReadOffset2   = 0b0110'0100,
    BurstLower8BitValueWriteOffset2  = 0b0110'0101,
    BurstLower8BitValueReadOffset3   = 0b0110'0110,
    BurstLower8BitValueWriteOffset3  = 0b0110'0111,
    BurstLower8BitValueReadOffset4   = 0b0110'1000,
    BurstLower8BitValueWriteOffset4  = 0b0110'1001,
    BurstLower8BitValueReadOffset5   = 0b0110'1010,
    BurstLower8BitValueWriteOffset5  = 0b0110'1011,
    BurstLower8BitValueReadOffset6   = 0b0110'1100,
    BurstLower8BitValueWriteOffset6  = 0b0110'1101,
    BurstLower8BitValueReadOffset7   = 0b0110'1110,
    BurstLower8BitValueWriteOffset7  = 0b0110'1111,
    ErrorValueReadOffset0            = 0b0111'0000,
    ErrorValueWriteOffset0           = 0b0111'0001,
    ErrorValueReadOffset1            = 0b0111'0010,
    ErrorValueWriteOffset1           = 0b0111'0011,
    ErrorValueReadOffset2            = 0b0111'0100,
    ErrorValueWriteOffset2           = 0b0111'0101,
    ErrorValueReadOffset3            = 0b0111'0110,
    ErrorValueWriteOffset3           = 0b0111'0111,
    ErrorValueReadOffset4            = 0b0111'1000,
    ErrorValueWriteOffset4           = 0b0111'1001,
    ErrorValueReadOffset5            = 0b0111'1010,
    ErrorValueWriteOffset5           = 0b0111'1011,
    ErrorValueReadOffset6            = 0b0111'1100,
    ErrorValueWriteOffset6           = 0b0111'1101,
    ErrorValueReadOffset7            = 0b0111'1110,
    ErrorValueWriteOffset7           = 0b0111'1111,
};
/// @todo fix this pinout for different targets
enum class i960Pinout : decltype(A0) {
        // this is described in digial pin order!
        // leave this one alone
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
    Count,          // special, must be last
    // PORT B
    Ready = PORT_B0,
    CLOCK_OUT = PORT_B1,
    AS_ = PORT_B2,
    PSRAM_EN = PORT_B3,
    GPIOSelect = PORT_B4,
    MOSI = PORT_B5,          // reserved
    MISO = PORT_B6,          // reserved
    SCK = PORT_B7,          // reserved
    RX0 = PORT_D0,          // reserved
    TX0 = PORT_D1,          // reserved
    DEN_ = PORT_D2,      // AVR Interrupt INT0
    CACHE_EN_ = PORT_D3,
    // PD4 is not used by simple management card
    Reset960= PORT_D5,
    Int0_ = PORT_D6,
    // PD7 is not used by simple management card
    SCL = PORT_C0,
    SDA = PORT_C1,
    SPI_OFFSET0 = PORT_C2,
    SPI_OFFSET1 = PORT_C3,
    SPI_OFFSET2 = PORT_C4,
    DISPLAY_EN = PORT_C5,
    DC = PORT_C6,     // output
    SD_EN = PORT_C7,      // output
    W_R_ = PORT_A0,
    BA1 = PORT_A1,
    BA2 = PORT_A2,
    BA3 = PORT_A3,
    BE0 = PORT_A4,
    BE1 = PORT_A5,
    BLAST_ = PORT_A6,     // input
    FAIL = PORT_A7,         // input

};
template<i960Pinout pin>
constexpr bool isValidPin = static_cast<byte>(pin) < static_cast<byte>(i960Pinout::Count);
static_assert(!isValidPin<i960Pinout::Count>, "The Count \"pin\" should be an invalid pin!");
//static_assert(isValidPin<i960Pinout::CACHE_A0>, "The CACHE_A0 pin should be a valid pin!");
template<i960Pinout pin>
[[nodiscard]] inline volatile unsigned char& getAssociatedOutputPort() noexcept {
    static_assert(isValidPin<pin>, "INVALID PIN PROVIDED");
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
[[nodiscard]] inline volatile unsigned char& getAssociatedInputPort() noexcept {
    static_assert(isValidPin<pin>, "INVALID PIN PROVIDED");
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
    static_assert(isValidPin<pin>, "INVALID PIN PROVIDED");
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

template<i960Pinout pin>
inline void pulse() noexcept {
    // save registers and do the pulse
    uint8_t theSREG = SREG;
    cli();
    auto& thePort = getAssociatedOutputPort<pin>();
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
inline void digitalWrite() {
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

inline void digitalWrite(i960Pinout ip, decltype(HIGH) value) {
    digitalWrite(static_cast<int>(ip), value);
}

inline void pinMode(i960Pinout ip, decltype(INPUT) value) {
    pinMode(static_cast<int>(ip), value);
}
template<i960Pinout pin>
inline auto digitalRead() noexcept {
    return (getAssociatedInputPort<pin>() & getPinMask<pin>()) ? HIGH : LOW;
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
#define DefSPICSPin(pin) DefOutputPin(pin, LOW, HIGH)

DefSPICSPin(i960Pinout::GPIOSelect);
DefSPICSPin(i960Pinout::CACHE_EN_);
DefSPICSPin(i960Pinout::SD_EN);
DefSPICSPin(i960Pinout::PSRAM_EN);
DefSPICSPin(i960Pinout::DISPLAY_EN);
DefOutputPin(i960Pinout::Reset960, LOW, HIGH);
DefOutputPin(i960Pinout::Ready, LOW, HIGH);
DefInputPin(i960Pinout::FAIL, HIGH, LOW);
DefInputPin(i960Pinout::DEN_, LOW, HIGH);
DefInputPin(i960Pinout::AS_, LOW, HIGH);
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
