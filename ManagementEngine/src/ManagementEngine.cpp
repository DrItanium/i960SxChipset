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

/// i960Sx chipset mcu, based on atmega1284p with fuses set for:
/// - 20Mhz crystal
/// - D1 acts as CLKO
/// Language options:
/// - C++17
/// Board Platform: MightyCore
#include <Arduino.h>

/// @todo fix this pinout for different targets
enum class i960Pinout : decltype(A0) {
    Digital_PB0 = 0,
    Digital_PB1 ,
    Digital_PB2 ,
    Digital_PB3,
    Digital_PB4,
    Digital_PB5,
    Digital_PB6,
    Digital_PB7,
    Digital_PD0,
    Digital_PD1 ,
    Digital_PD2 ,
    Digital_PD3,
    Digital_PD4,
    Digital_PD5,
    Digital_PD6,
    Digital_PD7,
    Digital_PC0,
    Digital_PC1 ,
    Digital_PC2 ,
    Digital_PC3,
    Digital_PC4,
    Digital_PC5,
    Digital_PC6,
    Digital_PC7,
    Digital_PA0,
    Digital_PA1 ,
    Digital_PA2 ,
    Digital_PA3,
    Digital_PA4,
    Digital_PA5,
    Digital_PA6,
    Digital_PA7,
    Count,
    Led = Digital_PB0,
    CLKO = Digital_PB1,
    AS_ = Digital_PB2,
    BLAST_ = Digital_PB3, // input
    FAIL = Digital_PB4, // input
    RX0 = Digital_PD0,
    TX0 = Digital_PD1,
    DEN_ = Digital_PD2,      // AVR Interrupt INT0
    CYCLE_READY_ = Digital_PD3,        // Output, AVR Interrupt INT1
    // PORT C contains all of the outputs we need so I can directly manipulate the port on startup
    Reset960 = Digital_PC0, // output
    SYSTEM_FAIL_ = Digital_PC1,
    NEW_REQUEST_ = Digital_PC2,
    SUCCESSFUL_BOOT_ = Digital_PC3, // output
    Int0_ = Digital_PC4, // output
    Ready = Digital_PC5, // output
};
template<i960Pinout pin>
constexpr bool isValidPin = static_cast<byte>(pin) < static_cast<byte>(i960Pinout::Count);
static_assert(!isValidPin<i960Pinout::Count>, "The Count \"pin\" should be an invalid pin!");
static_assert(isValidPin<i960Pinout::Led>, "The Led pin should be a valid pin!");
template<i960Pinout pin>
[[nodiscard]] inline volatile unsigned char& getAssociatedOutputPort() noexcept {
    static_assert(isValidPin<pin>, "INVALID PIN PROVIDED");
    switch (pin) {
        case i960Pinout::Digital_PA0: // output
        case i960Pinout::Digital_PA1:
        case i960Pinout::Digital_PA2:
        case i960Pinout::Digital_PA3:
        case i960Pinout::Digital_PA4:
        case i960Pinout::Digital_PA5:
        case i960Pinout::Digital_PA6:
        case i960Pinout::Digital_PA7:
            return PORTA;
        case i960Pinout::Digital_PB0: // output
        case i960Pinout::Digital_PB1:
        case i960Pinout::Digital_PB2:
        case i960Pinout::Digital_PB3:
        case i960Pinout::Digital_PB4:
        case i960Pinout::Digital_PB5:
        case i960Pinout::Digital_PB6:
        case i960Pinout::Digital_PB7:
            return PORTB;
        case i960Pinout::Digital_PC0: // output
        case i960Pinout::Digital_PC1:
        case i960Pinout::Digital_PC2:
        case i960Pinout::Digital_PC3:
        case i960Pinout::Digital_PC4:
        case i960Pinout::Digital_PC5:
        case i960Pinout::Digital_PC6:
        case i960Pinout::Digital_PC7:
            return PORTC;
        case i960Pinout::Digital_PD0: // output
        case i960Pinout::Digital_PD1:
        case i960Pinout::Digital_PD2:
        case i960Pinout::Digital_PD3:
        case i960Pinout::Digital_PD4:
        case i960Pinout::Digital_PD5:
        case i960Pinout::Digital_PD6:
        case i960Pinout::Digital_PD7:
            return PORTD;
        default:
            return PORTA;
    }
}

template<i960Pinout pin>
[[nodiscard]] inline volatile unsigned char& getAssociatedInputPort() noexcept {
    static_assert(isValidPin<pin>, "INVALID PIN PROVIDED");
    switch (pin) {
        case i960Pinout::Digital_PA0: // output
        case i960Pinout::Digital_PA1:
        case i960Pinout::Digital_PA2:
        case i960Pinout::Digital_PA3:
        case i960Pinout::Digital_PA4:
        case i960Pinout::Digital_PA5:
        case i960Pinout::Digital_PA6:
        case i960Pinout::Digital_PA7:
            return PINA;
        case i960Pinout::Digital_PB0: // output
        case i960Pinout::Digital_PB1:
        case i960Pinout::Digital_PB2:
        case i960Pinout::Digital_PB3:
        case i960Pinout::Digital_PB4:
        case i960Pinout::Digital_PB5:
        case i960Pinout::Digital_PB6:
        case i960Pinout::Digital_PB7:
            return PINB;
        case i960Pinout::Digital_PC0: // output
        case i960Pinout::Digital_PC1:
        case i960Pinout::Digital_PC2:
        case i960Pinout::Digital_PC3:
        case i960Pinout::Digital_PC4:
        case i960Pinout::Digital_PC5:
        case i960Pinout::Digital_PC6:
        case i960Pinout::Digital_PC7:
            return PINC;
        case i960Pinout::Digital_PD0: // output
        case i960Pinout::Digital_PD1:
        case i960Pinout::Digital_PD2:
        case i960Pinout::Digital_PD3:
        case i960Pinout::Digital_PD4:
        case i960Pinout::Digital_PD5:
        case i960Pinout::Digital_PD6:
        case i960Pinout::Digital_PD7:
            return PIND;
        default:
            return PINA;
    }
}
template<i960Pinout pin>
[[nodiscard]] constexpr decltype(auto) getPinMask() noexcept {
    static_assert(isValidPin<pin>, "INVALID PIN PROVIDED");
    switch (pin) {
        case i960Pinout::Digital_PA0:        return _BV(PA0);
        case i960Pinout::Digital_PA1:        return _BV(PA1);
        case i960Pinout::Digital_PA2:      return _BV(PA2);
        case i960Pinout::Digital_PA3:      return _BV(PA3);
        case i960Pinout::Digital_PA4:       return _BV(PA4);
        case i960Pinout::Digital_PA5:   return _BV(PA5);
        case i960Pinout::Digital_PA6:     return _BV(PA6);
        case i960Pinout::Digital_PA7:       return _BV(PA7);
        case i960Pinout::Digital_PB0:        return _BV(PB0);
        case i960Pinout::Digital_PB1:        return _BV(PB1);
        case i960Pinout::Digital_PB2:      return _BV(PB2);
        case i960Pinout::Digital_PB3:      return _BV(PB3);
        case i960Pinout::Digital_PB4:       return _BV(PB4);
        case i960Pinout::Digital_PB5:   return _BV(PB5);
        case i960Pinout::Digital_PB6:     return _BV(PB6);
        case i960Pinout::Digital_PB7:       return _BV(PB7);
        case i960Pinout::Digital_PC0:        return _BV(PC0);
        case i960Pinout::Digital_PC1:        return _BV(PC1);
        case i960Pinout::Digital_PC2:      return _BV(PC2);
        case i960Pinout::Digital_PC3:      return _BV(PC3);
        case i960Pinout::Digital_PC4:       return _BV(PC4);
        case i960Pinout::Digital_PC5:   return _BV(PC5);
        case i960Pinout::Digital_PC6:     return _BV(PC6);
        case i960Pinout::Digital_PC7:       return _BV(PC7);
        case i960Pinout::Digital_PD0:        return _BV(PD0);
        case i960Pinout::Digital_PD1:        return _BV(PD1);
        case i960Pinout::Digital_PD2:      return _BV(PD2);
        case i960Pinout::Digital_PD3:      return _BV(PD3);
        case i960Pinout::Digital_PD4:       return _BV(PD4);
        case i960Pinout::Digital_PD5:   return _BV(PD5);
        case i960Pinout::Digital_PD6:     return _BV(PD6);
        case i960Pinout::Digital_PD7:       return _BV(PD7);
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

template<i960Pinout pin, decltype(HIGH) value>
inline void digitalWrite() {
    uint8_t theSREG = SREG;
    cli();
    if constexpr (auto& thePort = getAssociatedOutputPort<pin>(); value == LOW) {
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
    if (auto& thePort = getAssociatedOutputPort<pin>(); value == LOW) {
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

DefOutputPin(i960Pinout::Reset960, LOW, HIGH);
DefOutputPin(i960Pinout::Ready, LOW, HIGH);
DefOutputPin(i960Pinout::SYSTEM_FAIL_, LOW, HIGH);
DefOutputPin(i960Pinout::SUCCESSFUL_BOOT_, LOW, HIGH);
DefOutputPin(i960Pinout::NEW_REQUEST_, LOW, HIGH);

DefInputPin(i960Pinout::FAIL, HIGH, LOW);
DefInputPin(i960Pinout::DEN_, LOW, HIGH);
DefInputPin(i960Pinout::AS_, LOW, HIGH);
DefInputPin(i960Pinout::CYCLE_READY_, LOW, HIGH);
DefInputPin(i960Pinout::BLAST_, LOW, HIGH);
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

volatile bool asTriggered = false;
volatile bool denTriggered = false;
volatile bool signalProcessorReady = false;
void onASAsserted() {
    asTriggered = true;
}
void onDENAsserted() {
    denTriggered = true;
}
void onSPRAsserted() {
    signalProcessorReady = true;
}

// the setup routine runs once when you press reset:
void setup() {
    // setup the output port c as fast as possible
    DDRC = 0xFF; // all pins are outputs
    PORTC = 0b1111'1110; // PB0 is RESET960. Pull the i960 into reset state
    // now configure the rest of the pins
    pinMode(i960Pinout::Led, OUTPUT);
    digitalWrite<i960Pinout::Led, LOW>();
    // all of these pins need to be pulled high
    setupPins(INPUT,
              i960Pinout::CYCLE_READY_,
              i960Pinout::AS_,
              i960Pinout::DEN_,
              i960Pinout::FAIL,
              i960Pinout::BLAST_);
    // configure all of the interrupts to operate on falling edges
    attachInterrupt(digitalPinToInterrupt(static_cast<int>(i960Pinout::AS_)), onASAsserted, FALLING);
    attachInterrupt(digitalPinToInterrupt(static_cast<int>(i960Pinout::DEN_)), onDENAsserted, FALLING);
    attachInterrupt(digitalPinToInterrupt(static_cast<int>(i960Pinout::CYCLE_READY_)), onSPRAsserted, FALLING);
    // then wait for a little bit to make sure that we have actually
    delay(1000);
    // pull the i960 out of reset
    digitalWrite<i960Pinout::Reset960, HIGH>();
    // at this point we have started execution of the i960
    // wait until we enter self test state
    while (DigitalPin<i960Pinout::FAIL>::isDeasserted());
    // now wait until we leave self test state
    while (DigitalPin<i960Pinout::FAIL>::isAsserted());
    // at this point we are in idle so we are safe to loaf around a bit
    DigitalPin<i960Pinout::SUCCESSFUL_BOOT_>::assertPin();
}
// ----------------------------------------------------------------
// state machine
// ----------------------------------------------------------------
// The bootup process has a separate set of states
// TStart - Where we start
// TSystemTest - Processor performs self test
//
// TStart -> TSystemTest via FAIL being asserted
// TSystemTest -> Ti via FAIL being deasserted
//
// State machine will stay here for the duration
// State diagram based off of i960SA/SB Reference manual
// Basic Bus States
// Ti - Idle State
// Ta - Address State
// Td - Data State
// Tw - Wait State

// READY - ~READY asserted
// NOT READY - ~READY not asserted
// BURST - ~BLAST not asserted
// NO BURST - ~BLAST asserted
// NEW REQUEST - ~AS asserted
// NO REQUEST - ~AS not asserted when in

// Ti -> Ti via no request
// Ti -> Ta via new request
// on enter of Ta, set address state to false
// on enter of Td, burst is sampled
// Ta -> Td
// Td -> Ti after signaling ready and no burst (blast low)
// Td -> Td after signaling ready and burst (blast high)
// Ti -> TChecksumFailure if FAIL is asserted

// NOTE: Tw may turn out to be synthetic
void loop() {
    //fsm.run_machine();
    if (DigitalPin<i960Pinout::FAIL>::isAsserted()) {
        /// @todo trigger a control line to signify a system failure
        DigitalPin<i960Pinout::SYSTEM_FAIL_>::assertPin();
        while(true) {
            delay(1000);
        }
    }
    // both as and den must be triggered before we can actually execute.
    // the i960 manual states we can do things in between but the time for this is so small
    // I think it is smarter to just wait. I can easily add a read address lines signal if needed
    while (!asTriggered && !denTriggered);
    denTriggered = false;
    asTriggered = false;
    // now we model the basic design of the memory transaction process
    do {
        DigitalPin<i960Pinout::NEW_REQUEST_>::pulse();
        auto isBlastLast = DigitalPin<i960Pinout::BLAST_>::isAsserted();
        while (!signalProcessorReady);
        DigitalPin<i960Pinout::Ready>::pulse();
        signalProcessorReady = false;
        if (isBlastLast) {
            break;
        }
    } while (true);
}