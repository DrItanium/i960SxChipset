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
    // this is described in digial pin order!
    // leave this one alone
    // PORT B
    Led = 0,      // output
    CLOCK_OUT, // output, unusable
    AS_,     // input, AVR Int2
    SUCCESSFUL_BOOT_,     // output to chipset
    NEW_REQUEST_,          // output to chipset
    Ready,          // output to i960
    Int0_,          // output to i960
    SYSTEM_FAIL_,          // output to Mi
// PORT D
    RX0,          // reserved
    TX0,          // reserved
    DEN_,      // AVR Interrupt INT0
    CYCLE_READY_,        // Output, AVR Interrupt INT1
    Reset960, // output
    Digital_PD5,     // output
    Digital_PD6, // output
    Digital_PD7,      // output
// PORT C => Output State?
    Digital_PC0, // output
    Digital_PC1,
    Digital_PC2,
    Digital_PC3,
    Digital_PC4,
    Digital_PC5,
    Digital_PC6,
    Digital_PC7,
// PORT A, used to select the spi bus address (not directly used)
    W_R_,
    BurstAddress1,
    BurstAddress2,
    BurstAddress3,
    ByteEnable0,
    ByteEnable1,
    BLAST_,
    FAIL,
    Count,          // special, must be last

};
template<i960Pinout pin>
constexpr bool isValidPin = static_cast<byte>(pin) < static_cast<byte>(i960Pinout::Count);
static_assert(!isValidPin<i960Pinout::Count>, "The Count \"pin\" should be an invalid pin!");
static_assert(isValidPin<i960Pinout::Led>, "The Led pin should be a valid pin!");
template<i960Pinout pin>
[[nodiscard]] inline volatile unsigned char& getAssociatedOutputPort() noexcept {
    static_assert(isValidPin<pin>, "INVALID PIN PROVIDED");
    switch (pin) {
        case i960Pinout::W_R_:
        case i960Pinout::BurstAddress1:
        case i960Pinout::BurstAddress2:
        case i960Pinout::BurstAddress3:
        case i960Pinout::ByteEnable0:
        case i960Pinout::ByteEnable1:
        case i960Pinout::BLAST_:
        case i960Pinout::FAIL:
            return PORTA;
        case i960Pinout::Digital_PC0: // output
        case i960Pinout::Digital_PC1:
        case i960Pinout::Digital_PC2:
        case i960Pinout::Digital_PC3:
        case i960Pinout::Digital_PC4:
        case i960Pinout::Digital_PC5:
        case i960Pinout::Digital_PC6:
        case i960Pinout::Digital_PC7:
            return PORTC;
        case i960Pinout::Led:      // output
        case i960Pinout::CLOCK_OUT: // output: unusable
        case i960Pinout::AS_:     // input: AVR Int2
        case i960Pinout::SUCCESSFUL_BOOT_:     // output to chipset
        case i960Pinout::NEW_REQUEST_:          // output to chipset
        case i960Pinout::Ready:          // output to i960
        case i960Pinout::Int0_:          // output to i960
        case i960Pinout::SYSTEM_FAIL_:          // output to Mi
            return PORTB;
        case i960Pinout::RX0:          // reserved
        case i960Pinout::TX0:          // reserved
        case i960Pinout::DEN_:      // AVR Interrupt INT0
        case i960Pinout::CYCLE_READY_:        // AVR Interrupt INT1
        case i960Pinout::Reset960: // output
        case i960Pinout::Digital_PD5:     // output
        case i960Pinout::Digital_PD6: // output
        case i960Pinout::Digital_PD7:      // output
            return PORTD;
        default:
            return PORTA;
    }
}

template<i960Pinout pin>
[[nodiscard]] inline volatile unsigned char& getAssociatedInputPort() noexcept {
    static_assert(isValidPin<pin>, "INVALID PIN PROVIDED");
    switch (pin) {
        case i960Pinout::W_R_:
        case i960Pinout::BurstAddress1:
        case i960Pinout::BurstAddress2:
        case i960Pinout::BurstAddress3:
        case i960Pinout::ByteEnable0:
        case i960Pinout::ByteEnable1:
        case i960Pinout::BLAST_:
        case i960Pinout::FAIL:
            return PINA;
        case i960Pinout::Digital_PC0: // output
        case i960Pinout::Digital_PC1:
        case i960Pinout::Digital_PC2:
        case i960Pinout::Digital_PC3:
        case i960Pinout::Digital_PC4:
        case i960Pinout::Digital_PC5:
        case i960Pinout::Digital_PC6:
        case i960Pinout::Digital_PC7:
            return PINC;
        case i960Pinout::Led:      // output
        case i960Pinout::CLOCK_OUT: // output: unusable
        case i960Pinout::AS_:     // input: AVR Int2
        case i960Pinout::SUCCESSFUL_BOOT_:     // output to chipset
        case i960Pinout::NEW_REQUEST_:          // output to chipset
        case i960Pinout::Ready:          // output to i960
        case i960Pinout::Int0_:          // output to i960
        case i960Pinout::SYSTEM_FAIL_:          // output to Mi
            return PINB;
        case i960Pinout::RX0:          // reserved
        case i960Pinout::TX0:          // reserved
        case i960Pinout::DEN_:      // AVR Interrupt INT0
        case i960Pinout::CYCLE_READY_:        // AVR Interrupt INT1
        case i960Pinout::Reset960: // output
        case i960Pinout::Digital_PD5:     // output
        case i960Pinout::Digital_PD6: // output
        case i960Pinout::Digital_PD7:      // output
            return PIND;
        default:
            return PINA;
    }
}
template<i960Pinout pin>
[[nodiscard]] constexpr decltype(auto) getPinMask() noexcept {
    static_assert(isValidPin<pin>, "INVALID PIN PROVIDED");
    switch (pin) {
        case i960Pinout::W_R_: return _BV(PA0) ;
        case i960Pinout::BurstAddress1: return _BV(PA1);
        case i960Pinout::BurstAddress2: return _BV(PA2);
        case i960Pinout::BurstAddress3: return _BV(PA3);
        case i960Pinout::ByteEnable0: return _BV(PA4);
        case i960Pinout::ByteEnable1: return _BV(PA5);
        case i960Pinout::BLAST_: return _BV(PA6);
        case i960Pinout::FAIL: return _BV(PA7);
        case i960Pinout::Digital_PC0:        return _BV(PC0);
        case i960Pinout::Digital_PC1:        return _BV(PC1);
        case i960Pinout::Digital_PC2:      return _BV(PC2);
        case i960Pinout::Digital_PC3:      return _BV(PC3);
        case i960Pinout::Digital_PC4:       return _BV(PC4);
        case i960Pinout::Digital_PC5:   return _BV(PC5);
        case i960Pinout::Digital_PC6:     return _BV(PC6);
        case i960Pinout::Digital_PC7:       return _BV(PC7);

        case i960Pinout::Led:      return _BV(PB0);
        case i960Pinout::CLOCK_OUT:  return _BV(PB1);
        case i960Pinout::AS_:     return _BV(PB2);
        case i960Pinout::SUCCESSFUL_BOOT_: return _BV(PB3);
        case i960Pinout::NEW_REQUEST_:     return _BV(PB4);
        case i960Pinout::Ready:          return _BV(PB5);
        case i960Pinout::Int0_:          return _BV(PB6);
        case i960Pinout::SYSTEM_FAIL_:    return _BV(PB7);
        case i960Pinout::RX0:          return _BV(PD0);
        case i960Pinout::TX0:          return _BV(PD1);
        case i960Pinout::DEN_:          return _BV(PD2);
        case i960Pinout::CYCLE_READY_:        return _BV(PD3);
        case i960Pinout::Reset960: return _BV(PD4);
        case i960Pinout::Digital_PD5:     return _BV(PD5);
        case i960Pinout::Digital_PD6: return _BV(PD6);
        case i960Pinout::Digital_PD7:     return _BV(PD7);
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

template<i960Pinout pinId>
class PinAsserter {
public:
    static_assert(DigitalPin<pinId>::isOutputPin());
    PinAsserter() { DigitalPin<pinId>::assertPin(); }
    ~PinAsserter() { DigitalPin<pinId>::deassertPin(); }
};

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




// ----------------------------------------------------------------
// setup routines
// ----------------------------------------------------------------


// we have a second level cache of 1 megabyte in sram over spi

// the setup routine runs once when you press reset:
void setup() {
    // before we do anything else, configure as many pins as possible and then
    // pull the i960 into a reset state, it will remain this for the entire
    // duration of the setup function
    setupPins(OUTPUT,
              i960Pinout::Reset960,
              i960Pinout::Ready,
              i960Pinout::Led,
              i960Pinout::Int0_,
              i960Pinout::SYSTEM_FAIL_,
              i960Pinout::SUCCESSFUL_BOOT_);
    {
        PinAsserter<i960Pinout::Reset960> holdi960InReset;
        // all of these pins need to be pulled high
        digitalWriteBlock(HIGH,
                          i960Pinout::Ready,
                          i960Pinout::Int0_,
                          i960Pinout::SYSTEM_FAIL_,
                          i960Pinout::SUCCESSFUL_BOOT_);
        digitalWrite(i960Pinout::Led, LOW);
        setupPins(INPUT,
                  i960Pinout::CYCLE_READY_,
                  i960Pinout::AS_,
                  i960Pinout::DEN_,
                  i960Pinout::FAIL);

        attachInterrupt(digitalPinToInterrupt(static_cast<int>(i960Pinout::AS_)), onASAsserted, FALLING);
        attachInterrupt(digitalPinToInterrupt(static_cast<int>(i960Pinout::DEN_)), onDENAsserted, FALLING);
        attachInterrupt(digitalPinToInterrupt(static_cast<int>(i960Pinout::CYCLE_READY_)), onSPRAsserted, FALLING);
        //Serial.println(F("i960Sx chipset bringup"));
        // purge the cache pages
        delay(1000);
        //Serial.println(F("i960Sx chipset brought up fully!"));
    }
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
    // both as and den must be triggered before we can actually
    // wait until den is triggered via interrupt, we could even access the base address of the memory transaction
    while (!asTriggered && !denTriggered);
    denTriggered = false;
    asTriggered = false;
    do {
        DigitalPin<i960Pinout::NEW_REQUEST_>::pulse();
        auto isBlastLast = DigitalPin<i960Pinout::BLAST_>::isAsserted();
        while (!signalProcessorReady);
        DigitalPin<i960Pinout::Ready>::pulse();
        signalProcessorReady = false;
        if (isBlastLast) {
            break;
        }
        asm("nop"); // delay a couple of times to make sure
        asm("nop"); // delay a couple of times to make sure
    } while (true);
}


/// @todo Eliminate after MightyCore update
#if __cplusplus >= 201402L

void operator delete(void * ptr, size_t)
{
    ::operator delete(ptr);
}

void operator delete[](void * ptr, size_t)
{
    ::operator delete(ptr);
}

#endif // end language is C++14 or greater
