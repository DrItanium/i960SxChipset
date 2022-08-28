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
#include "EBI.h"

using Address = uint32_t;
/**
 * @brief Sx Load/Store styles that the processor will request
 */
enum class i960Pinout : int {
#define Entry(name) name,
#define PIN(name, port, offset)  Entry(name)
#define PORT(name, size)
#ifdef __AVR_ATmega1284P__
#include "1284pPinout.def"
#elif defined(__AVR_ATmega2560__)
#include "Mega2560PinoutFull.def"
#else
#error "Target Chipset Hardware has no pinout defined"
#endif
#undef PORT
#undef PIN
#undef Entry
#define X(name, pin, direction, assert, deassert, addr, haddr, offset) name = pin,
#define GPIO(name, pin, direction, assert, deassert) X(name, pin, direction, assert, deassert, 0, 0, 0)
#define EBI(name, pin, direction, assert, deassert, address, offset) X(name, pin, direction, assert, deassert, address, 0, offset)
#define IOEXP(name, pin, direction, assert, deassert, index, offset) X(name, pin, direction, assert, deassert, 0, index, offset)
#define SINK(name, pin, direction, assert, deassert) X(name, pin, direction, assert, deassert, 0, 0, 0)
#define ALIAS(name, pin) X(name, pin, INPUT, LOW, HIGH, 0, 0, 0)
#ifdef CHIPSET_TYPE1
#include "Type1Pinout.def"
#elif defined(CHIPSET_TYPE_MEGA)
#include "TypeMegaPinout.def"
#else
#error "No table defined!"
#endif
#undef GPIO
#undef EBI
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
#ifdef __AVR_ATmega1284P__
#include "1284pPinout.def"
#elif defined(__AVR_ATmega2560__)
#include "Mega2560PinoutFull.def"
#else
#error "No port map defined!"
#endif
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
#ifdef __AVR_ATmega1284P__
#include "1284pPinout.def"
#elif defined(__AVR_ATmega2560__)
                #include "Mega2560PinoutFull.def"
#else
#error "No port map defined!"
#endif
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
#ifdef __AVR_ATmega1284P__
#include "1284pPinout.def"
#elif defined(__AVR_ATmega2560__)
        #include "Mega2560PinoutFull.def"
#else
#error "No port map defined!"
#endif
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
#ifdef __AVR_ATmega1284P__
#include "1284pPinout.def"
#elif defined(__AVR_ATmega2560__)
        #include "Mega2560PinoutFull.def"
#else
#error "No port map defined!"
#endif
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
#ifdef __AVR_ATmega1284P__
#include "1284pPinout.def"
#elif defined(__AVR_ATmega2560__)
        #include "Mega2560PinoutFull.def"
#else
#error "No port map defined!"
#endif
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
#ifdef __AVR_ATmega1284P__
#include "1284pPinout.def"
#elif defined(__AVR_ATmega2560__)
        #include "Mega2560PinoutFull.def"
#else
#error "No port map defined!"
#endif
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
#ifdef __AVR_ATmega1284P__
#include "1284pPinout.def"
#elif defined(__AVR_ATmega2560__)
        #include "Mega2560PinoutFull.def"
#else
#error "No port map defined!"
#endif
#undef PIN
#undef PORT
#undef Entry
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

#define DefIOExpanderPin(pin, assert, deassert, direction, device, index) \
template<> \
struct DigitalPinDescription < i960Pinout:: pin> {\
    DigitalPinDescription() = delete; \
    ~DigitalPinDescription() = delete; \
    DigitalPinDescription(const DigitalPinDescription&) = delete;\
    DigitalPinDescription(DigitalPinDescription&&) = delete;\
    DigitalPinDescription& operator=(const DigitalPinDescription&) = delete; \
    DigitalPinDescription& operator=(DigitalPinDescription&&) = delete; \
    static constexpr auto Assert = assert; \
    static constexpr auto Deassert = deassert;\
    static constexpr auto Direction = direction;\
    static constexpr auto Pin =  i960Pinout:: pin;                                      \
    static constexpr auto DeviceID  = MCP23S17::HardwareDeviceAddress::Device ## device ; \
    static constexpr auto Offset = MCP23S17::PinIndex::Pin ## index; \
    using CSPin = DigitalPin<i960Pinout::GPIOSelect>;                     \
    using BackingImplementation = MCP23S17::BackingPin<Pin, DeviceID, Offset, Direction, CSPin>;\
}

DefIOExpanderPin(RESET960, LOW, HIGH, OUTPUT, 3, 0);
DefIOExpanderPin(HOLD, HIGH, LOW, OUTPUT, 3, 5);
DefIOExpanderPin(HLDA, HIGH, LOW, INPUT, 3, 6);
DefIOExpanderPin(LOCK_, LOW, HIGH, INPUT, 3, 7);

#ifdef CHIPSET_TYPE_MEGA
DefInputPin(i960Pinout::INT960_0_, LOW, HIGH);
DefInputPin(i960Pinout::INT960_1, HIGH, LOW);
DefInputPin(i960Pinout::INT960_2, HIGH, LOW);
DefInputPin(i960Pinout::INT960_3_, LOW, HIGH);
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
#define DefEBIPin(pin, address, offset, assert, deassert, direction) \
template<> \
struct DigitalPinDescription< pin > { \
        DigitalPinDescription() = delete; \
        ~DigitalPinDescription() = delete; \
        DigitalPinDescription(const DigitalPinDescription&) = delete; \
        DigitalPinDescription(DigitalPinDescription&&) = delete; \
        DigitalPinDescription& operator=(const DigitalPinDescription&) = delete; \
        DigitalPinDescription& operator=(DigitalPinDescription&&) = delete; \
    static constexpr auto Assert = assert; \
    static constexpr auto Deassert = deassert;\
    static constexpr auto Direction = direction;\
    static constexpr auto Pin =  pin;                                \
    static constexpr auto Index = EBI::Register8BitIndex::Bit ## offset ;        \
    static constexpr size_t Address = address; \
    using BackingImplementation = EBI::BackingDigitalPin<Pin, Address, Index>;\
}

DefEBIPin(i960Pinout::BE0, CTL0Register::Address, 0, LOW, HIGH, INPUT);
DefEBIPin(i960Pinout::BE1, CTL0Register::Address, 1, LOW, HIGH, INPUT);
DefEBIPin(i960Pinout::BLAST_, CTL0Register::Address, 2, LOW, HIGH, INPUT);
DefEBIPin(i960Pinout::FAIL, CTL0Register::Address, 3, HIGH, LOW, INPUT);
DefEBIPin(i960Pinout::RAM_SPACE_, CTL1Register::Address, 0, LOW, HIGH, INPUT);
DefEBIPin(i960Pinout::IO_SPACE_, CTL1Register::Address, 1, LOW, HIGH, INPUT);
DefEBIPin(i960Pinout::W_R_, AddressRegister::Address, 0, LOW, HIGH, INPUT);
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
DefInputPin(i960Pinout::IOEXP_INT0, LOW, HIGH);
DefInputPin(i960Pinout::IOEXP_INT1, LOW, HIGH);
DefInputPin(i960Pinout::IOEXP_INT2, LOW, HIGH);
DefInputPin(i960Pinout::IOEXP_INT3, LOW, HIGH);
DefIOExpanderPin(INT960_0_, LOW, HIGH, OUTPUT, 3, 1);
DefIOExpanderPin(INT960_1, HIGH, LOW, OUTPUT, 3, 2);
DefIOExpanderPin(INT960_2, HIGH, LOW, OUTPUT, 3, 3);
DefIOExpanderPin(INT960_3_, LOW, HIGH, OUTPUT, 3, 4);
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
