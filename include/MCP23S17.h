/*
i960SxChipset
Copyright (c) 2020-2022, Joshua Scoggins
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
//
// Created by jwscoggins on 8/28/22.
//

#ifndef SXCHIPSET_MCP23S17_H
#define SXCHIPSET_MCP23S17_H
#include <Arduino.h>
#include <SPI.h>
#include <MCUPlatform.h>

namespace MCP23S17 {
    /**
     * @brief The set of registers exposed by the MCP23S17 in the default bank mode
     */
    enum class Registers : byte {
        IODIRA = 0,
        IODIRB,
        IPOLA,
        IPOLB,
        GPINTENA,
        GPINTENB,
        DEFVALA,
        DEFVALB,
        INTCONA,
        INTCONB,
        IOCONA_,
        IOCONB_,
        GPPUA,
        GPPUB,
        INTFA,
        INTFB,
        INTCAPA,
        INTCAPB,
        GPIOA,
        GPIOB,
        OLATA,
        OLATB,
        OLAT = OLATA,
        GPIO = GPIOA,
        IOCON = IOCONA_,
        IODIR = IODIRA,
        INTCAP = INTCAPA,
        INTF = INTFA,
        GPPU = GPPUA,
        INTCON = INTCONA,
        DEFVAL = DEFVALA,
        GPINTEN = GPINTENA,
        IPOL = IPOLA,
    };
    /**
     * @brief The MCP23S17 devices connected to the single select pin. The MCP23S17 uses biased addressing to allow up to 8 io expanders to
     * use the same enable line. When hardware addressing is enabled, the address described via biasing is encoded into the spi data stream
     * in the first byte transmitted. This enum class is meant to make construction the read/write opcodes trivial
     */
    enum class HardwareDeviceAddress : byte {
        Device0 = 0b0000,
        Device1 = 0b0010,
        Device2 = 0b0100,
        Device3 = 0b0110,
        Device4 = 0b1000,
        Device5 = 0b1010,
        Device6 = 0b1100,
        Device7 = 0b1110,
    };

    enum class PinIndex : byte {
        Pin0,
        Pin1,
        Pin2,
        Pin3,
        Pin4,
        Pin5,
        Pin6,
        Pin7,
        Pin8,
        Pin9,
        Pin10,
        Pin11,
        Pin12,
        Pin13,
        Pin14,
        Pin15,
    };
    constexpr PinIndex validate(uint8_t index) noexcept { return static_cast<PinIndex>(index & 0b1111); }
    constexpr uint16_t getMask(PinIndex value) noexcept { return 1u << static_cast<uint8_t>(value); }
    constexpr byte generateReadOpcode(uint8_t address, bool shift) noexcept {
        return 0b0100'0001 | (shift ? (address << 1) : address);
    }
    constexpr byte generateReadOpcode(HardwareDeviceAddress address) noexcept {
        return generateReadOpcode(static_cast<uint8_t>(address), false);
    }
    constexpr byte generateWriteOpcode(uint8_t address, bool shift) noexcept {
        return 0b0100'0000 | (shift ? (address << 1) : address);
    }
    constexpr byte generateWriteOpcode(HardwareDeviceAddress address) noexcept {
        return generateWriteOpcode(static_cast<uint8_t>(address), false);
    }
    template<HardwareDeviceAddress address, Registers opcode>
    [[gnu::always_inline]]
    inline uint8_t read8(byte pin) noexcept {
        digitalWrite(pin, LOW);
        SPI.transfer(generateReadOpcode(address));
        SPI.transfer(static_cast<uint8_t>(opcode));
        auto result = SPI.transfer(0);
        digitalWrite(pin, HIGH);
        return result;
    }
    template<HardwareDeviceAddress address, Registers opcode, typename Pin>
    [[gnu::always_inline]]
    inline uint8_t read8() noexcept {
        Pin::assertPin();
        SPI.transfer(generateReadOpcode(address));
        SPI.transfer(static_cast<uint8_t>(opcode));
        auto result = SPI.transfer(0);
        Pin::deassertPin();
        return result;
    }
    template<HardwareDeviceAddress address, Registers opcode>
    [[gnu::always_inline]]
    inline SplitWord16 read16(byte pin) noexcept {
        SplitWord16 output(0);
        digitalWrite(pin, LOW);
        SPDR = generateReadOpcode(address);
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait
        SPDR = static_cast<byte>(opcode) ;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait
        SPDR = 0;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait
        auto lower = SPDR;
        SPDR = 0;
        asm volatile("nop");
        {
            output.bytes[0] = lower;
        }
        while (!(SPSR & _BV(SPIF))) ; // wait
        output.bytes[1] = SPDR;
        digitalWrite(pin, HIGH);
        return output;
    }
    template<HardwareDeviceAddress address, Registers opcode, typename Pin>
    [[gnu::always_inline]]
    inline SplitWord16 read16() noexcept {
        SplitWord16 output(0);
        Pin::assertPin();
        SPDR = generateReadOpcode(address);
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait
        SPDR = static_cast<byte>(opcode) ;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait
        SPDR = 0;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait
        auto lower = SPDR;
        SPDR = 0;
        asm volatile("nop");
        {
            output.bytes[0] = lower;
        }
        while (!(SPSR & _BV(SPIF))) ; // wait
        output.bytes[1] = SPDR;
        Pin::deassertPin();
        return output;
    }
    template<HardwareDeviceAddress addr, Registers opcode>
    [[gnu::always_inline]]
    inline void write8(byte pin, byte value) noexcept {
        digitalWrite(pin, LOW);
        SPDR = generateWriteOpcode(addr);
        /*
         * The following NOP introduces a small delay that can prevent the wait
         * loop form iterating when running at the maximum speed. This gives
         * about 10% more speed, even if it seems counter-intuitive. At lower
         * speeds it is unnoticed.
         */
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait
        SPDR = static_cast<byte>(opcode);
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait
        SPDR = value;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait
        digitalWrite(pin, HIGH);
    }
    template<HardwareDeviceAddress addr, Registers opcode, typename Pin>
    [[gnu::always_inline]]
    inline void write8(byte value) noexcept {
        Pin::assertPin();
        SPDR = generateWriteOpcode(addr);
        /*
         * The following NOP introduces a small delay that can prevent the wait
         * loop form iterating when running at the maximum speed. This gives
         * about 10% more speed, even if it seems counter-intuitive. At lower
         * speeds it is unnoticed.
         */
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait
        SPDR = static_cast<byte>(opcode);
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait
        SPDR = value;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait
        Pin::deassertPin();
    }
    template<HardwareDeviceAddress addr, Registers opcode>
    [[gnu::always_inline]]
    inline void write16(byte pin, uint16_t v) noexcept {
        SplitWord16 valueDiv(v);
        digitalWrite(pin, LOW);
        SPDR = generateWriteOpcode(addr);
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait
        SPDR = static_cast<byte>(opcode);
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait
        SPDR = valueDiv.bytes[0];
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait
        SPDR = valueDiv.bytes[1];
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait
        digitalWrite(pin, HIGH);
    }
    template<HardwareDeviceAddress addr, Registers opcode, typename Pin>
    [[gnu::always_inline]]
    inline void write16(uint16_t v) noexcept {
        SplitWord16 valueDiv(v);
        Pin::assertPin();
        SPDR = generateWriteOpcode(addr);
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait
        SPDR = static_cast<byte>(opcode);
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait
        SPDR = valueDiv.bytes[0];
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait
        SPDR = valueDiv.bytes[1];
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait
        Pin::deassertPin();
    }
    /**
     * @brief Read all 16 GPIOs of an io expander
     * @tparam addr The io expander to read from
     * @return The contents of the GPIO register pair
     */
    template<HardwareDeviceAddress addr>
    inline SplitWord16 readGPIO16(byte pin) noexcept {
        return read16<addr, Registers::GPIO>(pin);
    }
    template<HardwareDeviceAddress addr, typename Pin>
    [[gnu::always_inline]]
    inline SplitWord16 readGPIO16() noexcept {
        return read16<addr, Registers::GPIO, Pin>();
    }
    /**
     * @brief Set all 16 GPIOs of an io expander
     * @tparam addr The io expander to write to
     * @tparam standalone When true, wrap the call in a begin/endTransaction call. When false omit them because you are doing many spi operations back to back and the begin/end is handled manually (default true)
     * @param value The value to set the gpios to
     */
     template<HardwareDeviceAddress addr>
    [[gnu::always_inline]]
    inline void writeGPIO16(byte pin, uint16_t value) noexcept {
        write16<addr, Registers::GPIO>(pin, value);
    }
    template<HardwareDeviceAddress addr, typename Pin>
    [[gnu::always_inline]]
    inline void writeGPIO16(uint16_t value) noexcept {
        write16<addr, Registers::GPIO, Pin>(value);
    }
    /**
     * @brief Describe the directions of all 16 pins on a given io expander.
     * @tparam addr The io expander to update
     * @tparam standalone When true, wrap the call in a begin/endTransaction call. When false omit them because you are doing many spi operations back to back and the begin/end is handled manually (default true)
     * @param value The 16-bit direction mask to write to the io expander (a 1 means input, a 0 means output)
     */
    template<HardwareDeviceAddress addr>
    [[gnu::always_inline]]
    inline void writeDirection(byte pin, uint16_t value) noexcept {
        write16<addr, Registers::IODIR>(pin, value) ;
    }
    template<HardwareDeviceAddress addr, typename Pin>
    [[gnu::always_inline]]
    inline void writeDirection(uint16_t value) noexcept {
        write16<addr, Registers::IODIR, Pin>(value) ;
    }
    /**
     * @brief Get all 16 direction bits for the given io expander
     * @param pin The CS pin
     * @param index The biased hardware address of the io expander
     * @return The 16-bits of direction information for the target io expander
     */
    template<HardwareDeviceAddress addr>
    [[gnu::always_inline]]
    inline SplitWord16 readDirection(byte pin) noexcept {
        return read16<addr, Registers::IODIR>(pin);
    }
    template<HardwareDeviceAddress addr, typename Pin>
    [[gnu::always_inline]]
    inline SplitWord16 readDirection() noexcept {
        return read16<addr, Registers::IODIR, Pin>();
    }

    template<HardwareDeviceAddress addr, uint16_t mask, typename Pin>
    inline void pinMode(decltype(INPUT) direction) {
        auto directionBits = readDirection<addr, Pin>().wholeValue_;
        uint16_t pullupBits = read16<addr, Registers::GPPU, Pin>().wholeValue_;
        switch (direction) {
            case OUTPUT:
                directionBits &= ~mask;
                pullupBits &= ~mask;
                break;
            case INPUT_PULLUP:
                // we need to activate the pullup
                directionBits |= mask;
                pullupBits |= mask;
                break;
            case INPUT:
                pullupBits &= ~mask;
                directionBits |= mask;
                break;
            default:
                return;
        }
        writeDirection<addr, Pin>(directionBits);
        write16<addr, Registers::GPPU, Pin>(pullupBits);
    }
    template<HardwareDeviceAddress addr, uint16_t mask, typename Pin>
    inline decltype(HIGH) digitalRead() noexcept {
        return (readGPIO16<addr, Pin>().wholeValue_ & mask) != 0 ? HIGH : LOW;
    }
    template<HardwareDeviceAddress addr, uint16_t mask, typename Pin>
    inline void digitalWrite(decltype(HIGH) value) noexcept {
        if (auto result = readGPIO16<addr, Pin>(); value == LOW) {
            result.wholeValue_ &= ~mask;
            writeGPIO16<addr, Pin>(result.wholeValue_);
        } else {
            result.wholeValue_ |= mask;
            writeGPIO16<addr, Pin>(result.wholeValue_);
        }
    }
    template<HardwareDeviceAddress addr, uint16_t mask, typename Pin, decltype(HIGH) value>
    inline void digitalWrite() noexcept {
        if constexpr (auto result = readGPIO16<addr, Pin>(); value == LOW) {
            result.wholeValue_ &= ~mask;
            writeGPIO16<addr, Pin>(result.wholeValue_);
        } else {
            result.wholeValue_ |= mask;
            writeGPIO16<addr, Pin>(result.wholeValue_);
        }
    }
    template<HardwareDeviceAddress addr, uint16_t mask, typename Pin>
    inline decltype(INPUT) getMode() noexcept {
        auto direction = readDirection<addr, Pin>();
        if ((direction.wholeValue_ & mask) == 0) {
            return OUTPUT;
        } else {
            auto pullups = read16<addr, Registers::GPPU, Pin>();
            if ((pullups.wholeValue_ & mask) == 0) {
                return INPUT;
            } else {
                return INPUT_PULLUP;
            }
        }
    }
    template<auto pin, HardwareDeviceAddress addr, PinIndex index, decltype(INPUT) defaultDirection, typename CS>
    struct BackingPin {
        static constexpr uint16_t ActualMask = getMask(index);
        BackingPin() = delete;
        ~BackingPin() = delete;
        BackingPin(const BackingPin&)  = delete;
        BackingPin& operator=(const BackingPin&)  = delete;
        BackingPin(BackingPin&&)  = delete;
        BackingPin& operator=(BackingPin&&)  = delete;
        static decltype(HIGH) read() noexcept { return digitalRead<addr, ActualMask, CS>(); }
        static void write(decltype(HIGH) value) noexcept { digitalWrite<addr, ActualMask, CS>(value); }
        template<decltype(HIGH) value>
        static void write() noexcept  { digitalWrite<addr, ActualMask, CS, value>(); }
        static void configure(decltype(INPUT) mode = defaultDirection) {
            pinMode<addr, ActualMask, CS>(mode);
        }
        static constexpr bool isInputPin() noexcept {
            auto dir = mode();
            return dir == INPUT || dir == INPUT_PULLUP;
        }
        static constexpr bool isOutputPin() noexcept { return mode() == OUTPUT; }
        static constexpr decltype(INPUT) mode() noexcept { return getMode<addr, ActualMask, CS>(); }
        static constexpr bool valid() noexcept { return false; }
        static constexpr bool getPin() noexcept { return pin; }
        template<decltype(HIGH) to = LOW>
        [[gnu::always_inline]] static void pulse() {
            write<to>();
            if constexpr (to == LOW) {
                write<HIGH>();
            } else {
                write<LOW>();
            }
        }
    };
} // end namespace MCP23S17

#endif //SXCHIPSET_MCP23S17_H
