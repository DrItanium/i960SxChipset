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

#ifndef ARDUINO_IOEXPANDERS_H
#define ARDUINO_IOEXPANDERS_H
#include <Arduino.h>
#include <SPI.h>
#include "Pinout.h"
#include "i960SxChipset.h"

/**
 * @brief Static class which is responsible for managing the interacting between the chipset and the i960 itself
 */
class ProcessorInterface final {
    /**
     * @brief The set of registers exposed by the MCP23S17 in the default bank mode
     */
    enum class MCP23x17Registers : byte {
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
        _IOCONA,
        _IOCONB,
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
        IOCON = _IOCONA,
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
    enum class IOExpanderAddress : byte {
        DataLines = 0b0000,
        Lower16Lines = 0b0010,
        Upper16Lines = 0b0100,
        /**
         * @brief Any extra pins of the i960 that need to be managed external but are generally unimportant are connected to this io expander
         */
        MemoryCommitExtras = 0b0110,
        OtherDevice0 = 0b1000,
        OtherDevice1 = 0b1010,
        OtherDevice2 = 0b1100,
        OtherDevice3 = 0b1110,
    };
    static constexpr byte generateReadOpcode(ProcessorInterface::IOExpanderAddress address) noexcept {
        return 0b0100'0001 | static_cast<uint8_t>(address);
    }
    static constexpr byte generateWriteOpcode(ProcessorInterface::IOExpanderAddress address) noexcept {
        return 0b0100'0000 | static_cast<uint8_t>(address);
    }
    /**
     * @brief Read a 16-bit value from a given io expander register
     * @tparam addr The io expander to read from
     * @tparam opcode The register pair to read from
     * @tparam standalone When true, wrap the call in a begin/endTransaction call. When false omit them because you are doing many spi operations back to back and the begin/end is handled manually (default true)
     * @return The 16-bit value pulled from the io expander
     */
    template<IOExpanderAddress addr, MCP23x17Registers opcode, bool standalone = true>
    static SplitWord16 read16() noexcept {
        if constexpr (standalone) {
            SPI.beginTransaction(SPISettings(TargetBoard::runIOExpanderSPIInterfaceAt(), MSBFIRST, SPI_MODE0));
        }
        SplitWord16 output(0);
        digitalWrite<i960Pinout::GPIOSelect, LOW>();
        SPDR = generateReadOpcode(addr);
        /*
         * The following NOP introduces a small delay that can prevent the wait
         * loop form iterating when running at the maximum speed. This gives
         * about 10% more speed, even if it seems counter-intuitive. At lower
         * speeds it is unnoticed.
         */
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
        digitalWrite<i960Pinout::GPIOSelect, HIGH>();
        if constexpr (standalone) {
            SPI.endTransaction();
        }
        return output;
    }
    /**
     * @brief Read a 8-bit value from a given io expander register
     * @tparam addr The io expander to read from
     * @tparam opcode The register pair to read from
     * @tparam standalone When true, wrap the call in a begin/endTransaction call. When false omit them because you are doing many spi operations back to back and the begin/end is handled manually (default true)
     * @return The contents of an 8-bit register on the io expander
     */
    template<IOExpanderAddress addr, MCP23x17Registers opcode, bool standalone = true>
    static uint8_t read8() noexcept {
        if constexpr (standalone) {
            SPI.beginTransaction(SPISettings(TargetBoard::runIOExpanderSPIInterfaceAt(), MSBFIRST, SPI_MODE0));
        }
        digitalWrite<i960Pinout::GPIOSelect, LOW>();
        SPDR = generateReadOpcode(addr);
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
        SPDR = 0;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait
        digitalWrite<i960Pinout::GPIOSelect, HIGH>();
        if constexpr (standalone) {
            SPI.endTransaction();
        }
        return SPDR;
    }
    /**
     * @brief Write a 16-bit value to a register pair on a target io expander
     * @tparam addr The expander to talk to
     * @tparam opcode The register pair to update
     * @tparam standalone When true, wrap the call in a begin/endTransaction call. When false omit them because you are doing many spi operations back to back and the begin/end is handled manually (default true)
     * @param value The 16-bit value to send to the io expander
     */
    template<IOExpanderAddress addr, MCP23x17Registers opcode, bool standalone = true>
    static void write16(uint16_t value) noexcept {
        SplitWord16 valueDiv(value);
        if constexpr (standalone) {
            SPI.beginTransaction(SPISettings(TargetBoard::runIOExpanderSPIInterfaceAt(), MSBFIRST, SPI_MODE0));
        }
        digitalWrite<i960Pinout::GPIOSelect, LOW>();
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
        SPDR = valueDiv.bytes[0];
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait
        SPDR = valueDiv.bytes[1];
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait
        digitalWrite<i960Pinout::GPIOSelect, HIGH>();
        if constexpr (standalone) {
            SPI.endTransaction();
        }
    }
    /**
     * @brief Write an 8-bit value to a register on a target io expander
     * @tparam addr The expander to talk to
     * @tparam opcode The register to update
     * @tparam standalone When true, wrap the call in a begin/endTransaction call. When false omit them because you are doing many spi operations back to back and the begin/end is handled manually (default true)
     * @param value The 8-bit value to send to the io expander
     */
    template<IOExpanderAddress addr, MCP23x17Registers opcode, bool standalone = true>
    static void write8(uint8_t value) noexcept {
        if constexpr (standalone) {
            SPI.beginTransaction(SPISettings(TargetBoard::runIOExpanderSPIInterfaceAt(), MSBFIRST, SPI_MODE0));
        }
        digitalWrite<i960Pinout::GPIOSelect, LOW>();
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
        digitalWrite<i960Pinout::GPIOSelect, HIGH>();
        if constexpr (standalone) {
            SPI.endTransaction();
        }
    }
    /**
     * @brief Read all 16 GPIOs of an io expander
     * @tparam addr The io expander to read from
     * @tparam standalone When true, wrap the call in a begin/endTransaction call. When false omit them because you are doing many spi operations back to back and the begin/end is handled manually (default true)
     * @return The contents of the GPIO register pair
     */
    template<IOExpanderAddress addr, bool standalone = true>
    static inline SplitWord16 readGPIO16() noexcept {
        return read16<addr, MCP23x17Registers::GPIO, standalone>();
    }
    /**
     * @brief Set all 16 GPIOs of an io expander
     * @tparam addr The io expander to write to
     * @tparam standalone When true, wrap the call in a begin/endTransaction call. When false omit them because you are doing many spi operations back to back and the begin/end is handled manually (default true)
     * @param value The value to set the gpios to
     */
    template<IOExpanderAddress addr, bool standalone = true>
    static inline void writeGPIO16(uint16_t value) noexcept {
        write16<addr, MCP23x17Registers::GPIO, standalone>(value);
    }
    /**
     * @brief Describe the directions of all 16 pins on a given io expander.
     * @tparam addr The io expander to update
     * @tparam standalone When true, wrap the call in a begin/endTransaction call. When false omit them because you are doing many spi operations back to back and the begin/end is handled manually (default true)
     * @param value The 16-bit direction mask to write to the io expander (a 1 means input, a 0 means output)
     */
    template<IOExpanderAddress addr, bool standalone = true>
    static inline void writeDirection(uint16_t value) noexcept {
        write16<addr, MCP23x17Registers::IODIR, standalone>(value);
    }
public:
    ProcessorInterface() = delete;
    ~ProcessorInterface() = delete;
    ProcessorInterface(const ProcessorInterface&) = delete;
    ProcessorInterface(ProcessorInterface&&) = delete;
    ProcessorInterface& operator=(const ProcessorInterface&) = delete;
    ProcessorInterface& operator=(ProcessorInterface&&) = delete;
public:
    /**
     * @brief Setup the processor interface on chipset startup.
     */
    static void begin() noexcept {
        if (!initialized_) {
            initialized_ = true;
            SPI.beginTransaction(SPISettings(TargetBoard::runIOExpanderSPIInterfaceAt(), MSBFIRST, SPI_MODE0));
            pinMode(i960Pinout::GPIOSelect, OUTPUT);
            digitalWrite<i960Pinout::GPIOSelect, HIGH>();
            // at bootup, the IOExpanders all respond to 0b000 because IOCON.HAEN is
            // disabled. We can send out a single IOCON.HAEN enable message and all
            // should receive it.
            // so do a begin operation on all chips (0b000)
            // set IOCON.HAEN on all chips
            write8<ProcessorInterface::IOExpanderAddress::DataLines, MCP23x17Registers::IOCON, false>(initialIOCONValue_);
            write8<ProcessorInterface::IOExpanderAddress::Upper16Lines, MCP23x17Registers::IOCON, false>(initialIOCONValue_);
            write8<ProcessorInterface::IOExpanderAddress::Lower16Lines, MCP23x17Registers::IOCON, false>(initialIOCONValue_);
            write8<ProcessorInterface::IOExpanderAddress::MemoryCommitExtras, MCP23x17Registers::IOCON, false>(initialIOCONValue_);
            // immediately pull the i960 into reset as soon as possible
            writeDirection<IOExpanderAddress::MemoryCommitExtras, false>(currentGPIO4Direction_);
            writeGPIO16<IOExpanderAddress::MemoryCommitExtras, false>(currentGPIO4Status_);
            // now all devices tied to this ~CS pin have separate addresses
            // make each of these inputs
            writeDirection<IOExpanderAddress::Lower16Lines, false>(0xFFFF);
            writeDirection<IOExpanderAddress::Upper16Lines, false>(0xFFFF);
            // enable pin change interrupts on address lines
            write16<IOExpanderAddress::Lower16Lines, MCP23x17Registers::GPINTEN, false>(0xFFFF) ;
            write16<IOExpanderAddress::Upper16Lines, MCP23x17Registers::GPINTEN, false>(0xFFFF) ;
            write16<IOExpanderAddress::Lower16Lines, MCP23x17Registers::INTCON, false>(0x0000) ;
            write16<IOExpanderAddress::Upper16Lines, MCP23x17Registers::INTCON, false>(0x0000) ;
            // setup the direction pins in the
            writeDirection<IOExpanderAddress::DataLines, false>(dataLinesDirection_ == 0xFF ? 0xFFFF : 0x0000);
            // write the default value out to the latch to start with
            write16<IOExpanderAddress::DataLines, MCP23x17Registers::OLAT, false>(latchedDataOutput.getWholeValue());
            SPI.endTransaction();
        }
    }
    /**
     * @brief Get the address for the current transaction
     * @return The full 32-bit address for the current transaction
     */
    [[nodiscard]] static constexpr Address getAddress() noexcept { return address_.getWholeValue(); }
    /**
     * @brief Update the contents of the GPIO register pair on the data lines io expander (the i960 wants to read from memory). Also check to
     * see if this is the last word transmitted in the given transaction.
     * @param value The value to set the GPIO register pair to
     * @return True if this is the last word of a burst transaction
     */
    static bool setDataBits(uint16_t value) noexcept {
        // the latch is preserved in between data line changes
        // okay we are still pointing as output values
        // check the latch and see if the output value is the same as what is latched
        if (latchedDataOutput.getWholeValue() != value) {
            //latchedDataOutput.wholeValue_ = value;
            bool isLastRead;
            byte computedValue = 0;
            digitalWrite<i960Pinout::GPIOSelect, LOW>();
            SPDR = generateWriteOpcode(IOExpanderAddress::DataLines);
            {
                // this operation is very important to interleave because it can be very expensive to
                // but if we are also communicating over SPI, then the cost of this operation is nullified considerably
                latchedDataOutput.wholeValue_ = value;
            }
            while (!(SPSR & _BV(SPIF))) ; // wait
            SPDR = static_cast<byte>(MCP23x17Registers::GPIO);
            {
                // find out if this is the last transaction while we are talking to the io expander
                isLastRead = DigitalPin<i960Pinout::BLAST_>::isAsserted();
                computedValue = latchedDataOutput.bytes[0];
            }
            while (!(SPSR & _BV(SPIF))) ; // wait
            SPDR = computedValue;
            {
                computedValue = latchedDataOutput.bytes[1];
            }
            while (!(SPSR & _BV(SPIF))) ; // wait
            SPDR = computedValue;
            {
                asm volatile("nop");
                /// @todo insert tiny independent operations here if desired, delete nop if code added here
            }
            while (!(SPSR & _BV(SPIF))) ; // wait
            digitalWrite<i960Pinout::GPIOSelect, HIGH>();
            return isLastRead;
        } else {
            return DigitalPin<i960Pinout::BLAST_>::isAsserted();

        }
    }
    /**
     * @brief Query the ~BE0 and ~BE1 pins provided by the i960 to denote how the chipset should treat the current word in the transaction
     * @return The LoadStoreStyle derived from the ~BE0 and ~BE1 pins.
     */
    [[nodiscard]] static auto getStyle() noexcept {
        return static_cast<LoadStoreStyle>(((PINA >> 4) & 0b11));
    }
    /**
     * @brief Check the W/~R pin to see if we are dealing with a read transaction.
     * Only needs to be queried once at the beginning of a new transaction
     * @return If true, then the current transaction is a read operation. If false, then the current transaction is a write operation.
     */
    [[nodiscard]] static bool isReadOperation() noexcept { return DigitalPin<i960Pinout::W_R_>::isAsserted(); }
    /**
     * @brief Retrieve the computed cache offset entry start. This is the word index that the current transaction will start at within a
     * target cache line.
     * @return The precomputed cache offset for the current transaction
     */
    [[nodiscard]] static auto getCacheOffsetEntry() noexcept { return cacheOffsetEntry_; }
private:
    /**
     * @brief A method responsible for turning data lines from input to output and vice-versa. It also inverts the tracking byte used by this
     * class to only update the direction when needed (SPI bus transactions are relatively expensive).
     */
    inline static void invertDataLinesDirection() noexcept {
        SPI.beginTransaction(SPISettings(TargetBoard::runIOExpanderSPIInterfaceAt(), MSBFIRST, SPI_MODE0));
        digitalWrite<i960Pinout::GPIOSelect, LOW>();
        SPDR = generateWriteOpcode(IOExpanderAddress::DataLines);
        {
            dataLinesDirection_ = ~dataLinesDirection_;
        }
        while (!(SPSR & _BV(SPIF))); // wait
        SPDR = static_cast<byte>(MCP23x17Registers::IODIR);
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        SPDR = dataLinesDirection_;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        SPDR = dataLinesDirection_;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait
        digitalWrite<i960Pinout::GPIOSelect, HIGH>();
        SPI.endTransaction();
    }
public:
    inline static void setupDataLinesForWrite() noexcept {
        if (!dataLinesDirection_) {
            invertDataLinesDirection();
        }
    }
    inline static void setupDataLinesForRead() noexcept {
        if (dataLinesDirection_) {
            invertDataLinesDirection();
        }
    }
private:
    /**
     * @brief Query the MCP23S17 interrupt pins to figure out which ports on the address lines had actually changed since the last transaction
     * @tparam useInterrupts When true, query the interrupt lines to generate a difference mask. When false, 0 is returned which means all have changed
     * @return A four bit code where each bit corresponds to a block of 8-bits and if they have changed since the last transaction or not.
     * Each zero found in the code signifies that that 8-bit quantity must be updated. If useInterrupts is off then zero is returned which
     * means the whole address must be updated.
     */
    template<bool useInterrupts = true>
    static byte getUpdateKind() noexcept {
        if constexpr (!useInterrupts) {
            return 0;
        } else {
            if constexpr (TargetBoard::onAtmega1284p_Type1()) {
                // even though three of the four pins are actually in use, I want to eventually diagnose the problem itself
                // so this code is ready for that day
                return PINA & 0b0000'1111;
            } else {
                return 0;
            }
        }
    }
    /**
     * @brief Pull an entire 32-bit address from the upper and lower address io expanders. Updates the function to execute to satisfy the request
     * @tparam C The cache line type used by the L1 cache
     * @tparam inDebugMode When true, any extra debugging code becomes active. Will be propagated to any child methods which take in the parameter
     */
    template<typename C, bool inDebugMode>
    inline static void full32BitUpdate() noexcept {
        static constexpr auto OffsetMask = C::CacheEntryMask;
        static constexpr auto OffsetShiftAmount = C::CacheEntryShiftAmount;
        constexpr auto Lower16Opcode = generateReadOpcode(ProcessorInterface::IOExpanderAddress::Lower16Lines);
        constexpr auto Upper16Opcode = generateReadOpcode(ProcessorInterface::IOExpanderAddress::Upper16Lines);
        constexpr auto GPIOOpcode = static_cast<byte>(MCP23x17Registers::GPIO);
        // we want to overlay actions as much as possible during spi transfers, there are blocks of waiting for a transfer to take place
        // where we can insert operations to take place that would otherwise be waiting
        digitalWrite<i960Pinout::GPIOSelect, LOW>();
        SPDR = Upper16Opcode;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        SPDR = GPIOOpcode;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        SPDR = 0;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        auto higher = SPDR;
        SPDR = 0;
        {
            address_.bytes[2] = higher;
        }
        while (!(SPSR & _BV(SPIF))); // wait
        auto highest = SPDR;
        DigitalPin<i960Pinout::GPIOSelect>::pulse<HIGH>(); // pulse high
        SPDR = Lower16Opcode;
        {
            // don't do the comparison, instead just force the update every single time
            // there should be enough time in between transactions to make sure
            if constexpr (inDebugMode) {
                lastReadDebug_ = getReadBody<true>(highest);
            }
            lastRead_ = getReadBody<false>(highest);
        }
        while (!(SPSR & _BV(SPIF))); // wait
        SPDR = GPIOOpcode;
        {
            if constexpr (inDebugMode) {
                lastWriteDebug_ = getWriteBody<true>(highest);
            }
            lastWrite_ = getWriteBody<false>(highest);
        }
        while (!(SPSR & _BV(SPIF))); // wait
        SPDR = 0;
        {
            address_.bytes[3] = highest;
        }
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        auto lowest = SPDR;
        SPDR = 0;
        {
            // inside of here we have access to 12 cycles to play with, so let's actually do some operations while we wait
            // put scope ticks to force the matter
            cacheOffsetEntry_ = (lowest >> OffsetShiftAmount) & OffsetMask; // we want to make this quick to increment
            address_.bytes[0] = lowest;
        }
        while (!(SPSR & _BV(SPIF))); // wait
        address_.bytes[1] = SPDR;
        digitalWrite<i960Pinout::GPIOSelect, HIGH>();
    }
    /**
     * @brief Only update the lower 16 bits of the current transaction's base address
     * @tparam C The cache line type used by the L1 cache
     */
    template<typename C>
    static void lower16Update() noexcept {
        static constexpr auto OffsetMask = C::CacheEntryMask;
        static constexpr auto OffsetShiftAmount = C::CacheEntryShiftAmount;
        constexpr auto Lower16Opcode = generateReadOpcode(ProcessorInterface::IOExpanderAddress::Lower16Lines);
        constexpr auto GPIOOpcode = static_cast<byte>(MCP23x17Registers::GPIO);
        // read only the lower half
        // we want to overlay actions as much as possible during spi transfers, there are blocks of waiting for a transfer to take place
        // where we can insert operations to take place that would otherwise be waiting
        digitalWrite<i960Pinout::GPIOSelect, LOW>();
        SPDR = Lower16Opcode;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        SPDR = GPIOOpcode;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        SPDR = 0;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        auto lowest = SPDR;
        SPDR = 0;
        asm volatile("nop");
        {
            // inside of here we have access to 12 cycles to play with, so let's actually do some operations while we wait
            // put scope ticks to force the matter
            cacheOffsetEntry_ = (lowest >> OffsetShiftAmount) & OffsetMask; // we want to make this quick to increment
            address_.bytes[0] = lowest;
        }
        while (!(SPSR & _BV(SPIF))); // wait
        address_.bytes[1] = SPDR;
        digitalWrite<i960Pinout::GPIOSelect, HIGH>();
    }
    /**
     * @brief Update the upper 16-bits of the current transaction's address. Update the function to invoke to satisfy this request
     * @tparam inDebugMode If true then enable debugging output and pass that to any child methods which accept the parameter as well
     */
    template<bool inDebugMode>
    static void upper16Update() noexcept {
        constexpr auto Upper16Opcode = generateReadOpcode(ProcessorInterface::IOExpanderAddress::Upper16Lines);
        constexpr auto GPIOOpcode = static_cast<byte>(MCP23x17Registers::GPIO);
        // only read the upper 16-bits
        digitalWrite<i960Pinout::GPIOSelect, LOW>();
        SPDR = Upper16Opcode;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        SPDR = GPIOOpcode;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        SPDR = 0;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        auto higher = SPDR;
        SPDR = 0;
        asm volatile("nop");
        {
            address_.bytes[2] = higher;
        }
        while (!(SPSR & _BV(SPIF))); // wait
        auto highest = SPDR;
        digitalWrite<i960Pinout::GPIOSelect, HIGH>();
        if (address_.bytes[3] != highest) {
            updateTargetFunctions<inDebugMode>(highest);
        }
        address_.bytes[3] = highest;
    }
    /**
     * @brief Update the upper most 8 bits of the current transaction address. Also update the function to execute to satisfy this request
     * @tparam inDebugMode If true then display debugging information and pass it to child methods that accept it
     */
    template<bool inDebugMode>
    static void updateHighest8() noexcept {
        constexpr auto Upper16Opcode = generateReadOpcode(ProcessorInterface::IOExpanderAddress::Upper16Lines);
        constexpr auto GPIOOpcode = static_cast<byte>(MCP23x17Registers::GPIOB);
        // only read the upper 8 bits
        digitalWrite<i960Pinout::GPIOSelect, HIGH>();
        SPDR = Upper16Opcode;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        SPDR = GPIOOpcode;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        SPDR = 0;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        auto highest = SPDR;
        digitalWrite<i960Pinout::GPIOSelect, LOW>();
        if (highest != address_.bytes[3]) {
            updateTargetFunctions<inDebugMode>(highest);
        }
        address_.bytes[3] = highest;
    }
    /**
     * @brief Update address bits 16-23
     */
    static void updateHigher8() noexcept {
        constexpr auto Upper16Opcode = generateReadOpcode(ProcessorInterface::IOExpanderAddress::Upper16Lines);
        constexpr auto GPIOOpcode = static_cast<byte>(MCP23x17Registers::GPIOA);
        // only read the upper 8 bits
        digitalWrite<i960Pinout::GPIOSelect, HIGH>();
        SPDR = Upper16Opcode;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        SPDR = GPIOOpcode;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        SPDR = 0;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        auto highest = SPDR;
        digitalWrite<i960Pinout::GPIOSelect, LOW>();
        address_.bytes[2] = highest;
    }
    /**
     * @brief Update address bits 0-7 and compute the base cache offset value
     * @tparam C The cache line type used by the L1 cache
     */
    template<typename C>
    static void updateLowest8() noexcept {
        static constexpr auto OffsetMask = C::CacheEntryMask;
        static constexpr auto OffsetShiftAmount = C::CacheEntryShiftAmount;
        constexpr auto Lower16Opcode = generateReadOpcode(ProcessorInterface::IOExpanderAddress::Lower16Lines);
        constexpr auto GPIOOpcode = static_cast<byte>(MCP23x17Registers::GPIOA);
        // read only the lower half
        // we want to overlay actions as much as possible during spi transfers, there are blocks of waiting for a transfer to take place
        // where we can insert operations to take place that would otherwise be waiting
        digitalWrite<i960Pinout::GPIOSelect, LOW>();
        SPDR = Lower16Opcode;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        SPDR = GPIOOpcode;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        SPDR = 0;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        auto lowest = SPDR;
        digitalWrite<i960Pinout::GPIOSelect, HIGH>();
        // inside of here we have access to 12 cycles to play with, so let's actually do some operations while we wait
        // put scope ticks to force the matter
        cacheOffsetEntry_ = (lowest >> OffsetShiftAmount) & OffsetMask; // we want to make this quick to increment
        address_.bytes[0] = lowest;
    }
    /**
     * @brief Update address bits 8-15
     */
    static void updateLower8() noexcept {
        constexpr auto Lower16Opcode = generateReadOpcode(ProcessorInterface::IOExpanderAddress::Lower16Lines);
        constexpr auto GPIOOpcode = static_cast<byte>(MCP23x17Registers::GPIOB);
        // read only the lower half
        // we want to overlay actions as much as possible during spi transfers, there are blocks of waiting for a transfer to take place
        // where we can insert operations to take place that would otherwise be waiting
        digitalWrite<i960Pinout::GPIOSelect, LOW>();
        SPDR = Lower16Opcode;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        SPDR = GPIOOpcode;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        SPDR = 0;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        address_.bytes[1] = SPDR;
        digitalWrite<i960Pinout::GPIOSelect, HIGH>();
    }
private:
    static void updateGPIO4Pins(uint16_t value) noexcept {
        if (value != currentGPIO4Status_) {
            // only update the io expander when needed
            currentGPIO4Status_ = value;
            writeGPIO16<IOExpanderAddress::MemoryCommitExtras>(currentGPIO4Status_);
        }
    }
public:
    static void putCPUIntoReset() noexcept {
        updateGPIO4Pins(currentGPIO4Status_ & 0xFFFE);
    }
    static void pullCPUOutOfReset() noexcept {
        updateGPIO4Pins(currentGPIO4Status_ | 1);
    }
    static void triggerInt0() noexcept {
        updateGPIO4Pins(currentGPIO4Status_ & 0xFFFD);
        updateGPIO4Pins(currentGPIO4Status_ | 0b10);
    }
    static void triggerInt1() noexcept {
        updateGPIO4Pins(currentGPIO4Status_ | 0b100);
        updateGPIO4Pins(currentGPIO4Status_ & 0xFFFB);
    }
    static void triggerInt2() noexcept {
        updateGPIO4Pins(currentGPIO4Status_ | 0b1000);
        updateGPIO4Pins(currentGPIO4Status_ & 0xFFF7);
    }
    static void triggerInt3() noexcept {
        updateGPIO4Pins(currentGPIO4Status_ & 0xFFEF);
        updateGPIO4Pins(currentGPIO4Status_ | 0b1'0000);
    }
    /// @todo implement HOLD and LOCK functionality
private:
    /**
     * @brief save the read and write function pointers for a given index
     * @tparam inDebugMode If true then also update the debug versions of the function pointers in addition to the non debug ones
     */
    template<bool inDebugMode>
    inline static void updateTargetFunctions(byte index) noexcept {
        if constexpr (inDebugMode) {
            lastReadDebug_ = getReadBody<true>(index);
            lastWriteDebug_ = getWriteBody<true>(index) ;
        }
        lastRead_ = getReadBody<false>(index);
        lastWrite_ = getWriteBody<false>(index) ;
    }
public:
    /**
     * @brief Starts a new memory transaction. It is responsible for updating the target base address and then invoke the proper read/write function to satisfy the request
     * @tparam inDebugMode When true, use debug versions of the read/write function pointers to fulfill the request. This is passed to direct children as well.
     * @tparam C The cache line type useful for computing the offset mask and other components
     * @tparam offsetMask The cache address offset mask. It is used when computing the starting cache offset entry. It must be provided by the
     * cache entry class that the L1 cache uses
     * @tparam useInterrupts If true, then query the directly connected interrupt pins to get a proper update mask
     */
    template<bool inDebugMode, typename C, bool useInterrupts = true, bool useLookupTable = false>
    static void newDataCycle() noexcept {

        if constexpr (useLookupTable) {
            using Operation = void (*)();
            constexpr Operation operations[16]{
                    full32BitUpdate<C, inDebugMode>,
                    []() {
                        updateLower8();
                        upper16Update<inDebugMode>();
                    },
                    []() {
                        updateLowest8<C>();
                        upper16Update<inDebugMode>();
                    },
                    upper16Update<inDebugMode>,
                    []() {
                        lower16Update<C>();
                        updateHighest8<inDebugMode>();
                    },
                    []() {
                        updateLower8();
                        updateHighest8<inDebugMode>();
                    },
                    []() {
                        updateLowest8<C>();
                        updateHighest8<inDebugMode>();
                    },
                    updateHighest8<inDebugMode>,
                    []() {
                        lower16Update<C>();
                        updateHigher8();
                    },
                    []() {
                        updateHigher8();
                        updateLower8();
                    },
                    []() {
                        updateHigher8();
                        updateLowest8<C>();
                    },
                    updateHigher8,
                    lower16Update<C>,
                    updateLower8,
                    updateLowest8<C>,
                    []() {},
            };
            operations[getUpdateKind<useInterrupts>()]();
        } else {
            switch (getUpdateKind<useInterrupts>()) {
                case 0b0001:
                    updateLower8();
                    upper16Update<inDebugMode>();
                    break;
                case 0b0010:
                    updateLowest8<C>();
                    upper16Update<inDebugMode>();
                    break;
                case 0b0011:
                    upper16Update<inDebugMode>();
                    break;
                case 0b0100:
                    lower16Update<C>();
                    updateHighest8<inDebugMode>();
                    break;
                case 0b0101:
                    updateLower8();
                    updateHighest8<inDebugMode>();
                    break;
                case 0b0110:
                    updateLowest8<C>();
                    updateHighest8<inDebugMode>();
                    break;
                case 0b0111:
                    updateHighest8<inDebugMode>();
                    break;
                case 0b1000:
                    lower16Update<C>();
                    updateHigher8();
                    break;
                case 0b1001:
                    updateHigher8();
                    updateLower8();
                    break;
                case 0b1010:
                    updateHigher8();
                    updateLowest8<C>();
                    break;
                case 0b1011:
                    updateHigher8();
                    break;
                case 0b1100:
                    lower16Update<C>();
                    break;
                case 0b1101:
                    updateLower8();
                    break;
                case 0b1110:
                    updateLowest8<C>();
                    break;
                case 0b1111: break;
                default:
                    full32BitUpdate<C, inDebugMode>();
                    break;
            }
        }
        if (isReadOperation()) {
            setupDataLinesForRead();
            if constexpr (inDebugMode) {
                lastReadDebug_();
            } else {
                lastRead_();
            }
        } else {
            setupDataLinesForWrite();
            if constexpr (inDebugMode) {
                lastWriteDebug_();
            } else {
                lastWrite_();
            }
        }
    }
    /**
     * @brief Return the least significant byte of the address, useful for CoreChipsetFeatures
     * @return The LSB of the address
     */
    [[nodiscard]] static auto getPageOffset() noexcept { return address_.bytes[0]; }
    [[nodiscard]] static auto getPageIndex() noexcept { return address_.bytes[1]; }

    /**
     * @brief loads a cache line based on base transaction address and then bursts up to 16 bytes to the i960
     * @tparam CacheLine The type of a single cache line
     * @tparam inDebugMode Are we in debug mode?
     * @param line The cache line which we will be using for this transaction
     */
    template<typename CacheLine, bool inDebugMode>
    static inline void performCacheRead(const CacheLine& line) noexcept {
        SPI.beginTransaction(SPISettings(TargetBoard::runIOExpanderSPIInterfaceAt(), MSBFIRST, SPI_MODE0));
        // read 32-bits at a time instead of 16 just to keep the throughput increased
        for (auto offset = getCacheOffsetEntry(); ;offset += 8) {
            // this is more unsafe than waiting later on in the process as we are guessing that we need this data
            // These full words will hold onto garbage data if we span two cache lines. That is actually okay because
            // the i960 will never request that data. It will operate on a maximum of 16-bytes at a time.
            // For consistency, I am leaving the loop in (my research may be flawed in some way)
            auto a0 = line.get(offset+0);
            auto a1 = line.get(offset+1);
            auto a2 = line.get(offset+2);
            auto a3 = line.get(offset+3);
            auto a4 = line.get(offset+4);
            auto a5 = line.get(offset+5);
            auto a6 = line.get(offset+6);
            auto a7 = line.get(offset+7);
            auto isLastRead = setDataBits(a0);
            DigitalPin<i960Pinout::Ready>::pulse();
            if (isLastRead) {
                break;
            }
            isLastRead = setDataBits(a1);
            DigitalPin<i960Pinout::Ready>::pulse();
            if (isLastRead) {
                break;
            }
            // we are looking at a double word, triple word, or quad word
            isLastRead = setDataBits(a2);
            DigitalPin<i960Pinout::Ready>::pulse();
            if (isLastRead) {
                break;
            }
            isLastRead = setDataBits(a3);
            DigitalPin<i960Pinout::Ready>::pulse();
            if (isLastRead) {
                break;
            }
            // okay so we are looking at a triple word or quad word operation
            isLastRead = setDataBits(a4);
            DigitalPin<i960Pinout::Ready>::pulse();
            if (isLastRead) {
                break;
            }
            isLastRead = setDataBits(a5);
            DigitalPin<i960Pinout::Ready>::pulse();
            if (isLastRead) {
                break;
            }
            // okay so we are looking at a quad word operation of some kind
            // perhaps the processor loading instruction into the data cache?
            isLastRead = setDataBits(a6);
            DigitalPin<i960Pinout::Ready>::pulse();
            if (isLastRead) {
                break;
            }
            isLastRead = setDataBits(a7);
            DigitalPin<i960Pinout::Ready>::pulse();
            if (isLastRead) {
                break;
            }
        }
        SPI.endTransaction();
    }
private:
    struct CacheWriteRequest {
        LoadStoreStyle style;
        SplitWord16 value;
        byte offset;
        template<typename CacheLine>
        inline void set(CacheLine& line) noexcept {
            line.set(offset, style, value);
        }
    };
    static inline CacheWriteRequest transactions[8];
    template<byte count>
    [[nodiscard]]
    [[gnu::always_inline]]
    static inline bool getDataBits(byte offset) noexcept {
        // getDataBits will be expanded here
        digitalWrite<i960Pinout::GPIOSelect, LOW>();
        SPDR = generateReadOpcode(IOExpanderAddress::DataLines);
        auto isLast = DigitalPin<i960Pinout::BLAST_>::isAsserted();
        auto& request = transactions[count];
        while (!(SPSR & _BV(SPIF))); // wait
        SPDR = static_cast<byte>(MCP23x17Registers::GPIO);
        {
            request.style = getStyle();
        }
        while (!(SPSR & _BV(SPIF))); // wait
        SPDR = 0;
        {
            request.offset = offset + count;
        }
        while (!(SPSR & _BV(SPIF))); // wait
        auto lower = SPDR;
        SPDR = 0;
        {
            request.value.bytes[0] = lower;
        }
        while (!(SPSR & _BV(SPIF))); // wait
        request.value.bytes[1] = SPDR;
        digitalWrite<i960Pinout::GPIOSelect, HIGH>();
        return isLast;
    }
    template<byte count, typename C>
    static inline void commitTransactions(C& line) noexcept {
        static_assert(count <= 8, "Can only transmit 8 transactions at a time");
        for (byte i = 0; i < count; ++i) {
            transactions[i].set(line);
        }
    }
public:
    /**
     * @brief commit up to 16-bytes of data from the i960 to a specified cache line
     * @tparam CacheLine The type of the cache line to write to
     * @tparam inDebugMode are we in debug mode?
     * @param line The cache line to write to.
     */
    template<typename CacheLine, bool inDebugMode>
    static inline void performCacheWrite(CacheLine& line) noexcept {
        SPI.beginTransaction(SPISettings(TargetBoard::runIOExpanderSPIInterfaceAt(), MSBFIRST, SPI_MODE0));
        for (auto offset = getCacheOffsetEntry();;offset += 8) {
            auto isLast = getDataBits<0>(offset);
            DigitalPin<i960Pinout::Ready>::pulse();
            if (isLast) {
                commitTransactions<1, CacheLine>(line);
                break;
            }
            isLast = getDataBits<1>(offset);
            DigitalPin<i960Pinout::Ready>::pulse();
            if (isLast) {
                commitTransactions<2, CacheLine>(line);
                break;
            }
            isLast = getDataBits<2>(offset);
            DigitalPin<i960Pinout::Ready>::pulse();
            if (isLast) {
                commitTransactions<3, CacheLine>(line);
                break;
            }
            isLast = getDataBits<3>(offset);
            DigitalPin<i960Pinout::Ready>::pulse();
            if (isLast) {
                commitTransactions<4, CacheLine>(line);
                break;
            }
            isLast = getDataBits<4>(offset);
            DigitalPin<i960Pinout::Ready>::pulse();
            if (isLast) {
                commitTransactions<5, CacheLine>(line);
                break;
            }
            isLast = getDataBits<5>(offset);
            DigitalPin<i960Pinout::Ready>::pulse();
            if (isLast) {
                commitTransactions<6, CacheLine>(line);
                break;
            }
            isLast = getDataBits<6>(offset);
            DigitalPin<i960Pinout::Ready>::pulse();
            if (isLast) {
                commitTransactions<7, CacheLine>(line);
                break;
            }
            isLast = getDataBits<7>(offset);
            DigitalPin<i960Pinout::Ready>::pulse();
            // perform the commit at this point
            commitTransactions<8, CacheLine>(line);
            if (isLast) {
                break;
            }
        }
        SPI.endTransaction();
    }
    /**
     * @brief The current transaction is reading from a specific memory mapped device connected to the chipset.
     * @tparam T The specific device to be read from
     * @tparam inDebugMode are we in debug mode?
     */
    template<typename T, bool inDebugMode>
    static inline void performExternalDeviceRead() noexcept {
        // this is a subset of actions, we just need to read the byte enable bits continuously and advance the address by two to get to the
        // next 16-bit word
        // don't increment everything just the lowest byte since we will never actually span 16 byte segments in a single burst transaction
        for (byte pageOffset = getPageOffset(); ;pageOffset += 2) {
            auto result = T::read(getPageIndex(),
                                  pageOffset,
                                  getStyle());
            if constexpr (inDebugMode) {
                Serial.print(F("\tPage Index: 0x")) ;
                Serial.println(getPageIndex(), HEX);
                Serial.print(F("\tPage Offset: 0x")) ;
                Serial.println(pageOffset, HEX);
                Serial.print(F("\tRead Value: 0x"));
                Serial.println(result, HEX);
            }
            auto isLast = setDataBits(result);
            DigitalPin<i960Pinout::Ready>::pulse();
            if (isLast) {
                return;
            }
        }
    }
    /**
     * @brief The current transaction is writing to a specific memory mapped device connected to the chipset.
     * @tparam T The specific device to write to
     * @tparam inDebugMode are we in debug mode?
     */
    template<typename T, bool inDebugMode>
    static inline void performExternalDeviceWrite() noexcept {
        // be careful of querying i960 state at this point because the chipset runs at twice the frequency of the i960
        // so you may still be reading the previous i960 cycle state!
        for (byte pageOffset = getPageOffset(); ; pageOffset += 2) {
            bool isLast;
            LoadStoreStyle currLSS;
            SplitWord16 dataBits;
            byte pageIndex;
            // getDataBits will be expanded here
            digitalWrite<i960Pinout::GPIOSelect, LOW>();
            SPDR = generateReadOpcode(IOExpanderAddress::DataLines);
            {
                isLast = DigitalPin<i960Pinout::BLAST_>::isAsserted();
            }
            while (!(SPSR & _BV(SPIF))) ; // wait
            SPDR = static_cast<byte>(MCP23x17Registers::GPIO) ;
            {
                currLSS = getStyle();
            }
            while (!(SPSR & _BV(SPIF))) ; // wait
            SPDR = 0;
            {
                pageIndex = getPageIndex();
            }
            while (!(SPSR & _BV(SPIF))) ; // wait
            auto lower = SPDR;
            SPDR = 0;
            {
                dataBits.bytes[0] = lower;
            }
            while (!(SPSR & _BV(SPIF))) ; // wait
            dataBits.bytes[1] = SPDR;
            digitalWrite<i960Pinout::GPIOSelect, HIGH>();
            if constexpr (inDebugMode) {
                Serial.print(F("\tPage Index: 0x"));
                Serial.println(pageIndex, HEX);
                Serial.print(F("\tPage Offset: 0x"));
                Serial.println(pageOffset, HEX);
                Serial.print(F("\tData To Write: 0x"));
                Serial.println(dataBits.getWholeValue(), HEX);
            }
            T::write(pageIndex,
                     pageOffset,
                     currLSS,
                     dataBits);
            // we could actually pulse the cpu and then perform the write, unsure at this point
            DigitalPin<i960Pinout::Ready>::pulse();
            if (isLast) {
                break;
            }
        }
    }
    /**
     * @brief Used when the transaction is reading from unmapped memory in the i960's memory space. Zero will be sent to the i960 for the duration of the transaction
     * @tparam inDebugMode are we in debug mode?
     */
    static inline void performFallbackRead() noexcept {
        do {
            auto isLast = setDataBits(0);
            DigitalPin<i960Pinout::Ready>::pulse();
            // need to introduce some delay
            if (isLast) {
                break;
            }
        } while (true);
    }
    /**
     * @brief Used when the transaction is writing to unmapped memory in the i960's memory space. Nothing will be written but an artificial delay will be introduced to be on the safe side.
     * @tparam inDebugMode are we in debug mode?
     */
    static inline void performFallbackWrite() noexcept {
        do {
            // put four cycles worth of delay into this to make damn sure we are ready with the i960
            auto isLast = DigitalPin<i960Pinout::BLAST_>::isAsserted();
            __builtin_avr_nops(4);
            DigitalPin<i960Pinout::Ready>::pulse();
            // need to introduce some delay
            if (isLast) {
                break;
            }
        } while (true);
    }
public:
    /**
     * @brief Complete the process of setting up the processor interface by seeding the cached function pointers with valid addresses.
     */
    static inline void setupMostRecentDispatchFunctions() noexcept {
        if (!initialDispatchFunctionsInitialized_) {
            initialDispatchFunctionsInitialized_ = true;
            // update all of the target functions at the same time on initial startup
            updateTargetFunctions<true>(0);
        }
    }
private:
    static inline SplitWord32 address_{0};
    static inline SplitWord16 latchedDataOutput {0};
    static inline byte dataLinesDirection_ = 0xFF;
    static inline byte cacheOffsetEntry_ = 0;
    static inline bool initialized_ = false;
    static inline bool initialDispatchFunctionsInitialized_ = false;
    static inline BodyFunction lastRead_ = nullptr;
    static inline BodyFunction lastReadDebug_ = nullptr;
    static inline BodyFunction lastWrite_ = nullptr;
    static inline BodyFunction lastWriteDebug_ = nullptr;
    static inline uint16_t currentGPIO4Status_ = 0b00000000'10010010;
    static inline uint16_t currentGPIO4Direction_ = 0b00000000'00100000;
    static constexpr uint8_t initialIOCONValue_ = 0b0000'1000;
};
// 8 IOExpanders to a single enable line for SPI purposes
// 4 of them are reserved

#endif //ARDUINO_IOEXPANDERS_H
