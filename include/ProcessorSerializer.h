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
#include "ConfigurationFlags.h"
#include "CacheDescription.h"
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
    static consteval byte generateReadOpcode(ProcessorInterface::IOExpanderAddress address) noexcept {
        return 0b0100'0001 | static_cast<uint8_t>(address);
    }
    static consteval byte generateWriteOpcode(ProcessorInterface::IOExpanderAddress address) noexcept {
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
     * @brief On board the type 1.01 boards, there is an AHC138 which divides the memory space into eight 512 megabyte sections;
     *        The lowest 512 megabytes is known as the ram area. This function checks to see if the ram space gpio is asserted
     * @return True if the ram space gpio is asserted.
     */
    [[nodiscard]] static inline auto inRAMSpace() noexcept { return DigitalPin<i960Pinout::RAM_SPACE_>::isAsserted(); }
    /**
     * @brief On board the type 1.01 boards, there is an AHC138 which divides the memory space into eight 512 megabyte sections;
     *        The highest 512 megabytes is known as the io area. This function checks to see if the io space gpio is asserted
     * @return True if the io space gpio is asserted.
     */
    [[nodiscard]] static inline auto inIOSpace() noexcept { return DigitalPin<i960Pinout::IO_SPACE_>::isAsserted(); }
    /**
     * @brief On board the type 1.01 boards, there is an AHC138 which divides the memory space into eight 512 megabyte sections;
     *        If the address is not in the lowest or highest sections then it is considered unmapped
     * @return True if the address is currently in the unmapped space location (neither ram or io spaces asserted)
     */
    [[nodiscard]] static inline auto inUnmappedSpace() noexcept { return !inRAMSpace() && !inIOSpace(); }
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
            // mirror the interrupts for the upper 16-bits, for some reason, the upper most 8-bits are never marked as changed
            // mirror the interrupts on the lower 16-bits to free up two pins at the cost of update accuracy
            // we want two separate pins free for the data lines io expander
            // we want mirroring on the data lines interrupts
            write8<ProcessorInterface::IOExpanderAddress::DataLines, MCP23x17Registers::IOCON, false>(initialIOCONValue_);
            write8<ProcessorInterface::IOExpanderAddress::Upper16Lines, MCP23x17Registers::IOCON, false>(0b0100'1000);
            write8<ProcessorInterface::IOExpanderAddress::Lower16Lines, MCP23x17Registers::IOCON, false>(0b0100'1000);
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
            write16<IOExpanderAddress::Lower16Lines, MCP23x17Registers::INTCON, false>(0x0000) ;
            write16<IOExpanderAddress::Upper16Lines, MCP23x17Registers::GPINTEN, false>(0xFFFF) ;
            write16<IOExpanderAddress::Upper16Lines, MCP23x17Registers::INTCON, false>(0x0000) ;
            // setup the direction pins in the
            writeDirection<IOExpanderAddress::DataLines, false>(dataLinesDirection_ == 0xFF ? 0xFFFF : 0x0000);
            // configure INTCON to be compare against previous value
            write16<IOExpanderAddress::DataLines, MCP23x17Registers::INTCON, false>(0) ;
            write16<IOExpanderAddress::DataLines, MCP23x17Registers::GPINTEN, false>(0xFFFF) ;
            // write the default value out to the latch to start with
            write16<IOExpanderAddress::DataLines, MCP23x17Registers::OLAT, false>(latchedDataOutput.getWholeValue());
            // enable interrupts for accelerating write operations in the future
            SPI.endTransaction();
            setupDispatchTable();
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
            byte computedValue = 0;
            digitalWrite<i960Pinout::GPIOSelect, LOW>();
            SPDR = generateWriteOpcode(IOExpanderAddress::DataLines);
            {
                // this operation is very important to interleave because it can be very expensive to
                // but if we are also communicating over SPI, then the cost of this operation is nullified considerably
                latchedDataOutput.wholeValue_ = value;
            }
            while (!(SPSR & _BV(SPIF))) ; // wait
            SPDR = static_cast<byte>(MCP23x17Registers::OLAT); // instead of GPIO (which still writes to OLAT automatically), do the OLAT to be very explicit
            {
                // find out if this is the last transaction while we are talking to the io expander
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
        }
        return DigitalPin<i960Pinout::BLAST_>::isAsserted();
    }
    /**
     * @brief Query the ~BE0 and ~BE1 pins provided by the i960 to denote how the chipset should treat the current word in the transaction
     * @return The LoadStoreStyle derived from the ~BE0 and ~BE1 pins.
     */
    [[nodiscard]] static auto getStyle() noexcept {
        static constexpr auto Mask = pinToPortBit<i960Pinout::BE0>() | pinToPortBit<i960Pinout::BE1>();
        /// @todo figure out how to auto compute the shift amount
        //auto contents = TargetInputPort;
        return static_cast<LoadStoreStyle>((getAssociatedInputPort<i960Pinout::BE0>()& Mask));
    }
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
    /**
     * @brief An expression of the interrupt pins associated with the data lines io expander, an asserted interrupt line means that there has been
     * a change to that port (there are two 8-bit ports). This only applies when a write is being requested by the i960 (input). It is designed
     * to reduce the number of spi transactions during runtime.
     */
    enum class DataUpdateKind : byte {
        /**
         * @brief Both the upper and lower halves of the data lines have changed compared to last time.
         */
        Full16 = 0,
        /**
         * @brief Only the upper eight bits are different compared to last time
         */
        Upper8 = pinToPortBit<i960Pinout::DATA_LO8_INT>(),
        /**
         * @brief Only the lower eight bits are different compared to last time
         */
        Lower8 = pinToPortBit<i960Pinout::DATA_HI8_INT>(),
        /**
         * @brief Neither the upper or lower eight bits are different compared to last time. This means there is no need to read from the data lines at all.
         */
        Neither = pinToPortBit<i960Pinout::DATA_LO8_INT>() | pinToPortBit<i960Pinout::DATA_HI8_INT>(),
    };
    /**
     * @brief Query the interrupt lines tied to the data lines io expander
     * @return An expression of how the data lines have changed compared to last write operation
     */
    static inline byte getDataLineInputUpdateKind() noexcept {
        return (getAssociatedInputPort<i960Pinout::DATA_LO8_INT>() >> 4) & 0b11;
    }
public:
    /**
     * @brief Change the data lines io expander port direction from output to input (if applicable)
     */
    inline static void setupDataLinesForWrite() noexcept {
        if (!dataLinesDirection_) {
            invertDataLinesDirection();
        }
    }
    /**
     * @brief Change the data lines io expander port direction from input to output (if applicable)
     */
    inline static void setupDataLinesForRead() noexcept {
        if (dataLinesDirection_) {
            invertDataLinesDirection();
        }
    }
private:
    /**
     * @brief Describes how much of the address is the same compared to the previous transaction. This is done via the io expander interrupt lines.
     * In this case, the iocon's of each io expander are set to mirrored mode to reduce pin count. Thus only two bits are used instead of four bits.
     * Keep in mind that the burst address pins are _not_ connected to the io expanders so the interrupts will only trigger when the address of a
     * new transaction is different compared to the previous one. This is designed to reduce transaction latency. With external io devices, this drops
     * the overall latency from ~14 usec down to 4.5 usec because the addresses are the same.
     */
    enum class AddressUpdateKind : byte {
        /**
         * @brief Compared to last transaction, all 32-bits have changed so re-read all of them from the io expanders
         */
        Full32 = 0,
        /**
         * @brief Only the lower 16-bits of the address have changed so only read that io expander
         */
        Lower16 = pinToPortBit<i960Pinout::ADDRESS_HI_INT>() >> 6, // the upper 16 is marked as high
        /**
         * @brief Only the upper 16-bits of the address have changed so only read that io expander
         */
        Upper16 = pinToPortBit<i960Pinout::ADDRESS_LO_INT>() >> 6, // the lower 16 is marked as high
        /**
         * @brief Neither the upper or lower halves of the address have changed. Do not query the SPI bus at all
         */
        Neither = (pinToPortBit<i960Pinout::ADDRESS_HI_INT>() | pinToPortBit<i960Pinout::ADDRESS_LO_INT>()) >> 6, // both are marked high
    };
    static_assert(static_cast<byte>(AddressUpdateKind::Lower16) == 0b10);
    static_assert(static_cast<byte>(AddressUpdateKind::Upper16) == 0b01);
    static_assert(static_cast<byte>(AddressUpdateKind::Neither) == 0b11);
    /**
     * @brief Query the MCP23S17 interrupt pins to figure out which ports on the address lines had actually changed since the last transaction
     * @tparam useInterrupts When true, query the interrupt lines to generate a difference mask. When false, 0 is returned which means all have changed
     * @return A four bit code where each bit corresponds to a block of 8-bits and if they have changed since the last transaction or not.
     * Each zero found in the code signifies that that 8-bit quantity must be updated. If useInterrupts is off then zero is returned which
     * means the whole address must be updated.
     */
    template<bool useInterrupts>
    static byte getUpdateKind() noexcept {
        if constexpr (useInterrupts) {
            if constexpr (TargetBoard::onAtmega1284p_Type1()) {
                return (getAssociatedInputPort<i960Pinout::ADDRESS_LO_INT>() >> 6) & 0b11;
            }
        }
        return 0;
    }
private:
    /**
     * @brief Intelligently updates the GPIO pins attached to GPIO4 (or the fourth io expander). It will only communicate with the io expander if there is a legit change
     * @param value The new gpio values
     */
    static void updateGPIO4Pins(uint16_t value) noexcept {
        if (value != currentGPIO4Status_) {
            // only update the io expander when needed
            currentGPIO4Status_ = value;
            writeGPIO16<IOExpanderAddress::MemoryCommitExtras>(currentGPIO4Status_);
        }
    }
public:
    /**
     * @brief Put the i960 into reset
     */
    static void putCPUIntoReset() noexcept {
        updateGPIO4Pins(currentGPIO4Status_ & 0xFFFE);
    }
    /**
     * @brief Pull the i960 out of reset
     */
    static void pullCPUOutOfReset() noexcept {
        updateGPIO4Pins(currentGPIO4Status_ | 1);
    }
    /**
     * @brief Trigger the ~INT0 pin on the i960
     */
    static void triggerInt0() noexcept {
        updateGPIO4Pins(currentGPIO4Status_ & 0xFFFD);
        updateGPIO4Pins(currentGPIO4Status_ | 0b10);
    }
    /**
     * @brief Trigger the INT1 pin on the i960
     */
    static void triggerInt1() noexcept {
        updateGPIO4Pins(currentGPIO4Status_ | 0b100);
        updateGPIO4Pins(currentGPIO4Status_ & 0xFFFB);
    }
    /**
     * @brief Trigger the INT2 pin on the i960
     */
    static void triggerInt2() noexcept {
        updateGPIO4Pins(currentGPIO4Status_ | 0b1000);
        updateGPIO4Pins(currentGPIO4Status_ & 0xFFF7);
    }
    /**
     * @brief Trigger the ~INT3 pin on the i960
     */
    static void triggerInt3() noexcept {
        updateGPIO4Pins(currentGPIO4Status_ & 0xFFEF);
        updateGPIO4Pins(currentGPIO4Status_ | 0b1'0000);
    }
    /// @todo implement HOLD and LOCK functionality
private:

    struct DecodeDispatch {
        constexpr DecodeDispatch(byte updateKind,
                                 bool isReadOp,
                                 bool dataIsWriting,
                                 bool isRAMSpace,
                                 bool isIOSpace,
                                 bool be0,
                                 bool be1,
                                 bool singleWordTransaction,
                                 byte r = 0) : updateKinds(updateKind),
                                 readOperation(isReadOp),
                                 currentlyWrite(dataIsWriting),
                                 isRAMSpace_(isRAMSpace),
                                 isIOSpace_(isIOSpace),
                                 byteEnable0_(be0),
                                 byteEnable1_(be1),
                                 singleWordTransaction_(singleWordTransaction),
                                 rest(r) { }
        explicit constexpr DecodeDispatch(uint16_t value) : DecodeDispatch((value & 0b0000'0000'0000'0011),
                                                                           (value & 0b0000'0000'0000'0100),
                                                                           (value & 0b0000'0000'0000'1000),
                                                                           (value & 0b0000'0000'0001'0000),
                                                                           (value & 0b0000'0000'0010'0000),
                                                                           (value & 0b0000'0000'0100'0000),
                                                                           (value & 0b0000'0000'1000'0000),
                                                                           (value & 0b0000'0001'0000'0000),
                                                                           value >> 9) { }
        constexpr DecodeDispatch(byte porta, byte portd, bool dataIsWriting) : DecodeDispatch(
                (porta >> 6) & 0b11,
                (portd & 0b0001'0000) == 0,
                dataIsWriting,
                (portd & 0b0010'0000) == 0,
                (portd & 0b0100'0000) == 0,
                (porta & 0b0000'0001) == 0,
                (porta & 0b0000'0010) == 0,
                (portd & 0b0000'0100) == 0,
                0) {}


        uint8_t updateKinds;
        bool readOperation;
        bool currentlyWrite;
        bool isRAMSpace_;
        bool isIOSpace_;
        bool byteEnable0_;
        bool byteEnable1_;
        bool singleWordTransaction_;
        uint8_t rest;
        [[nodiscard]] constexpr uint8_t getUpdateKinds() const noexcept { return updateKinds; }
        [[nodiscard]] constexpr bool isReadOperation() const noexcept { return readOperation; }
        [[nodiscard]] constexpr bool isCurrentlyWrite() const noexcept { return currentlyWrite; }
        [[nodiscard]] constexpr bool inRAMSpace() const noexcept { return isRAMSpace_; }
        [[nodiscard]] constexpr bool inIOSpace() const noexcept { return isIOSpace_; }
        [[nodiscard]] constexpr bool inUnmappedSpace() const noexcept { return !inRAMSpace() && !inIOSpace(); }
        [[nodiscard]] constexpr bool inIllegalSpace() const noexcept { return inRAMSpace() && inIOSpace(); }
        [[nodiscard]] constexpr bool isByteEnable0() const noexcept { return byteEnable0_; }
        [[nodiscard]] constexpr bool isByteEnable1() const noexcept { return byteEnable1_; }
        [[nodiscard]] constexpr bool isSingleWordTransaction() const noexcept { return singleWordTransaction_; }
        template<bool useInterrupts>
        static uint8_t makeDynamicValue() noexcept {
            union {
                uint16_t value;
                struct {
                    uint16_t updateKinds: 2;
                    uint16_t readOperation: 1;
                    uint16_t currentlyWrite: 1;
                    uint16_t inRAMSpace : 1;
                    uint16_t inIOSpace : 1;
                    uint16_t be0 : 1;
                    uint16_t be1 : 1;
                    uint16_t blast : 1;
                    uint16_t rest : 7;
                };
            } thingy;
            if constexpr (useInterrupts) {
                thingy.updateKinds = (PINA >> 6) & 0b11;
            } else {
                thingy.updateKinds = 0;
            }
            thingy.readOperation = (PIND & 0b0001'0000) == 0;
            thingy.currentlyWrite = dataLinesDirection_ != 0;
            thingy.inRAMSpace = (PIND & 0b0010'0000) == 0;
            thingy.inIOSpace = (PIND & 0b0100'0000) == 0;
            thingy.be0 = (PINA & 0b01) == 0;
            thingy.be1 = (PINA & 0b10) == 0;
            thingy.blast = (PIND & 0b100) == 0;
            thingy.rest = 0;
            return thingy.value;
        }
    };
    /**
     * @brief Pull an entire 32-bit address from the upper and lower address io expanders. Updates the function to execute to satisfy the request
     * @tparam inDebugMode When true, any extra debugging code becomes active. Will be propagated to any child methods which take in the parameter
     */
    template<bool inDebugMode, DecodeDispatch index>
    inline static void full32BitUpdate() noexcept {
        static constexpr auto OffsetMask = CacheLine::CacheEntryMask;
        static constexpr auto OffsetShiftAmount = CacheLine::CacheEntryShiftAmount;
        static constexpr auto Lower16Opcode = generateReadOpcode(ProcessorInterface::IOExpanderAddress::Lower16Lines);
        static constexpr auto Upper16Opcode = generateReadOpcode(ProcessorInterface::IOExpanderAddress::Upper16Lines);
        static constexpr auto GPIOOpcode = static_cast<byte>(MCP23x17Registers::GPIO);
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
            if constexpr (index.inIOSpace()) {
                maskedSpaceTarget_ = highest & 0b000'11111;
                lastRead_ = ioSectionRead_[maskedSpaceTarget_];
                lastWrite_ = ioSectionWrite_[maskedSpaceTarget_];
                if constexpr (inDebugMode) {
                    lastReadDebug_ = ioSectionRead_Debug_[maskedSpaceTarget_];
                    lastWriteDebug_ = ioSectionWrite_Debug_[maskedSpaceTarget_];
                }
            }
            address_.bytes[3] = highest;
        }
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
        {
            // inside of here we have access to 12 cycles to play with, so let's actually do some operations while we wait
            // put scope ticks to force the matter
            cacheOffsetEntry_ = (lowest >> OffsetShiftAmount) & OffsetMask; // we want to make this quick to increment
            address_.bytes[0] = lowest;
        }
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        address_.bytes[1] = SPDR;
        digitalWrite<i960Pinout::GPIOSelect, HIGH>();
    }
    /**
     * @brief Only update the lower 16 bits of the current transaction's base address
     */
    static void lower16Update() noexcept {
        static constexpr auto OffsetMask = CacheLine::CacheEntryMask;
        static constexpr auto OffsetShiftAmount = CacheLine::CacheEntryShiftAmount;
        static constexpr auto Lower16Opcode = generateReadOpcode(ProcessorInterface::IOExpanderAddress::Lower16Lines);
        static constexpr auto GPIOOpcode = static_cast<byte>(MCP23x17Registers::GPIO);
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
    template<bool inDebugMode, DecodeDispatch index>
    static void doDispatch() noexcept {
        if constexpr (index.inIllegalSpace()) {
            signalHaltState(F("ILLEGAL SPACE DEFINED!"));
        } else {
            if constexpr (index.getUpdateKinds() == 0b00 || index.getUpdateKinds() == 0b01) {
                full32BitUpdate<inDebugMode, index>();
            } else if constexpr (index.getUpdateKinds() == 0b10) {
                lower16Update();
            } else {
                // do nothing
            }
            /// @todo use the new ram_space and io space pins to accelerate decoding
            if constexpr (inDebugMode) {
                Serial.print(F("Target Address: 0x"));
                Serial.println(address_.getWholeValue(), HEX);
            }
            if constexpr (index.isReadOperation()) {
                if constexpr (index.isCurrentlyWrite()) {
                    invertDataLinesDirection();
                }
                if constexpr (index.inRAMSpace()) {
                    performCacheRead<inDebugMode, index>();
                } else if constexpr (index.inUnmappedSpace()) {
                    performFallbackRead();
                } else {
                    if constexpr (inDebugMode) {
                        lastReadDebug_();
                    } else {
                        lastRead_();
                    }
                }
            } else {
                if constexpr (!index.isCurrentlyWrite()) {
                    invertDataLinesDirection();
                }
                if constexpr (index.inRAMSpace()) {
                    performCacheWrite<inDebugMode, index>();
                } else if constexpr (index.inUnmappedSpace()) {
                    performFallbackWrite();
                } else {
                    if constexpr (inDebugMode) {
                        lastWriteDebug_();
                    } else {
                        lastWrite_();
                    }
                }
            }
        }
    }
    template<bool inDebugMode>
    static constexpr BodyFunction DispatchTable_NewDataCycle[512] {
#define X(ind) doDispatch<inDebugMode, DecodeDispatch{ind} >
#define Y(ind) X((8*(ind)) + 0), X((8*(ind))+1), X((8*(ind))+2), X((8*(ind))+3), X((8*(ind))+4), X((8*(ind))+5), X((8*(ind))+6), X((8*(ind))+7)
#define Z(ind) Y((8*(ind)) + 0), Y((8*(ind))+1), Y((8*(ind))+2), Y((8*(ind))+3), Y((8*(ind))+4), Y((8*(ind))+5), Y((8*(ind))+6), Y((8*(ind))+7)
        Z(0), Z(1), Z(2), Z(3), Z(4), Z(5), Z(6), Z(7),
                //Z(8), Z(9), Z(10), Z(11), Z(12), Z(13), Z(14), Z(15)
#undef Z
#undef Y
#undef X
    };
public:
    /**
     * @brief Starts a new memory transaction. It is responsible for updating the target base address and then invoke the proper read/write function to satisfy the request
     * @tparam inDebugMode When true, use debug versions of the read/write function pointers to fulfill the request. This is passed to direct children as well.
     * @tparam useInterrupts If true, then query the directly connected interrupt pins to get a proper update mask
     */
    template<bool inDebugMode, bool useInterrupts = true>
    static void newDataCycle() noexcept {
        DispatchTable_NewDataCycle<inDebugMode>[DecodeDispatch::makeDynamicValue<useInterrupts>()]();
    }
    /**
     * @brief Return the least significant byte of the address, useful for CoreChipsetFeatures
     * @return The LSB of the address
     */
    [[nodiscard]] static auto getPageOffset() noexcept { return address_.bytes[0]; }
    /**
     * @brief Return bits 8-15 of the address. Internally this is known as the page index.
     * @return The page index based off of the current address
     */
    [[nodiscard]] static auto getPageIndex() noexcept { return address_.bytes[1]; }

    /**
     * @brief loads a cache line based on base transaction address and then bursts up to 16 bytes to the i960
     * @tparam inDebugMode Are we in debug mode?
     * @param line The cache line which we will be using for this transaction
     */
    template<bool inDebugMode, DecodeDispatch index>
    static inline void performCacheRead(const CacheLine& line) noexcept {
        SPI.beginTransaction(SPISettings(TargetBoard::runIOExpanderSPIInterfaceAt(), MSBFIRST, SPI_MODE0));
        if constexpr (index.isSingleWordTransaction()) {
            (void)setDataBits(line.get(getCacheOffsetEntry()));
            DigitalPin<i960Pinout::Ready>::pulse();
        } else {
            // read 32-bits at a time instead of 16 just to keep the throughput increased
            for (auto offset = getCacheOffsetEntry();; offset += 8) {
                // this is more unsafe than waiting later on in the process as we are guessing that we need this data
                // These full words will hold onto garbage data if we span two cache lines. That is actually okay because
                // the i960 will never request that data. It will operate on a maximum of 16-bytes at a time.
                // For consistency, I am leaving the loop in (my research may be flawed in some way)
                auto basePtr = line.getRawData() + offset;
                auto &a0 = basePtr[0];
                auto &a1 = basePtr[1];
                auto &a2 = basePtr[2];
                auto &a3 = basePtr[3];
                auto &a4 = basePtr[4];
                auto &a5 = basePtr[5];
                auto &a6 = basePtr[6];
                auto &a7 = basePtr[7];
                auto isLastRead = setDataBits(a0.getWholeValue());
                DigitalPin<i960Pinout::Ready>::pulse();
                if (isLastRead) {
                    break;
                }
                isLastRead = setDataBits(a1.getWholeValue());
                DigitalPin<i960Pinout::Ready>::pulse();
                if (isLastRead) {
                    break;
                }
                // we are looking at a double word, triple word, or quad word
                isLastRead = setDataBits(a2.getWholeValue());
                DigitalPin<i960Pinout::Ready>::pulse();
                if (isLastRead) {
                    break;
                }
                isLastRead = setDataBits(a3.getWholeValue());
                DigitalPin<i960Pinout::Ready>::pulse();
                if (isLastRead) {
                    break;
                }
                // okay so we are looking at a triple word or quad word operation
                isLastRead = setDataBits(a4.getWholeValue());
                DigitalPin<i960Pinout::Ready>::pulse();
                if (isLastRead) {
                    break;
                }
                isLastRead = setDataBits(a5.getWholeValue());
                DigitalPin<i960Pinout::Ready>::pulse();
                if (isLastRead) {
                    break;
                }
                // okay so we are looking at a quad word operation of some kind
                // perhaps the processor loading instruction into the data cache?
                isLastRead = setDataBits(a6.getWholeValue());
                DigitalPin<i960Pinout::Ready>::pulse();
                if (isLastRead) {
                    break;
                }
                isLastRead = setDataBits(a7.getWholeValue());
                DigitalPin<i960Pinout::Ready>::pulse();
                if (isLastRead) {
                    break;
                }
            }
        }
        SPI.endTransaction();
    }
    static inline void performCacheRead_Single(const CacheLine& line) noexcept {
        SPI.beginTransaction(SPISettings(TargetBoard::runIOExpanderSPIInterfaceAt(), MSBFIRST, SPI_MODE0));
        SPI.endTransaction();
    }
private:
    /**
     * @brief Describes a write into the data cache meant to accelerate burst writes from the i960. Allows the chipset to pull values
     * from the i960 much faster.
     */
    struct CacheWriteRequest {
        LoadStoreStyle style;
        SplitWord16 value;
        /**
         * @brief Save the given data to the target cache line
         * @tparam CacheLine The type of the cache line being used by the data cache
         * @param line The cache line to write to
         */
        inline void set(byte offset, CacheLine& line) const noexcept {
            line.set(offset, style, value);
        }
    };
    /**
     * @brief A cache of transactions to carry out from a burst write
     */
    static inline CacheWriteRequest transactions[8];
    template<byte index, MCP23x17Registers reg>
    static void grab8Data() noexcept {
        digitalWrite<i960Pinout::GPIOSelect, LOW>();
        SPDR = generateReadOpcode(IOExpanderAddress::DataLines);
        asm volatile ("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        SPDR = static_cast<byte>(reg);
        asm volatile ("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        SPDR = 0;
        asm volatile ("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        latchedDataInput_.bytes[index] = SPDR;
        digitalWrite<i960Pinout::GPIOSelect, HIGH>();
    }
    static void upper8DataGrab() noexcept {
        grab8Data<1, MCP23x17Registers::GPIOB>();
    }
    static void lower8DataGrab() noexcept {
        grab8Data<0, MCP23x17Registers::GPIOA>();
    }
    static void fullDataLineGrab() noexcept {
        digitalWrite<i960Pinout::GPIOSelect, LOW>();
        SPDR = generateReadOpcode(IOExpanderAddress::DataLines);
        asm volatile ("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        SPDR = static_cast<byte>(MCP23x17Registers::GPIO);
        asm volatile ("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        SPDR = 0;
        asm volatile ("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        auto lower = SPDR;
        SPDR = 0;
        {
            latchedDataInput_.bytes[0] = lower;
        }
        asm volatile ("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        latchedDataInput_.bytes[1] = SPDR;
        digitalWrite<i960Pinout::GPIOSelect, HIGH>();
    }
    static inline BodyFunction DataLineUpdateFunctions[4] {
            fullDataLineGrab, // 0b00
            upper8DataGrab, // 0b01
            lower8DataGrab, // 0b10
            []() noexcept { },
    };
    static void updateDataInputLatch() noexcept {
        DataLineUpdateFunctions[getDataLineInputUpdateKind()]();
    }
    [[nodiscard]]
    static bool getDataBits(CacheWriteRequest& request) noexcept {
        request.style = getStyle();
        bool outcome = DigitalPin<i960Pinout::BLAST_>::isAsserted();
        updateDataInputLatch();
        request.value = latchedDataInput_;
        return outcome;
    }
    template<byte count>
    static bool getDataBits() noexcept {
        static_assert(count < 8, "Can only transmit 8 transactions at a time");
        return getDataBits(transactions[count]);
    }
    template<byte count>
    static inline void commitTransactions(CacheLine& line, byte offset) noexcept {
        static_assert(count <= 8, "Can only transmit 8 transactions at a time");
        for (byte i = 0, j = offset; i < count; ++i, ++j) {
            transactions[i].set(j, line);
        }
    }
public:
    /**
     * @brief commit up to 16-bytes of data from the i960 to a specified cache line
     * @tparam inDebugMode are we in debug mode?
     * @param line The cache line to write to.
     */
    template<bool inDebugMode, DecodeDispatch index>
    static inline void performCacheWrite(CacheLine& line) noexcept {
        SPI.beginTransaction(SPISettings(TargetBoard::runIOExpanderSPIInterfaceAt(), MSBFIRST, SPI_MODE0));
        if constexpr (index.isSingleWordTransaction()) {
            auto offset = getCacheOffsetEntry();
            updateDataInputLatch();
            DigitalPin<i960Pinout::Ready>::pulse();
            if constexpr (index.isByteEnable0()) {
               if constexpr (index.isByteEnable1()) {
                   line.set<LoadStoreStyle::Full16>(offset, latchedDataInput_);
               } else {
                   line.set<LoadStoreStyle::Lower8>(offset, latchedDataInput_);
               }
            } else {
                if constexpr (index.isByteEnable1()) {
                    line.set<LoadStoreStyle::Upper8>(offset, latchedDataInput_);
                } else {
                    line.set<LoadStoreStyle::None>(offset, latchedDataInput_);
                }
            }
        } else {
            for (auto offset = getCacheOffsetEntry();; offset += 8) {
                auto isLast = getDataBits<0>();
                DigitalPin<i960Pinout::Ready>::pulse();
                if (isLast) {
                    commitTransactions<1>(line, offset);
                    break;
                }
                isLast = getDataBits<1>();
                DigitalPin<i960Pinout::Ready>::pulse();
                if (isLast) {
                    commitTransactions<2>(line, offset);
                    break;
                }
                isLast = getDataBits<2>();
                DigitalPin<i960Pinout::Ready>::pulse();
                if (isLast) {
                    commitTransactions<3>(line, offset);
                    break;
                }
                isLast = getDataBits<3>();
                DigitalPin<i960Pinout::Ready>::pulse();
                if (isLast) {
                    commitTransactions<4>(line, offset);
                    break;
                }
                isLast = getDataBits<4>();
                DigitalPin<i960Pinout::Ready>::pulse();
                if (isLast) {
                    commitTransactions<5>(line, offset);
                    break;
                }
                isLast = getDataBits<5>();
                DigitalPin<i960Pinout::Ready>::pulse();
                if (isLast) {
                    commitTransactions<6>(line, offset);
                    break;
                }
                isLast = getDataBits<6>();
                DigitalPin<i960Pinout::Ready>::pulse();
                if (isLast) {
                    commitTransactions<7>(line, offset);
                    break;
                }
                isLast = getDataBits<7>();
                DigitalPin<i960Pinout::Ready>::pulse();
                // perform the commit at this point
                commitTransactions<8>(line, offset);
                if (isLast) {
                    break;
                }
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
            bool isLast = DigitalPin<i960Pinout::BLAST_>::isAsserted();
            LoadStoreStyle currLSS = getStyle();
            byte pageIndex = getPageIndex();
            updateDataInputLatch();
            if constexpr (inDebugMode) {
                Serial.print(F("\tPage Index: 0x"));
                Serial.println(pageIndex, HEX);
                Serial.print(F("\tPage Offset: 0x"));
                Serial.println(pageOffset, HEX);
                Serial.print(F("\tData To Write: 0x"));
                Serial.println(latchedDataInput_.getWholeValue(), HEX);
            }
            T::write(pageIndex,
                     pageOffset,
                     currLSS,
                     latchedDataInput_);
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
    static void performFallbackRead() noexcept;
    /**
     * @brief Used when the transaction is writing to unmapped memory in the i960's memory space. Nothing will be written but an artificial delay will be introduced to be on the safe side.
     * @tparam inDebugMode are we in debug mode?
     */
    static void performFallbackWrite() noexcept;
public:
    /**
     * @brief Complete the process of setting up the processor interface by seeding the cached function pointers with valid addresses.
     */
private:
    template<typename T>
    static void
    registerExternalDeviceWithLookupTable() noexcept {
        static constexpr byte reducedSectionId = T::SectionID & 0b0001'1111;
        ioSectionRead_[reducedSectionId] = performExternalDeviceRead<T, false>;
        ioSectionWrite_[reducedSectionId] = performExternalDeviceWrite<T, false>;
        if constexpr (CompileInAddressDebuggingSupport) {
            ioSectionRead_Debug_[reducedSectionId] = performExternalDeviceRead<T, true>;
            ioSectionWrite_Debug_[reducedSectionId] = performExternalDeviceWrite<T, true>;
        }
    }
    template<bool debug, DecodeDispatch index>
    static void performCacheRead() noexcept {
        performCacheRead<debug, index>(theCache.getLine(address_));
    }

    template<bool debug, DecodeDispatch index>
    static void performCacheWrite() noexcept {
        performCacheWrite<debug, index>(theCache.getLine(address_));
    }

    static void setupDispatchTable() noexcept;
private:
    static inline SplitWord32 address_{0};
    static inline SplitWord16 latchedDataOutput {0};
    static inline byte dataLinesDirection_ = 0;
    static inline byte cacheOffsetEntry_ = 0;
    static inline bool initialized_ = false;
    static inline BodyFunction lastRead_ = performFallbackRead;
    static inline BodyFunction lastReadDebug_ = performFallbackRead;
    static inline BodyFunction lastWrite_ = performFallbackWrite;
    static inline BodyFunction lastWriteDebug_ = performFallbackWrite;
    static inline uint16_t currentGPIO4Status_ = 0b00000000'10010010;
    static inline uint16_t currentGPIO4Direction_ = 0b00000000'00100000;
    static constexpr uint8_t initialIOCONValue_ = 0b0000'1000;
    static inline SplitWord16 latchedDataInput_ {0};
    using SpaceDispatchTable = BodyFunction[32];
    static inline SpaceDispatchTable ioSectionRead_ { nullptr };
    static inline SpaceDispatchTable ioSectionWrite_ { nullptr };
    static inline SpaceDispatchTable ioSectionRead_Debug_ { nullptr };
    static inline SpaceDispatchTable ioSectionWrite_Debug_ { nullptr };
    static inline byte maskedSpaceTarget_ = 0;
};
// 8 IOExpanders to a single enable line for SPI purposes
// 4 of them are reserved

#endif //ARDUINO_IOEXPANDERS_H
