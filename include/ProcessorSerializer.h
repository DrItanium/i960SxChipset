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
#include "CoreChipsetFeatures.h"
#include "MCP23S17.h"

/**
 * @brief Static class which is responsible for managing the interacting between the chipset and the i960 itself
 */
struct ProcessorInterface final {
    using MCP23x17Registers = MCP23S17::Registers;
    using IOExpanderAddress = MCP23S17::HardwareDeviceAddress;
    static constexpr auto DataLines = IOExpanderAddress ::Device0;
    static constexpr auto Lower16Lines = IOExpanderAddress ::Device1;
    static constexpr auto Upper16Lines = IOExpanderAddress ::Device2;
    static constexpr auto MemoryCommitExtras = IOExpanderAddress ::Device3;

    static consteval byte generateReadOpcode(IOExpanderAddress address) noexcept {
        return MCP23S17::generateReadOpcode(static_cast<uint8_t>(address), false);
    }
    static consteval byte generateWriteOpcode(IOExpanderAddress address) noexcept {
        return MCP23S17::generateWriteOpcode(static_cast<uint8_t>(address), false);
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
        auto output = MCP23S17::read16<addr, opcode, DigitalPin<i960Pinout::GPIOSelect>>();
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
        auto result = MCP23S17::read8<addr, opcode, DigitalPin<i960Pinout::GPIOSelect>>();
        if constexpr (standalone) {
            SPI.endTransaction();
        }
        return result;
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
        if constexpr (standalone) {
            SPI.beginTransaction(SPISettings(TargetBoard::runIOExpanderSPIInterfaceAt(), MSBFIRST, SPI_MODE0));
        }
        MCP23S17::write16<addr, opcode, DigitalPin<i960Pinout::GPIOSelect>>(value);
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
        MCP23S17::write8<addr, opcode, DigitalPin<i960Pinout::GPIOSelect>>(value);
        if constexpr (standalone) {
            SPI.endTransaction();
        }
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
     *        The highest 512 megabytes is known as the io area. This function checks to see if the io space gpio is asserted
     * @return True if the io space gpio is asserted.
     */
    [[nodiscard]] static inline auto inIOSpace() noexcept {
#ifdef CHIPSET_TYPE_MEGA
        /// @todo implement
        return false;
#else
        return inIOSpace_;
#endif
    }
    /**
     * @brief On board the type 1.01 boards, there is an AHC138 which divides the memory space into eight 512 megabyte sections;
     *        The lowest 512 megabytes is known as the ram area. This function checks to see if the ram space gpio is asserted
     * @return True if the ram space gpio is asserted.
     */
    [[nodiscard]] static inline auto inRAMSpace() noexcept {
        return !inIOSpace();
    }
    /**
     * @brief Check the W/~R pin to see if we are dealing with a read transaction.
     * Only needs to be queried once at the beginning of a new transaction
     * @return If true, then the current transaction is a read operation. If false, then the current transaction is a write operation.
     */
    [[nodiscard]] static bool isReadOperation() noexcept {
#ifdef CHIPSET_TYPE_MEGA
        /// @todo implement
        return isReadOperation_;
#else
        return DigitalPin<i960Pinout::W_R_>::isAsserted();
#endif
    }
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
            if constexpr (TargetBoard::onAtmega1284p_Type1()) {
                SPI.beginTransaction(SPISettings(TargetBoard::runIOExpanderSPIInterfaceAt(), MSBFIRST, SPI_MODE0));
                // immediately pull the i960 into reset as soon as possible
                // now all devices tied to this ~CS pin have separate addresses
                // make each of these inputs
                writeDirection<Lower16Lines, false>(0xFFFF);
                writeDirection<Upper16Lines, false>(0xFFFF);
                // enable pin change interrupts on address lines
                write16<Lower16Lines, MCP23x17Registers::GPINTEN, false>(0xFFFF);
                write16<Lower16Lines, MCP23x17Registers::INTCON, false>(0x0000);
                write16<Upper16Lines, MCP23x17Registers::GPINTEN, false>(0xFFFF);
                write16<Upper16Lines, MCP23x17Registers::INTCON, false>(0x0000);
                // setup the direction pins in the
                writeDirection<DataLines, false>(dataLinesDirection_ == 0xFF ? 0xFFFF : 0x0000);
                // configure INTCON to be compare against previous value
                write16<DataLines, MCP23x17Registers::INTCON, false>(0);
                write16<DataLines, MCP23x17Registers::GPINTEN, false>(0xFFFF);
                // write the default value out to the latch to start with
                write16<DataLines, MCP23x17Registers::OLAT, false>(latchedDataOutput.getWholeValue());
                // enable interrupts for accelerating write operations in the future
                SPI.endTransaction();
            } else if (TargetBoard::onAtmega2560_TypeMega()) {
                // setup the data lines
                getAssociatedDirectionPort<i960Pinout::Data0>() = 0xFF;
                getAssociatedDirectionPort<i960Pinout::Data8>() = 0xFF;
            }
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
    static void setDataBits(uint16_t value) noexcept {
        // the latch is preserved in between data line changes
        // okay we are still pointing as output values
        // check the latch and see if the output value is the same as what is latched
        if (latchedDataOutput.getWholeValue() != value) {
            //latchedDataOutput.wholeValue_ = value;
            byte computedValue = 0;
            digitalWrite<i960Pinout::GPIOSelect, LOW>();
            SPDR = generateWriteOpcode(DataLines);
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
    }
    /**
     * @brief Query the ~BE0 and ~BE1 pins provided by the i960 to denote how the chipset should treat the current word in the transaction
     * @return The LoadStoreStyle derived from the ~BE0 and ~BE1 pins.
     */
    [[nodiscard]] static auto getStyle() noexcept {
#ifdef CHIPSET_TYPE_MEGA
        /// @todo properly implement
        //return static_cast<LoadStoreStyle>(getCTL0().bits.byteEnable);
        return LoadStoreStyle::Full16;
#else
        static constexpr auto Mask = pinToPortBit<i960Pinout::BE0>() | pinToPortBit<i960Pinout::BE1>();
        /// @todo figure out how to auto compute the shift amount
        //auto contents = TargetInputPort;
        return static_cast<LoadStoreStyle>((getAssociatedInputPort<i960Pinout::BE0>()& Mask));
#endif
    }
private:
    /**
     * @brief A method responsible for turning data lines from input to output and vice-versa. It also inverts the tracking byte used by this
     * class to only update the direction when needed (SPI bus transactions are relatively expensive).
     */
    inline static void invertDataLinesDirection() noexcept {
        SPI.beginTransaction(SPISettings(TargetBoard::runIOExpanderSPIInterfaceAt(), MSBFIRST, SPI_MODE0));
        digitalWrite<i960Pinout::GPIOSelect, LOW>();
        SPDR = generateWriteOpcode(DataLines);
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
#ifdef CHIPSET_TYPE1
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
#else
        /**
         * @brief Only the upper eight bits are different compared to last time
         */
        Upper8 = 0b01,
        /**
         * @brief Only the lower eight bits are different compared to last time
         */
        Lower8 = 0b10,
        /**
         * @brief Neither the upper or lower eight bits are different compared to last time. This means there is no need to read from the data lines at all.
         */
        Neither = 0b11,
#endif
    };
    /**
     * @brief Query the interrupt lines tied to the data lines io expander
     * @return An expression of how the data lines have changed compared to last write operation
     */
    static inline byte getDataLineInputUpdateKind() noexcept {
#ifdef CHIPSET_TYPE1
        return (getAssociatedInputPort<i960Pinout::DATA_LO8_INT>() >> 4) & 0b11;
#else
        // update everything every single time
        return 0;
#endif
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
#ifdef CHIPSET_TYPE1
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
#else
    Upper16 = 0b01,
    Lower16 = 0b10,
    Neither = 0b11,
#endif
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
#ifdef CHIPSET_TYPE1
        if constexpr (useInterrupts) {
            if constexpr (TargetBoard::onAtmega1284p_Type1()) {
                return (getAssociatedInputPort<i960Pinout::ADDRESS_LO_INT>() >> 6) & 0b11;
            }
        }
#endif
        return 0;
    }
public:
    /**
     * @brief Trigger the ~INT0 pin on the i960
     */
    static void triggerInt0() noexcept {
        DigitalPin<i960Pinout::INT960_0_>::pulse();
    }
    /**
     * @brief Trigger the INT1 pin on the i960
     */
    static void triggerInt1() noexcept {
        DigitalPin<i960Pinout::INT960_1>::pulse();
    }
    /**
     * @brief Trigger the INT2 pin on the i960
     */
    static void triggerInt2() noexcept {
        DigitalPin<i960Pinout::INT960_2>::pulse();
    }
    /**
     * @brief Trigger the ~INT3 pin on the i960
     */
    static void triggerInt3() noexcept {
        DigitalPin<i960Pinout::INT960_3_>::pulse();
    }
    static void holdBus() noexcept {
        DigitalPin<i960Pinout::HOLD>::assertPin();
    }
    static void unholdBus() noexcept {
        DigitalPin<i960Pinout::HOLD>::deassertPin();
    }
    static bool busHeld() noexcept {
        return DigitalPin<i960Pinout::HLDA>::isAsserted();
    }
    /// @todo implement HOLD and LOCK functionality
private:

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
    [[nodiscard]] static bool isBurstLast() noexcept { return DigitalPin<i960Pinout::BLAST_>::isAsserted(); }
    struct DecodeDispatch {
        constexpr DecodeDispatch(bool isReadOp,
                                 bool dataIsWriting,
                                 bool isIOSpace) :
                                 readOperation(isReadOp),
                                 currentlyWrite(dataIsWriting),
                                 isIOSpace_(isIOSpace) { }
        explicit constexpr DecodeDispatch(uint8_t value) : DecodeDispatch(static_cast<bool>(value & 0b0000'0001),
                                                                          static_cast<bool>(value & 0b0000'0010),
                                                                          static_cast<bool>(value & 0b0000'0100)) { }
        bool readOperation;
        bool currentlyWrite;
        bool isIOSpace_;
        [[nodiscard]] constexpr bool isReadOperation() const noexcept { return readOperation; }
        [[nodiscard]] constexpr bool isCurrentlyWrite() const noexcept { return currentlyWrite; }
        [[nodiscard]] constexpr bool inIOSpace() const noexcept { return isIOSpace_; }
        [[nodiscard]] constexpr bool inRAMSpace() const noexcept { return !inIOSpace(); }
        static uint8_t makeDynamicValue() noexcept {
            union {
                uint8_t value;
                struct {
                    uint8_t readOperation: 1;
                    uint8_t currentlyWrite: 1;
                    uint8_t inIOSpace : 1;
                };
            } thingy;
            thingy.value = 0;
            thingy.readOperation = ProcessorInterface::isReadOperation();
            thingy.currentlyWrite = dataLinesDirection_ != 0;
            thingy.inIOSpace = ProcessorInterface::inIOSpace();
            return thingy.value;
        }
    };
    template<bool inDebugMode, bool isRead, bool isIO, bool invertDirection>
    inline static void performEffectiveDispatch() noexcept {
        if constexpr (invertDirection) {
            invertDataLinesDirection();
        }
        if constexpr (isRead) {
            if constexpr (isIO) {
                performExternalDeviceOperation<ConfigurationSpace, inDebugMode, isRead>();
            } else {
                performCacheRead<inDebugMode>();
            }
        } else {
            if constexpr (isIO) {
                performExternalDeviceOperation<ConfigurationSpace, inDebugMode, isRead>();
            } else {
                performCacheWrite<inDebugMode>();
            }
        }
    }
    template<bool inDebugMode, byte GPIOOpcode, auto OffsetShiftAmount, auto OffsetMask, typename T>
    inline static void ioReadOperation32() {
        while (!(SPSR & _BV(SPIF))); // wait
        SPDR = GPIOOpcode;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        SPDR = 0;
        asm volatile("nop");
        bool dataLinesAreWriting = dataLinesDirection_ != 0;
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
        if (dataLinesAreWriting) {
            invertDataLinesDirection();
        }
        performExternalDeviceOperation<T, inDebugMode, true>();
    }
    template<bool inDebugMode, byte GPIOOpcode, auto OffsetShiftAmount, auto OffsetMask>
    inline static void ramReadOperation32() {
        while (!(SPSR & _BV(SPIF))); // wait
        SPDR = GPIOOpcode;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        SPDR = 0;
        asm volatile("nop");
        bool dataLinesAreWriting = dataLinesDirection_ != 0;
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
        if (dataLinesAreWriting) {
            invertDataLinesDirection();
        }
        performCacheRead<inDebugMode>();
    }
    template<bool inDebugMode, byte GPIOOpcode, auto OffsetShiftAmount, auto OffsetMask, typename T>
    inline static void ioWriteOperation32() noexcept {
        while (!(SPSR & _BV(SPIF))); // wait
        SPDR = GPIOOpcode;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        SPDR = 0;
        asm volatile("nop");
        bool dataLinesAreWriting = dataLinesDirection_ != 0;
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
        auto pageIndex = SPDR;
        address_.bytes[1] = pageIndex;
        digitalWrite<i960Pinout::GPIOSelect, HIGH>();
        if (!dataLinesAreWriting) {
            invertDataLinesDirection();
        }
        performExternalDeviceOperation<T, inDebugMode, false>();
    }
    template<bool inDebugMode, byte GPIOOpcode, auto OffsetShiftAmount, auto OffsetMask>
    inline static void ramWriteOperation32() noexcept {
        while (!(SPSR & _BV(SPIF))); // wait
        SPDR = GPIOOpcode;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        SPDR = 0;
        asm volatile("nop");
        bool dataLinesAreWriting = dataLinesDirection_ != 0;
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
        if (!dataLinesAreWriting) {
            invertDataLinesDirection();
        }
        performCacheWrite<inDebugMode>();
    }
    template<bool inDebugMode, byte Lower16Opcode, byte GPIOOpcode, auto OffsetShiftAmount, auto OffsetMask>
    inline static void writeOperation32() noexcept {
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
        asm volatile("nop");
        address_.bytes[3] = highest;
        inIOSpace_ = highest >= 0xFE;
        if (inIOSpace_) {
            ioWriteOperation32<inDebugMode, GPIOOpcode, OffsetShiftAmount, OffsetMask, ConfigurationSpace>();
        } else {
            ramWriteOperation32<inDebugMode, GPIOOpcode, OffsetShiftAmount, OffsetMask>();
        }
    }
    template<bool inDebugMode, byte Lower16Opcode, byte GPIOOpcode, auto OffsetShiftAmount, auto OffsetMask>
    inline static void readOperation32() noexcept {
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
        asm volatile("nop");
        address_.bytes[3] = highest;
        inIOSpace_ = highest >= 0xFE;
        if (inIOSpace_) {
            ioReadOperation32<inDebugMode, GPIOOpcode, OffsetShiftAmount, OffsetMask, ConfigurationSpace>();
        } else {
            ramReadOperation32<inDebugMode, GPIOOpcode, OffsetShiftAmount, OffsetMask>();
        }
    }
    /**
     * @brief Pull an entire 32-bit address from the upper and lower address io expanders. Updates the function to execute to satisfy the request
     * @tparam inDebugMode When true, any extra debugging code becomes active. Will be propagated to any child methods which take in the parameter
     */
    template<bool inDebugMode>
    inline static void full32BitUpdate() noexcept {
        static constexpr auto OffsetMask = CacheLine::CacheEntryMask;
        static constexpr auto OffsetShiftAmount = CacheLine::CacheEntryShiftAmount;
        static constexpr auto Lower16Opcode = generateReadOpcode(Lower16Lines);
        static constexpr auto Upper16Opcode = generateReadOpcode(Upper16Lines);
        static constexpr auto GPIOOpcode = static_cast<byte>(MCP23x17Registers::GPIO);
        // we want to overlay actions as much as possible during spi transfers, there are blocks of waiting for a transfer to take place
        // where we can insert operations to take place that would otherwise be waiting
        digitalWrite<i960Pinout::GPIOSelect, LOW>();
        SPDR = Upper16Opcode;
        asm volatile("nop");
        if (isReadOperation()) {
            readOperation32<inDebugMode, Lower16Opcode, GPIOOpcode, OffsetShiftAmount, OffsetMask>();
        } else {
            writeOperation32<inDebugMode, Lower16Opcode, GPIOOpcode, OffsetShiftAmount, OffsetMask>();
        }
    }
    template<bool inDebugMode, auto shift, auto mask, bool isRead, bool isIO, bool invertDirection>
    static void completeOp16() noexcept {
        while (!(SPSR & _BV(SPIF))); // wait
        auto lowest = SPDR;
        SPDR = 0;
        {
            // inside of here we have access to 12 cycles to play with, so let's actually do some operations while we wait
            // put scope ticks to force the matter
            cacheOffsetEntry_ = (lowest >> shift) & mask; // we want to make this quick to increment
            address_.bytes[0] = lowest;
        }
        while (!(SPSR & _BV(SPIF))); // wait
        address_.bytes[1] = SPDR;
        digitalWrite<i960Pinout::GPIOSelect, HIGH>();
        performEffectiveDispatch<inDebugMode, isRead, isIO, invertDirection>();
    }
    template<bool inDebugMode, auto shift, auto mask, bool isRead, bool isIO>
    static void opSpace16() noexcept {
        while (!(SPSR & _BV(SPIF))); // wait
        SPDR = 0;
        asm volatile("nop");
        if constexpr (bool currentlyWrite = dataLinesDirection_ != 0; isRead) {
            // so we are looking at a read operation
            if (currentlyWrite) {
                // we are currently in write mode so we need to do an invert
                completeOp16<inDebugMode, shift, mask, isRead, isIO, true>();
            } else {
                completeOp16<inDebugMode, shift, mask, isRead, isIO, false>();
            }
        } else {
            if (currentlyWrite) {
                completeOp16<inDebugMode, shift, mask, isRead, isIO, false>();
            } else {
                // we are currently in read mode so we need to do an invert
                completeOp16<inDebugMode, shift, mask, isRead, isIO, true>();
            }
        }
    }
    template<bool inDebugMode, byte opcode, auto shift, auto mask, bool isRead>
    static void op16() noexcept {
        while (!(SPSR & _BV(SPIF))); // wait
        SPDR = opcode;
        asm volatile("nop");
        if (inIOSpace_) {
            opSpace16<inDebugMode, shift, mask, isRead, true>();
        } else {
            opSpace16<inDebugMode, shift, mask, isRead, false>();
        }
    }
    /**
     * @brief Only update the lower 16 bits of the current transaction's base address
     */
    template<bool inDebugMode>
    static void lower16Update() noexcept {
        static constexpr auto OffsetMask = CacheLine::CacheEntryMask;
        static constexpr auto OffsetShiftAmount = CacheLine::CacheEntryShiftAmount;
        static constexpr auto Lower16Opcode = generateReadOpcode(Lower16Lines);
        static constexpr auto GPIOOpcode = static_cast<byte>(MCP23x17Registers::GPIO);
        // read only the lower half
        // we want to overlay actions as much as possible during spi transfers, there are blocks of waiting for a transfer to take place
        // where we can insert operations to take place that would otherwise be waiting
        digitalWrite<i960Pinout::GPIOSelect, LOW>();
        SPDR = Lower16Opcode;
        asm volatile("nop");
        if (isReadOperation()) {
            op16<inDebugMode, GPIOOpcode, OffsetShiftAmount, OffsetMask, true>();
        } else {
            op16<inDebugMode, GPIOOpcode, OffsetShiftAmount, OffsetMask, false>();
        }
    }

    /**
     * @brief Only update the lower 16 bits of the current transaction's base address
     */
    template<bool inDebugMode>
    static void upper16Update() noexcept {
        static constexpr auto OffsetMask = CacheLine::CacheEntryMask;
        static constexpr auto OffsetShiftAmount = CacheLine::CacheEntryShiftAmount;
        static constexpr auto Upper16Opcode = generateReadOpcode(Upper16Lines);
        static constexpr auto GPIOOpcode = static_cast<byte>(MCP23x17Registers::GPIO);
        // read only the lower half
        // we want to overlay actions as much as possible during spi transfers, there are blocks of waiting for a transfer to take place
        // where we can insert operations to take place that would otherwise be waiting
        digitalWrite<i960Pinout::GPIOSelect, LOW>();
        SPDR = Upper16Opcode;
        asm volatile("nop");
        if (isReadOperation()) {
            op16<inDebugMode, GPIOOpcode, OffsetShiftAmount, OffsetMask, true>();
        } else {
            op16<inDebugMode, GPIOOpcode, OffsetShiftAmount, OffsetMask, false>();
        }
    }
    template<bool inDebugMode, DecodeDispatch index >
    inline static void doDispatch() noexcept {
        /// @todo use the new ram_space and io space pins to accelerate decoding
        if constexpr (inDebugMode) {
            Serial.print(F("Target Address: 0x"));
            Serial.println(address_.getWholeValue(), HEX);
        }
        if constexpr (index.isReadOperation()) {
            if constexpr (index.isCurrentlyWrite()) {
                invertDataLinesDirection();
            }
            if constexpr (index.inIOSpace()) {
                performExternalDeviceOperation<ConfigurationSpace, inDebugMode, true>();
            } else {
                performCacheRead<inDebugMode>();
            }
        } else {
            if constexpr (!index.isCurrentlyWrite()) {
                invertDataLinesDirection();
            }
            if constexpr (index.inIOSpace()) {
                performExternalDeviceOperation<ConfigurationSpace, inDebugMode, false>();
            } else {
                performCacheWrite<inDebugMode>();
            }
        }
    }
public:
    /**
     * @brief Starts a new memory transaction. It is responsible for updating the target base address and then invoke the proper read/write function to satisfy the request
     * @tparam inDebugMode When true, use debug versions of the read/write function pointers to fulfill the request. This is passed to direct children as well.
     * @tparam useInterrupts If true, then query the directly connected interrupt pins to get a proper update mask
     */
    template<bool inDebugMode, bool useInterrupts = true>
    static void newDataCycle() noexcept {
        if (auto op = getUpdateKind<useInterrupts>(); op == 0b00 || op == 0b01) {
            full32BitUpdate<inDebugMode>();
        } else if (op == 0b10) {
            lower16Update<inDebugMode>();
        } else {
            switch (DecodeDispatch::makeDynamicValue()) {
                case 0: doDispatch<inDebugMode, DecodeDispatch{0}>(); break;
                case 1: doDispatch<inDebugMode, DecodeDispatch{1}>(); break;
                case 2: doDispatch<inDebugMode, DecodeDispatch{2}>(); break;
                case 3: doDispatch<inDebugMode, DecodeDispatch{3}>(); break;
                case 4: doDispatch<inDebugMode, DecodeDispatch{4}>(); break;
                case 5: doDispatch<inDebugMode, DecodeDispatch{5}>(); break;
                case 6: doDispatch<inDebugMode, DecodeDispatch{6}>(); break;
                case 7: doDispatch<inDebugMode, DecodeDispatch{7}>(); break;
                default: break;
            }
        }
    }
    /**
     * @brief loads a cache line based on base transaction address and then bursts up to 16 bytes to the i960
     * @tparam inDebugMode Are we in debug mode?
     * @param line The cache line which we will be using for this transaction
     */
    template<bool inDebugMode>
    static inline void performCacheRead(const CacheLine& line) noexcept {
        SPI.beginTransaction(SPISettings(TargetBoard::runIOExpanderSPIInterfaceAt(), MSBFIRST, SPI_MODE0));
        digitalWrite<i960Pinout::GPIOSelect, LOW>();
        SPDR = generateWriteOpcode(DataLines);
        asm volatile ("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait
        // while we would normally write to OLAT, in this case we want to access gpio from what I can tell
        SPDR = static_cast<byte>(MCP23x17Registers::OLAT);
        asm volatile ("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait
        for (auto offset = line.getRawData() + getCacheOffsetEntry(); ; ++offset) {
            bool isLastRead = false;
            if (auto& a0 = *offset; a0.getWholeValue() != latchedDataOutput.getWholeValue()) {
                SPDR = a0.getLowerHalf();
                asm volatile ("nop");
                isLastRead = isBurstLast();
                while (!(SPSR & _BV(SPIF))); // wait
                SPDR = a0.getUpperHalf();
                asm volatile ("nop");
                latchedDataOutput.wholeValue_ = a0.getWholeValue();
                while (!(SPSR & _BV(SPIF))); // wait
            } else {
                isLastRead = isBurstLast();
            }
            DigitalPin<i960Pinout::Ready>::pulse();
            if (isLastRead) {
                // end the SPI transaction
                break;
            }
        }
        digitalWrite<i960Pinout::GPIOSelect, HIGH>();
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
        SPDR = generateReadOpcode(DataLines);
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
        SPDR = generateReadOpcode(DataLines);
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
        bool outcome = isBurstLast();
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
    template<bool inDebugMode>
    static inline void performCacheWrite(CacheLine& line) noexcept {
        SPI.beginTransaction(SPISettings(TargetBoard::runIOExpanderSPIInterfaceAt(), MSBFIRST, SPI_MODE0));
        byte count = 0;
        digitalWrite<i960Pinout::GPIOSelect, LOW>();
        SPDR = generateReadOpcode(DataLines);
        asm volatile ("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        SPDR = static_cast<byte>(MCP23x17Registers::GPIO);
        asm volatile ("nop");
        while (!(SPSR & _BV(SPIF))); // wait
        for (;;) {
            auto& currTransaction = transactions[count++];
            SPDR = 0;
            asm volatile ("nop");
            currTransaction.style = getStyle();
            bool isLast = isBurstLast();
            while (!(SPSR & _BV(SPIF))); // wait
            auto lower = SPDR;
            SPDR = 0;
            asm volatile ("nop");
            latchedDataInput_.bytes[0] = lower;
            while (!(SPSR & _BV(SPIF))); // wait
            latchedDataInput_.bytes[1] = SPDR;
            currTransaction.value = latchedDataInput_;
            DigitalPin<i960Pinout::Ready>::pulse();
            if (isLast) {
                // okay then perform the commit since we are done
                for (byte i = 0, j = getCacheOffsetEntry(); i < count; ++i, ++j) {
                    transactions[i].set(j, line);
                }
                break;
            }
        }
        digitalWrite<i960Pinout::GPIOSelect, HIGH>();
        SPI.endTransaction();
    }
    template<typename T, bool inDebugMode, bool isRead>
    static inline void performExternalDeviceOperation() noexcept {
        // this is a subset of actions, we just need to read the byte enable bits continuously and advance the address by two to get to the
        // next 16-bit word
        // don't increment everything just the lowest byte since we will never actually span 16 byte segments in a single burst transaction
        auto pageIndex = getPageIndex();
        for (byte pageOffset = getPageOffset(); ;pageOffset += 2) {
            if constexpr (isRead) {
                auto result = T::read(pageIndex,
                                      pageOffset,
                                      getStyle());
                if constexpr (inDebugMode) {
                    Serial.print(F("\tPage Index: 0x"));
                    Serial.println(getPageIndex(), HEX);
                    Serial.print(F("\tPage Offset: 0x"));
                    Serial.println(pageOffset, HEX);
                    Serial.print(F("\tRead Value: 0x"));
                    Serial.println(result, HEX);
                }
                setDataBits(result);
            } else {
                LoadStoreStyle currLSS = getStyle();
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
            }
            auto isLast = isBurstLast();
            DigitalPin<i960Pinout::Ready>::pulse();
            if (isLast) {
                return;
            }
        }
    }
public:
    /**
     * @brief Complete the process of setting up the processor interface by seeding the cached function pointers with valid addresses.
     */
private:
    template<bool inDebugMode>
    static void performCacheRead() noexcept {
        performCacheRead<inDebugMode>(theCache.getLine(address_));
    }

    template<bool inDebugMode>
    static void performCacheWrite() noexcept {
        performCacheWrite<inDebugMode>(theCache.getLine(address_));
    }

private:
    static inline SplitWord32 address_{0};
    static inline SplitWord16 latchedDataOutput {0};
    static inline byte dataLinesDirection_ = 0;
    static inline byte cacheOffsetEntry_ = 0;
    static inline bool initialized_ = false;
    static inline SplitWord16 latchedDataInput_ {0};
    static inline bool isReadOperation_ = false;
    static inline bool inIOSpace_ = false;
};

#endif //ARDUINO_IOEXPANDERS_H
