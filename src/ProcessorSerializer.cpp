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

#include "ProcessorSerializer.h"
#include "Pinout.h"
//#include "PSRAMChip.h"
inline void
drainUDR1() noexcept {
    (void) UDR1;
    (void) UDR1;
}
SplitWord16
ProcessorInterface::getDataBits() noexcept {
#if 0
    if constexpr (TargetBoard::onAtmega1284p_Type1()) {
        return readGPIO16<ProcessorInterface::IOExpanderAddress::DataLines>();
    } else {
        // stub out
        return SplitWord16(0);
    }
#endif
    digitalWrite<i960Pinout::GPIO_CS0, LOW>();
    digitalWrite<i960Pinout::GPIO_CS1, LOW>();
    SPDR = generateReadOpcode(ProcessorInterface::IOExpanderAddress::DataLines);
    // use the 16-bit write capabilities of the MSPIM device
    UDR1 = generateReadOpcode(ProcessorInterface::IOExpanderAddress::DataLines);
    UDR1 = static_cast<byte>(MCP23x17Registers::GPIOB);
    while (!(SPSR & _BV(SPIF))); // wait
    SPDR = static_cast<byte>(MCP23x17Registers::GPIOA);
    while (!(UCSR1A & (1 << UDRE1)));
    // use the 16-bit write capabilities of the MSPIM device
    UDR1 = 0;
    while (!(SPSR & _BV(SPIF))); // wait
    SPDR = 0;
    asm volatile ("nop");
    while (!(UCSR1A & (1 << UDRE1)));
    while (!(SPSR & _BV(SPIF))); // wait
    auto lower = UDR1;
    auto upper = SPDR;
    digitalWrite<i960Pinout::GPIO_CS0, HIGH>();
    digitalWrite<i960Pinout::GPIO_CS1, HIGH>();
    SplitWord16 theWord{lower, upper};
    Serial.print(F("getDataBits: 0x"));
    Serial.println(theWord.getWholeValue(), HEX);
    return theWord;
}
auto
USART_Receive(byte value) noexcept {
    while (!(UCSR1A & (1 << UDRE1)));
    UDR1 = value;
    while (!(UCSR1A & (1 << RXC1)));
    return UDR1;
}
void
MixedTransmit(byte udrValue, byte spiValue) noexcept {
    while (!(UCSR1A & (1 << UDRE1)));
    UDR1 = udrValue;
    SPDR = spiValue;
    asm volatile ("nop");
    while (!(SPSR & _BV(SPIF))); // wait
    while (!(UCSR1A & (1 << RXC1)));
    (void)UDR1;
}
void
ProcessorInterface::setDataBits(uint16_t value) noexcept {
#if 0
    if constexpr (TargetBoard::onAtmega1284p_Type1()) {
        // the latch is preserved in between data line changes
        // okay we are still pointing as output values
        // check the latch and see if the output value is the same as what is latched
        if (latchedDataOutput != value) {
            latchedDataOutput = value;
            writeGPIO16<ProcessorInterface::IOExpanderAddress::DataLines>(latchedDataOutput);
        }
    } else {
       // do nothing
    }
#endif
    //Serial.print(F("setDataBits: 0x"));
    //Serial.println(value, HEX);
    //if (latchedDataOutput != value) {
    //    latchedDataOutput = value;
    SplitWord16 divisor(value);
    drainUDR1();
    auto opcode = generateWriteOpcode(ProcessorInterface::IOExpanderAddress::DataLines);
    digitalWrite<i960Pinout::GPIO_CS1, LOW>();
    digitalWrite<i960Pinout::GPIO_CS0, LOW>();
    MixedTransmit(opcode, opcode) ;
    MixedTransmit(static_cast<byte>(MCP23x17Registers::GPIOB), static_cast<byte>(MCP23x17Registers::GPIOA));
    MixedTransmit(divisor.bytes[0], divisor.bytes[1]);
    digitalWrite<i960Pinout::GPIO_CS1, HIGH>();
    digitalWrite<i960Pinout::GPIO_CS0, HIGH>();
}




void
ProcessorInterface::begin() noexcept {
    if (!initialized_) {
        initialized_ = true;
        pinMode(i960Pinout::GPIO_CS0, OUTPUT);
        pinMode(i960Pinout::GPIO_CS1, OUTPUT);
        digitalWrite<i960Pinout::GPIO_CS0, HIGH>();
        digitalWrite<i960Pinout::GPIO_CS1, HIGH>();
        // at bootup, the IOExpanders all respond to 0b000 because IOCON.HAEN is
        // disabled. We can send out a single IOCON.HAEN enable message and all
        // should receive it.
        // so do a begin operation on all chips (0b000)
        // set IOCON.HAEN on all chips
        drainUDR1();
        digitalWrite<i960Pinout::GPIO_CS0, LOW>();
        digitalWrite<i960Pinout::GPIO_CS1, LOW>();
        SPDR = generateWriteOpcode(ProcessorInterface::IOExpanderAddress::DataLines);
        UDR1 = generateWriteOpcode(ProcessorInterface::IOExpanderAddress::DataLines);
        UDR1 = static_cast<byte>(MCP23x17Registers::IOCON);
        asm volatile ("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait
        SPDR = static_cast<byte>(MCP23x17Registers::IOCON);
        while (!(UCSR1A & (1 << UDRE1)));
        UDR1 = 0b0000'1000;
        while (!(SPSR & _BV(SPIF))) ; // wait
        SPDR = 0b0000'1000;
        asm volatile("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait
        while (!(UCSR1A & (1 << UDRE1)));
        digitalWrite<i960Pinout::GPIO_CS0, HIGH>();
        digitalWrite<i960Pinout::GPIO_CS1, HIGH>();
        drainUDR1();
        // setup the direction registers for the data lines
        digitalWrite<i960Pinout::GPIO_CS0, LOW>();
        digitalWrite<i960Pinout::GPIO_CS1, LOW>();
        SPDR = generateWriteOpcode(ProcessorInterface::IOExpanderAddress::DataLines);
        // use the 16-bit write capabilities of the MSPIM device
        UDR1 = generateWriteOpcode(ProcessorInterface::IOExpanderAddress::DataLines);
        asm volatile ("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait
        while (!(UCSR1A & (1 << UDRE1)));
        SPDR = static_cast<byte>(MCP23x17Registers::IODIR);
        UDR1 = static_cast<byte>(MCP23x17Registers::IODIR);
        asm volatile ("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait
        while (!(UCSR1A & (1 << UDRE1)));
        // use the 16-bit write capabilities of the MSPIM device
        UDR1 = 0xFF;
        SPDR = 0xFF;
        while (!(SPSR & _BV(SPIF))) ; // wait
        while (!(UCSR1A & (1 << UDRE1)));
        UDR1 = 0xFF;
        SPDR = 0xFF;
        while (!(UCSR1A & (1 << UDRE1)));
        while (!(SPSR & _BV(SPIF))) ; // wait
        digitalWrite<i960Pinout::GPIO_CS0, HIGH>();
        digitalWrite<i960Pinout::GPIO_CS1, HIGH>();
        // setup the direction registers for the address lines
        digitalWrite<i960Pinout::GPIO_CS0, LOW>();
        digitalWrite<i960Pinout::GPIO_CS1, LOW>();
        SPDR = generateWriteOpcode(ProcessorInterface::IOExpanderAddress::Lower16Lines);
        // use the 16-bit write capabilities of the MSPIM device
        UDR1 = generateWriteOpcode(ProcessorInterface::IOExpanderAddress::Lower16Lines);
        asm volatile ("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait
        while (!(UCSR1A & (1 << UDRE1)));
        UDR1 = static_cast<byte>(MCP23x17Registers::IODIR);
        SPDR = static_cast<byte>(MCP23x17Registers::IODIR);
        while (!(UCSR1A & (1 << UDRE1)));
        while (!(SPSR & _BV(SPIF))) ; // wait
        // use the 16-bit write capabilities of the MSPIM device
        UDR1 = 0xFF;
        SPDR = 0xFF;
        while (!(SPSR & _BV(SPIF))) ; // wait
        while (!(UCSR1A & (1 << UDRE1)));
        UDR1 = 0xFF;
        SPDR = 0xFF;
        while (!(SPSR & _BV(SPIF))) ; // wait
        while (!(UCSR1A & (1 << UDRE1)));
        digitalWrite<i960Pinout::GPIO_CS0, HIGH>();
        digitalWrite<i960Pinout::GPIO_CS1, HIGH>();

        // setup the direction registers for the reset, interrupts, and element lines
        digitalWrite<i960Pinout::GPIO_CS0, LOW>();
        SPDR = generateWriteOpcode(ProcessorInterface::IOExpanderAddress::MemoryCommitExtras);
        asm volatile ("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait
        SPDR = static_cast<byte>(MCP23x17Registers::IODIR);
        asm volatile ("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait
        SPDR = 0b0010'0000; // the only thing that should be input is HLDA
        asm volatile ("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait
        // don't waste time chaning the direction of portb
        digitalWrite<i960Pinout::GPIO_CS0, HIGH>();

        // then setup the output latch on the
        updateControlSignals();
        SplitWord16 ldo(latchedDataOutput);

        digitalWrite<i960Pinout::GPIO_CS0, LOW>();
        digitalWrite<i960Pinout::GPIO_CS1, LOW>();
        SPDR = generateWriteOpcode(ProcessorInterface::IOExpanderAddress::DataLines);
        UDR1 = generateWriteOpcode(ProcessorInterface::IOExpanderAddress::DataLines);
        asm volatile ("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait
        while (!(UCSR1A & (1 << UDRE1)));
        UDR1 = static_cast<byte>(MCP23x17Registers::OLATB);
        SPDR = static_cast<byte>(MCP23x17Registers::OLATA);
        asm volatile ("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait
        while (!(UCSR1A & (1 << UDRE1)));
        // use the 16-bit write capabilities of the MSPIM device
        UDR1 = ldo.bytes[0];
        SPDR = ldo.bytes[1];
        asm volatile ("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait
        while (!(UCSR1A & (1 << UDRE1)));
        digitalWrite<i960Pinout::GPIO_CS0, HIGH>();
        digitalWrite<i960Pinout::GPIO_CS1, HIGH>();
    }
}
void
ProcessorInterface::updateControlSignals() noexcept {
    digitalWrite<i960Pinout::GPIO_CS0, LOW>();
    SPDR = generateWriteOpcode(ProcessorInterface::IOExpanderAddress::MemoryCommitExtras);
    asm volatile ("nop");
    while (!(SPSR & _BV(SPIF))) ; // wait
    SPDR = static_cast<byte>(MCP23x17Registers::GPIOA);
    asm volatile ("nop");
    while (!(SPSR & _BV(SPIF))) ; // wait
    SPDR = controlSignals_;
    asm volatile ("nop");
    while (!(SPSR & _BV(SPIF))) ; // wait
    digitalWrite<i960Pinout::GPIO_CS0, HIGH>();
}
void
ProcessorInterface::holdResetLine() noexcept {
    controlSignals_ &= ~(0b0000'0001);
    updateControlSignals();
}
void
ProcessorInterface::releaseResetLine() noexcept {
    controlSignals_ |= 0b0000'0001;
    updateControlSignals();
}

byte
ProcessorInterface::newDataCycle() noexcept {
    Serial.print(F("PREVIOUS ADDRESS = 0x"));
    Serial.println(address_.getWholeValue(), HEX);
    (void)UDR1;
    (void)UDR1;
    // read from both busses
    digitalWrite<i960Pinout::GPIO_CS0, LOW>();
    digitalWrite<i960Pinout::GPIO_CS1, LOW>();
    SPDR = generateReadOpcode(ProcessorInterface::IOExpanderAddress::Lower16Lines);
    UDR1 = generateReadOpcode(ProcessorInterface::IOExpanderAddress::Lower16Lines);
    asm volatile ("nop");
    while (!(SPSR & _BV(SPIF))) ; // wait
    while (!(UCSR1A & (1 << UDRE1)));
    (void)UDR1;
    (void)UDR1;
    UDR1 = static_cast<byte>(MCP23x17Registers::GPIO);
    SPDR = static_cast<byte>(MCP23x17Registers::GPIO);
    asm volatile ("nop");
    while (!(SPSR & _BV(SPIF))) ; // wait
    while (!(UCSR1A & (1 << UDRE1)));
    (void)UDR1;
    (void)UDR1;
    UDR1 = 0;
    SPDR = 0;
    asm volatile ("nop");
    while (!(SPSR & _BV(SPIF))) ; // wait
    while (!(UCSR1A & (1 << UDRE1)));
    auto b0 = UDR1;
    auto b2 = SPDR;
    UDR1 = 0;
    SPDR = 0;
    asm volatile ("nop");
    while (!(SPSR & _BV(SPIF))) ; // wait
    while (!(UCSR1A & (1 << UDRE1)));
    auto b1 = UDR1;
    auto b3 = SPDR;
    // use the 16-bit write capabilities of the MSPIM device
    digitalWrite<i960Pinout::GPIO_CS0, HIGH>();
    digitalWrite<i960Pinout::GPIO_CS1, HIGH>();
    Serial.print(F("b0: 0x"));
    Serial.println(b0, HEX);
    Serial.print(F("b1: 0x"));
    Serial.println(b1, HEX);
    Serial.print(F("b2: 0x"));
    Serial.println(b2, HEX);
    Serial.print(F("b3: 0x"));
    Serial.println(b3, HEX);
    Serial.print(F("addrWhole: 0x"));
    Serial.println(address_.getWholeValue(), HEX);
    address_.bytes[0] = b0;
    address_.bytes[1] = b1;
    address_.bytes[2] = b2;
    address_.bytes[3] = b3;
    isReadOperation_ = (address_.bytes[0] & 0b1) == 0;
    if (isReadOperation()) {
        setupDataLinesForRead();
    } else {
        setupDataLinesForWrite();
    }
    address_.bytes[0] &= (~0b0000'0001); // clear the least significant bit
    Serial.print(F("Address Request: 0x"));
    Serial.println(address_.getWholeValue(), HEX);
    Serial.println(isReadOperation() ? F("READ") : F("WRITE"));
    return address_.bytes[3];
#if 0
    if constexpr (TargetBoard::onAtmega1284p_Type1()) {
        constexpr auto Lower16Opcode = generateReadOpcode(ProcessorInterface::IOExpanderAddress::Lower16Lines);
        constexpr auto Upper16Opcode = generateReadOpcode(ProcessorInterface::IOExpanderAddress::Upper16Lines);
        constexpr auto GPIOOpcode = static_cast<byte>(MCP23x17Registers::GPIO);
        // we want to overlay actions as much as possible during spi transfers, there are blocks of waiting for a transfer to take place
        // where we can insert operations to take place that would otherwise be waiting
        digitalWrite<i960Pinout::GPIOSelect, LOW>();
        SPDR = Lower16Opcode;
        /*
         * The following NOP introduces a small delay that can prevent the wait
         * loop form iterating when running at the maximum speed. This gives
         * about 10% more speed, even if it seems counter-intuitive. At lower
         * speeds it is unnoticed.
         */
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
            cacheOffsetEntry_ = (lowest >> 1) & 0b0000'0111; // we want to make this quick to increment
            address_.bytes[0] = lowest;
        }
        while (!(SPSR & _BV(SPIF))); // wait
        auto lower = SPDR;
        DigitalPin<i960Pinout::GPIOSelect>::pulse<HIGH>(); // pulse high
        SPDR = Upper16Opcode;
        asm volatile("nop");
        {
            address_.bytes[1] = lower;
            // interleave this operation in, can't get more complex than this
        }
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
        address_.bytes[3] = highest;
        if (isReadOperation()) {
            setupDataLinesForRead();
        } else {
            setupDataLinesForWrite();
        }
        return highest;
    } else {
        return 0;
    }
#endif
}
void
ProcessorInterface::setupDataLinesForWrite() noexcept {
    digitalWrite<i960Pinout::GPIO_CS0, LOW>();
    digitalWrite<i960Pinout::GPIO_CS1, LOW>();
    SPDR = generateWriteOpcode(ProcessorInterface::IOExpanderAddress::DataLines);
    // use the 16-bit write capabilities of the MSPIM device
    UDR1 = generateWriteOpcode(ProcessorInterface::IOExpanderAddress::DataLines);
    UDR1 = static_cast<byte>(MCP23x17Registers::IODIR);
    asm volatile ("nop");
    while (!(SPSR & _BV(SPIF))); // wait
    SPDR = static_cast<byte>(MCP23x17Registers::IODIR);
    while (!(UCSR1A & (1 << UDRE1)));
    // use the 16-bit write capabilities of the MSPIM device
    UDR1 = 0xFF;
    UDR1 = 0xFF;
    asm volatile ("nop");
    while (!(SPSR & _BV(SPIF))); // wait
    SPDR = 0xFF;
    while (!(UCSR1A & (1 << UDRE1)));
    while (!(SPSR & _BV(SPIF))); // wait
    SPDR = 0xFF;
    asm volatile ("nop");
    while (!(SPSR & _BV(SPIF))); // wait
    digitalWrite<i960Pinout::GPIO_CS0, HIGH>();
    digitalWrite<i960Pinout::GPIO_CS1, HIGH>();
}
void
ProcessorInterface::setupDataLinesForRead() noexcept {
    /// @todo eliminate the extra byte of transmission because of the separate io expanders
    digitalWrite<i960Pinout::GPIO_CS1, LOW>();
    UDR1 = generateWriteOpcode(ProcessorInterface::IOExpanderAddress::DataLines);
    UDR1 = static_cast<byte>(MCP23x17Registers::IODIR);
    while (!(UCSR1A & (1 << UDRE1)));
    UDR1 = 0;
    UDR1 = 0;
    while (!(UCSR1A & (1 << UDRE1)));
    digitalWrite<i960Pinout::GPIO_CS1, HIGH>();

    digitalWrite<i960Pinout::GPIO_CS0, LOW>();
    SPDR = generateWriteOpcode(ProcessorInterface::IOExpanderAddress::DataLines);
    // use the 16-bit write capabilities of the MSPIM device
    asm volatile ("nop");
    while (!(SPSR & _BV(SPIF))); // wait
    SPDR = static_cast<byte>(MCP23x17Registers::IODIR);
    asm volatile ("nop");
    while (!(SPSR & _BV(SPIF))); // wait
    SPDR = 0;
    asm volatile ("nop");
    while (!(SPSR & _BV(SPIF))); // wait
    SPDR = 0;
    asm volatile ("nop");
    while (!(SPSR & _BV(SPIF))); // wait
    digitalWrite<i960Pinout::GPIO_CS0, HIGH>();
}

void
ProcessorInterface::triggerInt0() noexcept {
#ifdef CHIPSET_TYPE1
    pulse<i960Pinout::Int0_>();
#elif defined(CHIPSET_TYPE3)
    /// @todo implement this
#endif
}


void
ProcessorInterface::ioExpanderWriteTest() noexcept {
    setupDataLinesForRead();
    for (uint32_t i = 0; i < 0x10000; ++i) {
        setDataBits(i);
    }
}