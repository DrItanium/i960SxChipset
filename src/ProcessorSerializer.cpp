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
auto
USART_Receive(byte value) noexcept {
    while (!(UCSR1A & (1 << UDRE1)));
    UDR1 = value;
    asm volatile ("nop");
    while (!(UCSR1A & (1 << RXC1)));
    return UDR1;
}
inline void
MixedTransmit(byte udrValue, byte spiValue) noexcept {
    while (!(UCSR1A & (1 << UDRE1)));
    UDR1 = udrValue;
    SPDR = spiValue;
    asm volatile ("nop");
    while (!(SPSR & _BV(SPIF))); // wait
    while (!(UCSR1A & (1 << RXC1)));
    (void)UDR1;
}
inline SplitWord16
MixedTransfer(byte udrValue, byte spiValue) noexcept {
    while (!(UCSR1A & (1 << UDRE1)));
    UDR1 = udrValue;
    SPDR = spiValue;
    asm volatile ("nop");
    while (!(SPSR & _BV(SPIF))); // wait
    auto sResult = SPDR;
    while (!(UCSR1A & (1 << RXC1)));
    volatile auto uResult = UDR1;
    Serial.println(F("{"));
    Serial.print(F("\tuR: 0x"));
    Serial.println(uResult, HEX);
    Serial.print(F("\tsR: 0x"));
    Serial.println(sResult, HEX);
    Serial.println(F("}"));
    return SplitWord16{uResult, sResult};
}

SplitWord16
ProcessorInterface::getDataBits() noexcept {
    static constexpr auto readOpcode = generateReadOpcode(ProcessorInterface::IOExpanderAddress::DataLines);
    digitalWrite<i960Pinout::GPIO_CS0, LOW>();
    digitalWrite<i960Pinout::GPIO_CS1, LOW>();
    MixedTransmit(readOpcode, readOpcode);
    MixedTransmit(static_cast<byte>(MCP23x17Registers::GPIOB),
                  static_cast<byte>(MCP23x17Registers::GPIOA));
    auto theWord = MixedTransfer(0, 0);
    digitalWrite<i960Pinout::GPIO_CS0, HIGH>();
    digitalWrite<i960Pinout::GPIO_CS1, HIGH>();
    //Serial.print(F("getDataBits: 0x"));
    //Serial.println(theWord.getWholeValue(), HEX);
    return theWord;
}
void
ProcessorInterface::setDataBits(uint16_t value) noexcept {
#if 0
    Serial.print(F("setDataBits: 0x"));
    Serial.println(value, HEX);
#endif
    /// @todo cache the result eventually
    SplitWord16 divisor(value);
    auto opcode = generateWriteOpcode(ProcessorInterface::IOExpanderAddress::DataLines);
    digitalWrite<i960Pinout::GPIO_CS1, LOW>();
    digitalWrite<i960Pinout::GPIO_CS0, LOW>();
    MixedTransmit(opcode, opcode);
    MixedTransmit(static_cast<byte>(MCP23x17Registers::GPIO),
                  static_cast<byte>(MCP23x17Registers::GPIO));
    MixedTransmit(divisor.bytes[0], divisor.bytes[1]);
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
        static constexpr auto dataLinesWriteOpcode = generateWriteOpcode(ProcessorInterface::IOExpanderAddress::DataLines);
        static constexpr auto addressLinesWriteOpcode = generateWriteOpcode(ProcessorInterface::IOExpanderAddress::Lower16Lines);
        static constexpr auto ioconRegister = static_cast<byte>(MCP23x17Registers::IOCON);
        static constexpr auto iodirRegister = static_cast<byte>(MCP23x17Registers::IODIR);
        static constexpr auto olatARegister = static_cast<byte>(MCP23x17Registers::OLATA);
        static constexpr auto olatBRegister = static_cast<byte>(MCP23x17Registers::OLATB);
        digitalWrite<i960Pinout::GPIO_CS0, LOW>();
        digitalWrite<i960Pinout::GPIO_CS1, LOW>();
        MixedTransmit(dataLinesWriteOpcode, dataLinesWriteOpcode);
        MixedTransmit(ioconRegister, ioconRegister);
        MixedTransmit(0b0000'1000,
                      0b0000'1000);
        digitalWrite<i960Pinout::GPIO_CS0, HIGH>();
        digitalWrite<i960Pinout::GPIO_CS1, HIGH>();
        // setup the direction registers for the data lines
        setupDataLinesForWrite();
        // setup the direction registers for the address lines
        digitalWrite<i960Pinout::GPIO_CS0, LOW>();
        digitalWrite<i960Pinout::GPIO_CS1, LOW>();
        MixedTransmit(addressLinesWriteOpcode,
                      addressLinesWriteOpcode);
        MixedTransmit(iodirRegister,
                      iodirRegister);
        MixedTransmit(0xFF,
                      0xFF);
        MixedTransmit(0xFF,
                      0xFF);
        digitalWrite<i960Pinout::GPIO_CS0, HIGH>();
        digitalWrite<i960Pinout::GPIO_CS1, HIGH>();
        digitalWrite<i960Pinout::GPIO_CS1, LOW>();
        (void)USART_Receive(generateReadOpcode(ProcessorInterface::IOExpanderAddress::Lower16Lines));
        (void)USART_Receive(static_cast<byte>(MCP23x17Registers::IPOL));
        auto lower = USART_Receive(0);
        auto upper = USART_Receive(0);
        digitalWrite<i960Pinout::GPIO_CS1, HIGH>();
        Serial.print(F("IPOL VALUE: 0x"));
        Serial.print(upper, HEX);
        Serial.println(lower, HEX);

        // setup the direction registers for the reset, interrupts, and element lines
        digitalWrite<i960Pinout::GPIO_CS0, LOW>();
        SPDR = generateWriteOpcode(ProcessorInterface::IOExpanderAddress::MemoryCommitExtras);
        asm volatile ("nop");
        while (!(SPSR & _BV(SPIF))) ; // wait
        SPDR = iodirRegister;
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
        MixedTransmit(dataLinesWriteOpcode,
                      dataLinesWriteOpcode);
        MixedTransmit(olatBRegister,
                      olatARegister);
        MixedTransmit(ldo.bytes[0],
                      ldo.bytes[1]);
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
    Serial.println(F("NEW DATA CYCLE {"));
    //Serial.print(F("PREVIOUS ADDRESS = 0x"));
    //Serial.println(address_.getWholeValue(), HEX);
    static constexpr auto addressLinesOpcode = generateReadOpcode(ProcessorInterface::IOExpanderAddress::Lower16Lines);
    static constexpr auto gpioRegister = static_cast<byte>(MCP23x17Registers::GPIO);
    // read from both busses
    Serial.println(F("Read Address Lines {"));
    digitalWrite<i960Pinout::GPIO_CS0, LOW>();
    digitalWrite<i960Pinout::GPIO_CS1, LOW>();
    (void)MixedTransfer(addressLinesOpcode,
                  addressLinesOpcode);
    (void)MixedTransfer(gpioRegister,
                  gpioRegister);
    auto p0 = MixedTransfer(0,
                            0);
    auto p1 = MixedTransfer(0,
                            0);
    // use the 16-bit write capabilities of the MSPIM device
    digitalWrite<i960Pinout::GPIO_CS0, HIGH>();
    digitalWrite<i960Pinout::GPIO_CS1, HIGH>();
    Serial.println(F("}"));
    auto b0 = p0.bytes[0];
    auto b1 = p1.bytes[0];
    auto b2 = p0.bytes[1];
    auto b3 = p1.bytes[1];
#if 0
    Serial.print(F("b0: 0x"));
    Serial.println(b0, HEX);
    Serial.print(F("b1: 0x"));
    Serial.println(b1, HEX);
    Serial.print(F("b2: 0x"));
    Serial.println(b2, HEX);
    Serial.print(F("b3: 0x"));
    Serial.println(b3, HEX);
#endif
    address_.bytes[0] = b0;
    address_.bytes[1] = b1;
    address_.bytes[2] = b2;
    address_.bytes[3] = b3;
    isReadOperation_ = (address_.bytes[0] & 0b1) == 0;
    Serial.println(F("Setup data lines {"));
    if (isReadOperation()) {
        setupDataLinesForRead();
    } else {
        setupDataLinesForWrite();
    }
    Serial.println(F("}"));
    address_.bytes[0] &= (~0b0000'0001); // clear the least significant bit
    Serial.print(F("Address Request: 0x"));
    Serial.print(address_.getWholeValue(), HEX);
    Serial.println(isReadOperation() ? F(": READ") : F(": WRITE"));
    Serial.println(F("}"));
    return address_.bytes[3];
}
void
ProcessorInterface::setupDataLinesForWrite() noexcept {
    auto opcode = generateWriteOpcode(ProcessorInterface::IOExpanderAddress::DataLines);
    auto target = static_cast<byte>(MCP23x17Registers::IODIR);
    digitalWrite<i960Pinout::GPIO_CS1, LOW>();
    digitalWrite<i960Pinout::GPIO_CS0, LOW>();
    MixedTransmit(opcode, opcode);
    MixedTransmit(target, target);
    MixedTransmit(0xFF, 0xFF);
    MixedTransmit(0xFF, 0xFF);
    digitalWrite<i960Pinout::GPIO_CS1, HIGH>();
    digitalWrite<i960Pinout::GPIO_CS0, HIGH>();
}
void
ProcessorInterface::setupDataLinesForRead() noexcept {
    /// @todo eliminate the extra byte of transmission because of the separate io expanders
    auto opcode = generateWriteOpcode(ProcessorInterface::IOExpanderAddress::DataLines);
    auto target = static_cast<byte>(MCP23x17Registers::IODIR);
    digitalWrite<i960Pinout::GPIO_CS1, LOW>();
    digitalWrite<i960Pinout::GPIO_CS0, LOW>();
    MixedTransmit(opcode, opcode);
    MixedTransmit(target, target);
    MixedTransmit(0, 0);
    MixedTransmit(0, 0);
    digitalWrite<i960Pinout::GPIO_CS1, HIGH>();
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