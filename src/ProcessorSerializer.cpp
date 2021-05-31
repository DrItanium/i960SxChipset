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
#include "MCUPlatform.h"

Address
ProcessorInterface::getAddress() noexcept {
#ifdef ARDUINO_AVR_MEGA2560
        // the address is made up of several ports on the mega2560
        // PORTA - x,1,2,3,4,5,6,7
        // PORTC - 8:15
        // PORTL - 16:23
        // PORTH - 24,25,x,x,28,x,x,x
        // PORTD - x,x,26,27,x,x,x,x
        // PORTB - x,x,x,x,x,29,30,31
        auto lowest = static_cast<Address>(PORTA & 0b1111'1110);
        auto lower = static_cast<Address>(PORTC) << 8;
        auto higher = static_cast<Address>(PORTL) << 16;
        auto highest = static_cast<Address>((PORTH & 0b0001'0011) | (PORTD & 0b0000'1100) | (PORTB & 0b1110'0000) ) << 24;
        return lowest | lower | higher | highest;
#else
        auto lower16Addr = static_cast<Address>(lower16_.readGPIOs());
        auto upper16Addr = static_cast<Address>(upper16_.readGPIOs()) << 16;
        return lower16Addr | upper16Addr;
#endif
}
void
ProcessorInterface::setDataLinesDirection(uint16_t direction) {
#ifdef ARDUINO_AVR_MEGA2560
    // set the direction registers
    DDRF = static_cast<uint8_t>(direction & 0xFF);
    DDRK = static_cast<uint8_t>((direction & 0xFF00) >> 8);
#else
    dataLines_.writeGPIOsDirection(direction);
#endif
}
uint16_t
ProcessorInterface::getDataBits() noexcept {
    setDataLinesDirection(0xFFFF);
#ifdef ARDUINO_AVR_MEGA2560
    auto lower = static_cast<uint16_t>(PORTF);
    auto upper = static_cast<uint16_t>(PORTK) << 8;
    return lower | upper;
#else
    return static_cast<uint16_t>(dataLines_.readGPIOs());
#endif
}

void
ProcessorInterface::setDataBits(uint16_t value) noexcept {
    setDataLinesDirection(0);
#ifdef ARDUINO_AVR_MEGA2560
#else
    dataLines_.writeGPIOs(value);
#endif
}



uint8_t
ProcessorInterface::getByteEnableBits() noexcept {
    return (extra_.readGPIOs() & 0b11000) >> 3;
}

decltype(LOW)
ProcessorInterface::getByteEnable0() noexcept {
    return (getByteEnableBits() & 1) == 0 ? LOW : HIGH;
}
decltype(LOW)
ProcessorInterface::getByteEnable1() noexcept {
    return (getByteEnableBits() & 0b10) == 0 ? LOW : HIGH;
}

uint8_t
ProcessorInterface::getBurstAddressBits() noexcept {
    auto gpios = extra_.readGPIOs();
    return (gpios & 0b111) << 1;
}

Address
ProcessorInterface::getBurstAddress(Address base) noexcept {
    return getBurstAddress(base, static_cast<Address>(getBurstAddressBits()));
}
Address
ProcessorInterface::getBurstAddress() noexcept {
    return getBurstAddress(getAddress());
}

bool
ProcessorInterface::isReadOperation() const noexcept {
    return DigitalPin<i960Pinout::W_R_>::isAsserted();
}
bool
ProcessorInterface::isWriteOperation() const noexcept {
    return DigitalPin<i960Pinout::W_R_>::isDeasserted();
}

void
ProcessorInterface::setHOLDPin(decltype(LOW) value) noexcept {
    digitalWrite(static_cast<int>(ExtraGPIOExpanderPinout::HOLD), value, extra_);
}
void
ProcessorInterface::setLOCKPin(decltype(LOW) value) noexcept {
    digitalWrite(static_cast<int>(ExtraGPIOExpanderPinout::LOCK_), value, extra_);
}

void
ProcessorInterface::begin() noexcept {
    if (!initialized_) {
        initialized_ = true;
#ifdef ARDUINO_AVR_MEGA2560
        DDRF = 0xFF;
        DDRK = 0xFF;

#else
        // at bootup, the IOExpanders all respond to 0b000 because IOCON.HAEN is
        // disabled. We can send out a single IOCON.HAEN enable message and all
        // should receive it.
        // so do a begin operation on all chips (0b000)
        dataLines_.begin();
        // set IOCON.HAEN on all chips
        dataLines_.enableHardwareAddressPins();
        // now we have to refresh our on mcu flags for each io expander
        lower16_.refreshIOCon();
        upper16_.refreshIOCon();
        extra_.refreshIOCon();
        // now all devices tied to this ~CS pin have separate addresses
        // make each of these inputs
        lower16_.writeGPIOsDirection(0xFFFF);
        upper16_.writeGPIOsDirection(0xFFFF);
        dataLines_.writeGPIOsDirection(0xFFFF);
        // set lower eight to inputs and upper eight to outputs
        extra_.writeGPIOsDirection(0x00FF);
        // then indirectly mark the outputs
        pinMode(static_cast<int>(ExtraGPIOExpanderPinout::LOCK_), OUTPUT, extra_);
        pinMode(static_cast<int>(ExtraGPIOExpanderPinout::HOLD), OUTPUT, extra_);
#endif
    }
}
ProcessorInterface::LoadStoreStyle
ProcessorInterface::getStyle() noexcept {
    return static_cast<LoadStoreStyle>(getByteEnableBits());
}
bool
ProcessorInterface::failTriggered() const noexcept {
#ifdef ARDUINO_AVR_MEGA2560
    // read from  PORTG and mask out the bits for the pattern we are looking for
    // fail is defined as: (and (not BLAST_) (and BE0_ BE1_))
    // This translates to 0b011
    return (PORTG & 0b111) == 0b011;
#else
    return DigitalPin<i960Pinout::FAIL>::isAsserted();
#endif
}
