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

#include <Arduino.h>
#include <SPI.h>
#include "SRAM_23LC1024.h"
#include "Pinout.h"
#include "SPIBus.h"
uint8_t
SRAM_23LC1024::read8(uint32_t address) {
    setSPIBusId(busId_);
    byte a = static_cast<byte>(address >> 16);
    byte b = static_cast<byte>(address >> 8);
    byte c = static_cast<byte>(address);
    HoldPinLow<i960Pinout::SPI_BUS_EN> transaction;
    SPI.transfer(static_cast<byte>(Opcodes::Read));
    SPI.transfer(a);
    SPI.transfer(b);
    SPI.transfer(c);
    return SPI.transfer(0x00);
}

uint16_t
SRAM_23LC1024::read16(uint32_t address) {
    setSPIBusId(busId_);
    byte a = static_cast<byte>(address >> 16);
    byte b = static_cast<byte>(address >> 8);
    byte c = static_cast<byte>(address);
    HoldPinLow<i960Pinout::SPI_BUS_EN> transaction;
    SPI.transfer(static_cast<byte>(Opcodes::Read));
    SPI.transfer(a);
    SPI.transfer(b);
    SPI.transfer(c);
    return SPI.transfer16(0x00);
}

void
SRAM_23LC1024::write8(uint32_t address, uint8_t value) {
    setSPIBusId(busId_);
    byte a = static_cast<byte>(address >> 16);
    byte b = static_cast<byte>(address >> 8);
    byte c = static_cast<byte>(address);
    HoldPinLow<i960Pinout::SPI_BUS_EN> transaction;
    SPI.transfer(static_cast<byte>(Opcodes::Write));
    SPI.transfer(a);
    SPI.transfer(b);
    SPI.transfer(c);
    SPI.transfer(value);
}
void
SRAM_23LC1024::write16(uint32_t address, uint16_t value) {
    setSPIBusId(busId_);
    byte a = static_cast<byte>(address >> 16);
    byte b = static_cast<byte>(address >> 8);
    byte c = static_cast<byte>(address);
    HoldPinLow<i960Pinout::SPI_BUS_EN> transaction;
    SPI.transfer(static_cast<byte>(Opcodes::Write));
    SPI.transfer(a);
    SPI.transfer(b);
    SPI.transfer(c);
    SPI.transfer16(value);
}
