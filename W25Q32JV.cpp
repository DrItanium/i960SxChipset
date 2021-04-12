//
// Created by jwscoggins on 4/11/21.
//

#include <Arduino.h>
#include <SPI.h>
#include "W25Q32JV.h"
#include "Pinout.h"

uint8_t
W25Q32JV::read8(uint32_t address) {
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
W25Q32JV::read16(uint32_t address) {
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
W25Q32JV::write8(uint32_t address, uint8_t value) {
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
W25Q32JV::write16(uint32_t address, uint16_t value) {
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
