//
// Created by jwscoggins on 4/11/21.
//

#include <Arduino.h>
#include <SPI.h>
#include "W25Q32JV.h"
#include "Pinout.h"

void
W25Q32JV::sendReadOperation(uint32_t address) {
    setSPIBusId(busId_);
    byte a = static_cast<byte>(address >> 16);
    byte b = static_cast<byte>(address >> 8);
    byte c = static_cast<byte>(address);
    HoldPinLow<i960Pinout::SPI_BUS_EN> transaction;
    SPI.transfer(static_cast<byte>(Opcodes::Read));
    SPI.transfer(a);
    SPI.transfer(b);
    SPI.transfer(c);

}
uint8_t
W25Q32JV::read8(uint32_t address) {
    sendReadOperation(address);
    return SPI.transfer(0x00);
}

uint16_t
W25Q32JV::read16(uint32_t address) {
    sendReadOperation(address);
    return SPI.transfer16(0x00);
}

void
W25Q32JV::write8(uint32_t, uint8_t) {
    // Do not allow writing to this flash module at runtime, it must be programmed offline
    // thus do nothing!
}
void
W25Q32JV::write16(uint32_t, uint16_t) {
    // Do not allow writing to this flash module at runtime, it must be programmed offline
    // thus do nothing!
}
