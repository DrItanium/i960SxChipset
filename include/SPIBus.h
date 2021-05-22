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

#ifndef ARDUINO_SPIBUS_H
#define ARDUINO_SPIBUS_H
#include <Arduino.h>
// The upper eight lines of the Extra GPIO expander is used as an SPI address selector
// This provides up to 256 devices to be present on the SPI address bus and controlable through the single SPI_BUS_EN pin from the 1284p
// The "SPI Address" is used to select which CS line to pull low when ~SPI_BUS_EN is pulled low by the MCU. It requires some extra logic to
// use but is stupid simple to layout physically. These IDs have _no_ direct correlation to the memory map exposed to the i960 itself.
// This is only used internally. The idea is to use the SDCard to define the memory mapping bound to a given SPIBusDevice.

/**
 * @brief Common interface to the SPIBus
 */
class SPIBus {
public:
    void setup() noexcept;
    void setId(byte id) noexcept;
    [[nodiscard]] byte getId() const noexcept;
private:
    byte _id = 0;
    bool _initialized = false;
};
#endif //ARDUINO_SPIBUS_H
