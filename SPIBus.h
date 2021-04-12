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

// Physically, the 8-bit address is broken up into a 2:3:3 design as follows:
// [0b00'000'000, 0b00'111'111] : "Onboard" Devices
// [0b01'000'000, 0b01'111'111] : "More Onboard/Optional" Devices
// [0b10'000'000, 0b10'111'111] : "Even More Onboard/Optional" Devices
// [0b11'000'000, 0b11'111'111] : User Devices [Direct access from the i960]
enum class SPIBusDevice : uint16_t {
    // WINBOND 4Megabit Flash
    Flash0,
    Flash1 ,
    Flash2,
    Flash3,
    Flash4,
    Flash5,
    Flash6,
    Flash7,
    // WINBOND 4Megabit Flash
    Flash8,
    Flash9 ,
    Flash10,
    Flash11,
    Flash12,
    Flash13,
    Flash14,
    Flash15,
    // WINBOND 4Megabit Flash
    Flash16,
    Flash17 ,
    Flash18,
    Flash19,
    Flash20,
    Flash21,
    Flash22,
    Flash23,
    // ESPRESSIF PSRAM64H (8Megabyte/64-megabit)
    PSRAM0,
    PSRAM1,
    PSRAM2,
    PSRAM3,
    PSRAM4,
    PSRAM5,
    PSRAM6,
    PSRAM7,
    // ESPRESSIF PSRAM64H (8Megabyte/64-megabit)
    PSRAM8,
    PSRAM9,
    PSRAM10,
    PSRAM11,
    PSRAM12,
    PSRAM13,
    PSRAM14,
    PSRAM15,
    // ESPRESSIF PSRAM64H (8Megabyte/64-megabit)
    PSRAM16,
    PSRAM17,
    PSRAM18,
    PSRAM19,
    PSRAM20,
    PSRAM21,
    PSRAM22,
    PSRAM23,
    //
    GPIO128_0,
    GPIO128_1,
    GPIO128_2,
    GPIO128_3,
    GPIO128_4,
    GPIO128_5,
    GPIO128_6,
    GPIO128_7,
    //
    ADC_0,
    ADC_1,
    ADC_2,
    ADC_3,
    ADC_4,
    ADC_5,
    ADC_6,
    ADC_7,

    Count
};
static_assert (static_cast<int>(SPIBusDevice::Count) <= 256);
/**
 * @brief Describes a block of RAM of an arbitrary size that is mapped into memory at a given location
 */

void setSPIBusId(SPIBusDevice id) noexcept;
SPIBusDevice getCurrentSPIBusDevice() noexcept;
#endif //ARDUINO_SPIBUS_H
