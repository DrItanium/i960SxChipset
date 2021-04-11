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

#ifndef ARDUINO_PSRAM64H_H
#define ARDUINO_PSRAM64H_H
#include "RAM.h"
/**
 * @brief Represents a single Espressif PSRAM64H device
 */
class PSRAM64H : public RAM {
public:
    enum class Opcodes : uint8_t {
        Read = 0x03,
        FastRead = 0x0B,
        FastReadQuad = 0xEB,
        Write = 0x02,
        QuadWrite = 0x38,
        EnterQuadMode = 0x35,
        ExitQuadMode = 0xF5,
        ResetEnable = 0x66,
        Reset = 0x99,
        SetBurstLength = 0xC0,
        ReadID = 0x9F,
    };
    static constexpr uint32_t Size = 8 * static_cast<uint32_t>(1024) * static_cast<uint32_t>(1024);
public:
    constexpr PSRAM64H(uint32_t startAddress, SPIBusDevice id) : RAM(startAddress, Size), busId_(id) { }
    ~PSRAM64H() override = default;
    constexpr auto getBusID() const noexcept { return busId_; }
protected:
    uint8_t read8(uint32_t address) override;
    uint16_t read16(uint32_t address) override;
    void write8(uint32_t address, uint8_t value) override;
    void write16(uint32_t address, uint16_t value) override;
private:
    SPIBusDevice busId_;
};

#endif //ARDUINO_PSRAM64H_H
