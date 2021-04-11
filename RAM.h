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

#ifndef ARDUINO_RAM_H
#define ARDUINO_RAM_H

class RAM : public Device{
public:
    constexpr RAM(uint32_t start, uint32_t ramSize) : Device(start, ramSize) { }
    ~RAM() override = default;
    uint16_t read(uint32_t address, LoadStoreStyle style) noexcept override {
        switch (style) {
            case LoadStoreStyle::Full16:
                return read16(address);
            case LoadStoreStyle::Lower8:
                return read8(address);
            case LoadStoreStyle::Upper8:
                return read8(address+1);
            default:
                return 0;
        }
    }
    void write (uint32_t address, uint16_t value, LoadStoreStyle style) noexcept override {
        switch (style) {
            case LoadStoreStyle::Full16:
                write16(address, value);
                break;
            case LoadStoreStyle::Lower8:
                write8(address, static_cast<uint8_t>(value));
                break;
            case LoadStoreStyle::Upper8:
                write8(address + 1, static_cast<uint8_t>(value >> 8));
                break;
            default:
                // do nothing
                break;
        }
    }
protected:
    virtual uint8_t read8(uint32_t address) = 0;
    virtual uint16_t read16(uint32_t address) = 0;
    virtual void write8(uint32_t address, uint8_t value) = 0;
    virtual void write16(uint32_t address, uint16_t value) = 0;
};
#endif //ARDUINO_RAM_H
