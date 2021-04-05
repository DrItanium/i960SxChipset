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
#ifndef ARDUINO_BOARDSPECIFIC_H
#define ARDUINO_BOARDSPECIFIC_H
enum class LoadStoreStyle : uint8_t {
    // based off of BE0,BE1 pins
    Load16 = 0b00,
    Upper8 = 0b01,
    Lower8 = 0b10,
    None = 0b11,
};
class Board {
public:
    Board() = default;
    virtual ~Board() = default;
    /// @todo migrate sketch specific things into the top level of this class (IOExpanders, etc)
    virtual void begin() noexcept = 0;
    virtual void loopBody() noexcept = 0;
    virtual void setupInterrupts() noexcept = 0;
    virtual uint16_t load(uint32_t address, LoadStoreStyle style) noexcept = 0;
    virtual void store(uint32_t address, uint16_t value, LoadStoreStyle style) noexcept = 0;
    virtual bool sdcardInstalled(uint8_t index = 0) const noexcept = 0;
};
#endif //ARDUINO_BOARDSPECIFIC_H
