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

#ifndef ARDUINO_DEVICE_H
#define ARDUINO_DEVICE_H
/**
 * @brief Describes an arbitrary device of an arbitrary size that is mapped into memory at a given location
 */
class Device {
public:
    constexpr Device(uint32_t start, uint32_t len) noexcept : startAddress_(start), length_(len) {}
    virtual ~Device() = default;
    constexpr auto getStartAddress() const noexcept { return startAddress_; }
    constexpr auto getLength() const noexcept { return length_; }
    constexpr auto getEndAddress() const noexcept { return length_ + startAddress_; }
    constexpr bool mapsToGivenRange(uint32_t address) const noexcept {
        return (address >= startAddress_) && (getEndAddress() > address);
    }
    virtual uint16_t read(uint32_t address, LoadStoreStyle style) noexcept = 0;
    virtual void write(uint32_t address, uint16_t value, LoadStoreStyle style) noexcept = 0;
private:
    uint32_t startAddress_ = 0;
    uint32_t length_ = 0;
};
#endif //ARDUINO_DEVICE_H
