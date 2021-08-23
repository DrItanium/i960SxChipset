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

#ifndef ARDUINO_IOEXPANDERS_H
#define ARDUINO_IOEXPANDERS_H
#include <Arduino.h>
#include "Pinout.h"
#include "MemoryThing.h"

class ProcessorInterface {
public:
    static ProcessorInterface&
    getInterface() noexcept {
        static ProcessorInterface theInterface;
        return theInterface;
    }
    ProcessorInterface(const ProcessorInterface&) = delete;
    ProcessorInterface(ProcessorInterface&&) = delete;
    ProcessorInterface& operator=(const ProcessorInterface&) = delete;
    ProcessorInterface& operator=(ProcessorInterface&&) = delete;
private:
    ProcessorInterface() = default;
public:
    enum class IOExpanderAddress : byte {
        DataLines = 0b0000,
        Lower16Lines = 0b0010,
        Upper16Lines = 0b0100,
        MemoryCommitExtras = 0b0110,
        OtherDevice0 = 0b1000,
        OtherDevice1 = 0b1010,
        OtherDevice2 = 0b1100,
        OtherDevice3 = 0b1110,
    };
// layout of the extra memory commit expander
// PA0 - BurstAddress1 - input
// PA1 - BurstAddress2 - input
// PA2 - BurstAddress3 - input
// PA3 - BE0_ - input
// PA4 - BE1_ - input
// PA5 - HOLD  - output
// PA6 - HLDA  - input
// PA7 - _LOCK - output
// PB0-PB7 - Unused

    enum class ExtraGPIOExpanderPinout : decltype(A0) {
        BurstAddress1,
        BurstAddress2,
        BurstAddress3,
        ByteEnable0,
        ByteEnable1,
        HOLD,
        HLDA,
        LOCK_,
        // add support for upto 256 spi devices
        Unused0,
        Unused1,
        Unused2,
        Unused3,
        Unused4,
        Unused5,
        Unused6,
        Unused7,
        Count,
    };
    static_assert(static_cast<int>(ExtraGPIOExpanderPinout::Count) == 16);
public:
    void begin() noexcept;
    [[nodiscard]] constexpr Address getAddress() const noexcept { return address_.wholeValue_; }
    [[nodiscard]] uint16_t getDataBits() noexcept;
    void setDataBits(uint16_t value) noexcept;
    [[nodiscard]] constexpr auto getStyle() const noexcept { return static_cast<LoadStoreStyle>(lss_); }
    //[[nodiscard]] bool isWriteOperation() const noexcept;
    void setHOLDPin(bool value) noexcept;
    void setLOCKPin(bool value) noexcept;
    [[nodiscard]] constexpr auto getAlignedAddress() const noexcept {
        // a copy should actually be faster
        auto copy = upperMaskedAddress_;
        copy.bytes[0] &= 0xE0;
        return copy.wholeValue_;
    }
    [[nodiscard]] constexpr auto getCacheOffsetEntry() const noexcept { return cacheOffsetEntry_; }
    [[nodiscard]] constexpr auto isReadOperation() const noexcept { return isReadOperation_; }
public:
    MemoryThing* newDataCycle() noexcept;
    void burstNext() noexcept;

private:
    void updateOutputLatch() noexcept;
private:
    uint16_t dataLinesDirection_ = 0xFFFF;
    SplitWord32 upperMaskedAddress_;
    SplitWord32 address_;
    LoadStoreStyle lss_ = LoadStoreStyle::None;
    bool initialized_ = false;
    bool lockValue_ = true;
    bool holdValue_ = false;
    byte cacheOffsetEntry_ = 0;
    bool isReadOperation_ = false;
};
// 8 IOExpanders to a single enable line for SPI purposes
// 4 of them are reserved

#endif //ARDUINO_IOEXPANDERS_H
