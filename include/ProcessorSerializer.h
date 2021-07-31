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
        DataLines = 0b000,
        Lower16Lines,
        Upper16Lines,
        MemoryCommitExtras,
        OtherDevice0,
        OtherDevice1,
        OtherDevice2,
        OtherDevice3,
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
    [[nodiscard]] constexpr Address getAddress() const noexcept { return address_; }
    [[nodiscard]] uint16_t getDataBits() noexcept;
#ifdef QUERY_BLAST
    [[nodiscard]] constexpr auto isBurstLast() const noexcept { return blastAsserted_; }
#endif
    void setDataBits(uint16_t value) noexcept;
    [[nodiscard]] constexpr auto getStyle() const noexcept { return static_cast<LoadStoreStyle>(lss_); }
    //[[nodiscard]] bool isWriteOperation() const noexcept;
    void setHOLDPin(bool value) noexcept;
    void setLOCKPin(bool value) noexcept;
    [[nodiscard]] constexpr auto getAlignedAddress() const noexcept { return upperMaskedAddress_; }
    [[nodiscard]] constexpr auto getBurstAddressBits() const noexcept { return burstAddressBits_; }
public:
    void setPortZDirectionRegister(byte value) noexcept;
    byte getPortZDirectionRegister() noexcept;
    void setPortZPolarityRegister(byte value) noexcept;
    byte getPortZPolarityRegister() noexcept;
    void setPortZPullupResistorRegister(byte value) noexcept;
    byte getPortZPullupResistorRegister() noexcept;
    byte readPortZGPIORegister() noexcept;
    void writePortZGPIORegister(byte value) noexcept;

    void newDataCycle() noexcept;
    void updateDataCycle() noexcept;
    //[[nodiscard]] constexpr auto getOpcode() const noexcept { return opcode_; }
private:
    void updateOutputLatch() noexcept;
private:
    uint16_t dataLinesDirection_ = 0xFFFF;
    Address upperMaskedAddress_ = 0;
    Address address_ = 0;
    LoadStoreStyle lss_ = LoadStoreStyle::None;
    bool initialized_ = false;
    bool lockValue_ = true;
    bool holdValue_ = false;
    byte burstAddressBits_ = 0;
    //TransactionDescription opcode_ = TransactionDescription::None;
#ifdef QUERY_BLAST
    bool blastAsserted_ = false;
#endif
};

// 8 IOExpanders to a single enable line for SPI purposes
// 4 of them are reserved

#endif //ARDUINO_IOEXPANDERS_H
