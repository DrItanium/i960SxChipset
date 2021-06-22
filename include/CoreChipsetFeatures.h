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
//
// Created by jwscoggins on 6/21/21.
//

#ifndef I960SXCHIPSET_CORECHIPSETFEATURES_H
#define I960SXCHIPSET_CORECHIPSETFEATURES_H
#include "MemoryThing.h"
#include "ProcessorSerializer.h"
class CoreChipsetFeatures : public IOSpaceThing {
public:
    static constexpr auto AllowDebuggingStatements = true;
    enum class Registers : uint32_t {
        Led, // one byte
        DisplayMemoryReadsAndWrites,
        DisplayCacheLineUpdates,
        PortZGPIO = 0x10, // one byte wide
        PortZGPIODirection, // one byte wide
        PortZGPIOPolarity,
        PortZGPIOPullup,
        ConsoleFlush = 0x20, // 2 bytes for alignment purposes
        ConsoleFlushUpper, // 2 bytes for alignment purposes
#define TwoByteEntry(Prefix) Prefix, Prefix ## Upper
#define FourByteEntry(Prefix) Prefix ## LowerHalf, Prefix ## UpperLowerHalf, Prefix ## UpperHalf, Prefix ## UpperUpperHalf
        TwoByteEntry(ConsoleAvailable),
        TwoByteEntry(ConsoleAvailableForWrite),
        TwoByteEntry(ConsoleIO),
        /// @todo implement console buffer
        FourByteEntry(ConsoleBufferAddress),
        ConsoleBufferLength, // up to 256 bytes in length
        ConsoleBufferDoorbell, // read from this to do a buffered read, write to this to do a write from memory to console
#undef FourByteEntry
#undef TwoByteEntry
    };
    explicit CoreChipsetFeatures(Address offsetFromIOBase = 0);
    ~CoreChipsetFeatures() override = default;
private:
    uint8_t readFromStream(Stream& aStream, Address baseAddress, uint8_t length) noexcept;
    uint8_t writeToStream(Stream& aStream, Address baseAddress, uint8_t length) noexcept;
public:
    [[nodiscard]] uint8_t read8(Address address) noexcept override;
    [[nodiscard]] uint16_t read16(Address address) noexcept override;
    void write16(Address address, uint16_t value) noexcept override;
    void write8(Address address, uint8_t value) noexcept override;
    [[nodiscard]] constexpr bool displayMemoryReadsAndWrites() const noexcept { return AllowDebuggingStatements && displayMemoryReadsAndWrites_; }
    [[nodiscard]] constexpr bool displayCacheLineUpdates() const noexcept { return AllowDebuggingStatements && displayCacheLineUpdates_; }
    void setDisplayMemoryReadsAndWrites(bool value) noexcept;
    void setDisplayCacheLineUpdates(bool value) noexcept;
    [[nodiscard]] constexpr bool debuggingActive() const noexcept { return AllowDebuggingStatements; }
private:
    static void writeLed(uint8_t value) noexcept;
    static uint8_t readLed() noexcept;
private:
    bool displayMemoryReadsAndWrites_ = false;
    bool displayCacheLineUpdates_ = false;
    union {
        uint32_t full = 0;
        uint16_t halves[2];
    } consoleBufferBaseAddress_;
    uint8_t consoleBufferLength_ = 0;
    uint8_t buffer_[256] = { 0 };
};
#endif //I960SXCHIPSET_CORECHIPSETFEATURES_H
