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
#define TwoByteEntry(Prefix) Prefix ## 0, Prefix ## 1
#define FourByteEntry(Prefix) \
        TwoByteEntry(Prefix ## 0), \
        TwoByteEntry(Prefix ## 1)
#define EightByteEntry(Prefix) \
        FourByteEntry(Prefix ## 0), \
        FourByteEntry(Prefix ## 1)
#define TwelveByteEntry(Prefix) \
        EightByteEntry(Prefix ## 0), \
        FourByteEntry(Prefix ## 1)
#define SixteenByteEntry(Prefix) \
        EightByteEntry(Prefix ## 0), \
        EightByteEntry(Prefix ## 1)
        Led, // one byte
        DisplayMemoryReadsAndWrites,
        DisplayCacheLineUpdates,
        PortZGPIO, // one byte wide
        PortZGPIODirection, // one byte wide
        PortZGPIOPolarity,
        PortZGPIOPullup,
        TwoByteEntry(ConsoleFlush),
        TwoByteEntry(ConsoleAvailable),
        TwoByteEntry(ConsoleAvailableForWrite),
        TwoByteEntry(ConsoleIO),
        FourByteEntry(ConsoleBufferAddress),
        ConsoleBufferLength, // up to 256 bytes in length
        ConsoleBufferDoorbell, // read from this to do a buffered read, write to this to do a write from memory to console
        SixteenByteEntry(PatternEngine_ActualPattern),
        FourByteEntry(PatternEngine_StartAddress),
        FourByteEntry(PatternEngine_Length),
        TwoByteEntry(PatternEngine_Doorbell),
#undef SixteenByteEntry
#undef TwelveByteEntry
#undef EightByteEntry
#undef FourByteEntry
#undef TwoByteEntry
        End,
        ConsoleFlush = ConsoleFlush0,
        ConsoleAvailable = ConsoleAvailable0,
        ConsoleAvailableForWrite = ConsoleAvailableForWrite0,
        ConsoleIO = ConsoleIO0,
        ConsoleBufferAddressLower = ConsoleBufferAddress00,
        ConsoleBufferAddressUpper = ConsoleBufferAddress10,
        PatternEngine_ActualPattern000 = PatternEngine_ActualPattern0000,
        PatternEngine_ActualPattern001 = PatternEngine_ActualPattern0010,
        PatternEngine_ActualPattern010 = PatternEngine_ActualPattern0100,
        PatternEngine_ActualPattern011 = PatternEngine_ActualPattern0110,
        PatternEngine_ActualPattern100 = PatternEngine_ActualPattern1000,
        PatternEngine_ActualPattern101 = PatternEngine_ActualPattern1010,
        PatternEngine_ActualPattern110 = PatternEngine_ActualPattern1100,
        PatternEngine_ActualPattern111 = PatternEngine_ActualPattern1110,
        PatternEngine_ActualPattern = PatternEngine_ActualPattern000,
        PatternEngine_StartAddressLower = PatternEngine_StartAddress00,
        PatternEngine_StartAddressUpper = PatternEngine_StartAddress10,
        PatternEngine_LengthLower = PatternEngine_Length00,
        PatternEngine_LengthUpper = PatternEngine_Length10,
        PatternEngine_Doorbell = PatternEngine_Doorbell0,
    };
    static_assert(static_cast<int>(Registers::End) < 0x100);
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
    SplitWord32 consoleBufferBaseAddress_;
    uint8_t consoleBufferLength_ = 0;
    uint8_t buffer_[256] = { 0 };
    uint16_t pattern_[16 / sizeof(uint16_t)] = { 0 };

};
#endif //I960SXCHIPSET_CORECHIPSETFEATURES_H
