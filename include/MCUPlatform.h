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

// contains routines for handling microcontroller platform detection

#ifndef I960SXCHIPSET_MCUPLATFORM_H
#define I960SXCHIPSET_MCUPLATFORM_H
#include <Arduino.h>
#include "DependentFalse.h"

constexpr unsigned long long int operator "" _KB(unsigned long long int value) noexcept { return value * 1024; }
constexpr unsigned long long int operator "" _MB(unsigned long long int value) noexcept { return value * 1024 * 1024; }
constexpr unsigned long long int operator "" _KHz(unsigned long long int value) noexcept { return value * 1000; }
constexpr unsigned long long int operator "" _MHz(unsigned long long int value) noexcept { return value * 1000 * 1000; }
static_assert(2_KHz == 2'000);
static_assert(2_MHz == 2'000'000);
static_assert(20_MHz == 20'000'000);

#ifndef NUM_ANALOG_OUTPUTS
#define NUM_ANALOG_OUTPUTS 0
#endif
enum class TargetMCU {
    ATmega1284p,
    Unknown,
};
class MCUConfiguration final {
public:
    constexpr MCUConfiguration(uint32_t sramSize,
                               uint32_t dataCacheLineCount,
                               uint32_t dataCacheLineSize,
                               uint32_t instructionCacheLineCount,
                               uint32_t instructionCacheLineSize,
                               uint32_t maxOpenFiles,
                               uint32_t ioExpanderSpeedCap,
                               bool hasBuiltinSDCard,
                               bool usesDisplayShield) noexcept : sramAmount_(sramSize),
                             dataCacheLineCount_(dataCacheLineCount),
                             dataCacheLineSize_(dataCacheLineSize),
                             instructionCacheLineCount_(instructionCacheLineCount),
                             instructionCacheLineSize_(instructionCacheLineSize),
                             maximumNumberOfOpenFiles_(maxOpenFiles),
                             ioExpanderPeripheralSpeed_(ioExpanderSpeedCap > 10_MHz ? 10_MHz : ioExpanderSpeedCap),
                             builtinSDCard_(hasBuiltinSDCard),
                             usesDisplayShield_(usesDisplayShield) { }
    [[nodiscard]] constexpr uint32_t getSramAmount() const noexcept { return sramAmount_; }
    [[nodiscard]] constexpr uint32_t getDataCacheLineCount() const noexcept { return dataCacheLineCount_; }
    [[nodiscard]] constexpr uint32_t getDataCacheLineSize() const noexcept { return dataCacheLineSize_; }
    [[nodiscard]] constexpr uint32_t getInstructionCacheLineCount() const noexcept { return instructionCacheLineCount_; }
    [[nodiscard]] constexpr uint32_t getInstructionCacheLineSize() const noexcept { return instructionCacheLineSize_; }
    [[nodiscard]] constexpr uint32_t getMaximumNumberOfOpenFiles() const noexcept { return maximumNumberOfOpenFiles_; }
    [[nodiscard]] constexpr auto hasBuiltinSDCard() const noexcept { return builtinSDCard_; }
    [[nodiscard]] constexpr auto usesDisplayShield() const noexcept { return usesDisplayShield_; }
    [[nodiscard]] constexpr auto runIOExpanderSPIInterfaceAt() const noexcept  { return ioExpanderPeripheralSpeed_; }
private:
    uint32_t sramAmount_;
    uint32_t dataCacheLineCount_;
    uint32_t dataCacheLineSize_;
    uint32_t instructionCacheLineCount_;
    uint32_t instructionCacheLineSize_;
    uint32_t maximumNumberOfOpenFiles_;
    uint32_t ioExpanderPeripheralSpeed_;
    bool builtinSDCard_;
    bool usesDisplayShield_;
};
template<TargetMCU mcu>
constexpr MCUConfiguration BoardDescription = {0, 8, 512, 8, 512, 32, 10_MHz, false, false};
template<>
constexpr MCUConfiguration BoardDescription<TargetMCU::ATmega1284p> = {
        16_KB,
        64, 64,
        64, 64,
        32,
        10_MHz,
        false,
        true
};
// depends on the board used
[[nodiscard]] constexpr auto inDebugMode() noexcept {
#if defined(__PLATFORMIO_BUILD_DEBUG__) || defined(DEBUG) || defined(__DEBUG__)
    return true;
#else
    return false;
#endif
}

class TargetBoard {
public:
    [[nodiscard]] static constexpr auto getCPUFrequency() noexcept { return F_CPU; }
    [[nodiscard]] static constexpr auto getDigitalPinCount() noexcept { return NUM_DIGITAL_PINS; }
    [[nodiscard]] static constexpr auto getAnalogInputCount() noexcept { return NUM_ANALOG_INPUTS; }
    [[nodiscard]] static constexpr auto getAnalogOutputCount() noexcept { return NUM_ANALOG_OUTPUTS; }
    [[nodiscard]] static constexpr auto getMCUTarget() noexcept {
#ifdef ARDUINO_AVR_ATmega1284
        return TargetMCU::ATmega1284p;
#elif defined(ADAFRUIT_FEATHER_M0_ADALOGGER)
        return TargetMCU::FeatherM0Adalogger;
#else
    return TargetMCU::Unknown;
#endif
    }
    [[nodiscard]] static constexpr auto onAtmega1284p() noexcept { return getMCUTarget() == TargetMCU::ATmega1284p; }
    [[nodiscard]] static constexpr auto onUnknownTarget() noexcept { return getMCUTarget() == TargetMCU::Unknown; }
/**
 * @brief Is there an onboard sdcard slot?
 * @return True if defined via the command line
 */
    [[nodiscard]] static constexpr auto hasBuiltinSDCard() noexcept { return BoardDescription<getMCUTarget()>.hasBuiltinSDCard(); }
    [[nodiscard]] static constexpr auto usesDisplayShield() noexcept { return BoardDescription<getMCUTarget()>.usesDisplayShield(); }
    [[nodiscard]] static constexpr auto getSPIMOSIPin() noexcept { return PIN_SPI_MOSI; }
    [[nodiscard]] static constexpr auto getSPIMISOPin() noexcept { return PIN_SPI_MISO; }
    [[nodiscard]] static constexpr auto getSPISCKPin() noexcept { return PIN_SPI_SCK; }
    [[nodiscard]] static constexpr auto getSDAPin() noexcept { return PIN_WIRE_SDA; }
    [[nodiscard]] static constexpr auto getSCLPin() noexcept { return PIN_WIRE_SCL; }
    [[nodiscard]] static constexpr auto getSRAMAmountInBytes() noexcept { return BoardDescription<getMCUTarget()>.getSramAmount(); }
    [[nodiscard]] static constexpr auto oneFourthSRAMAmountInBytes() noexcept { return getSRAMAmountInBytes() / 4; }
    [[nodiscard]] static constexpr auto oneEighthSRAMAmountInBytes() noexcept { return getSRAMAmountInBytes() / 8; }
    [[nodiscard]] static constexpr auto numberOfDataCacheLines() noexcept { return BoardDescription<getMCUTarget()>.getDataCacheLineCount(); }
    [[nodiscard]] static constexpr auto getDataCacheLineSize() noexcept { return BoardDescription<getMCUTarget()>.getDataCacheLineSize(); }
    [[nodiscard]] static constexpr auto numberOfInstructionCacheLines() noexcept { return BoardDescription<getMCUTarget()>.getInstructionCacheLineCount(); }
    [[nodiscard]] static constexpr auto getInstructionCacheLineSize() noexcept { return BoardDescription<getMCUTarget()>.getInstructionCacheLineSize(); }
    [[nodiscard]] static constexpr auto maximumNumberOfOpenFilesFromSDCard() noexcept { return BoardDescription<getMCUTarget()>.getMaximumNumberOfOpenFiles(); }
    [[nodiscard]] static constexpr auto runIOExpanderSPIInterfaceAt() noexcept { return BoardDescription<getMCUTarget()>.runIOExpanderSPIInterfaceAt(); }
public:
    TargetBoard() = delete;
    ~TargetBoard() = delete;
    TargetBoard(const TargetBoard&) = delete;
    TargetBoard(TargetBoard&&) = delete;
    TargetBoard& operator=(const TargetBoard&) = delete;
    TargetBoard& operator=(TargetBoard&&) = delete;
};

static_assert(!TargetBoard::onUnknownTarget(), "ERROR: Target Board has not been defined, please define to continue");
static_assert(TargetBoard::getSRAMAmountInBytes() >= 16_KB, "ERROR: Less than 16kb of sram is not allowed!");

/**
 * @brief The backing design of the registers within the chipset that are 32-bits in width
 */
union SplitWord32 {
    uint32_t wholeValue_ = 0;
    int32_t signedWholeValue;
    uint16_t halves[sizeof(uint32_t) / sizeof(uint16_t)];
    uint8_t bytes[sizeof(uint32_t) / sizeof(uint8_t)];
};
union SplitWord128 {
    uint8_t bytes[16] = { 0 };
    uint16_t shorts[16/sizeof(uint16_t)];
    uint32_t words[16/sizeof(uint32_t)];
    uint64_t quads[16/sizeof(uint64_t)];
};
union SplitWord16 {
    constexpr SplitWord16(uint16_t value = 0) noexcept : wholeValue_(value) { }
    uint16_t wholeValue_ = 0;
    uint8_t bytes[sizeof(uint16_t) / sizeof(uint8_t)];
};
union SplitWord64 {
    uint64_t wholeValue_ = 0;
    int64_t signedWholeValue;
    uint32_t words[sizeof(uint64_t) / sizeof(uint32_t)];
    uint16_t halves[sizeof(uint64_t) / sizeof(uint16_t)];
    uint8_t bytes[sizeof(uint64_t) / sizeof(uint8_t)];
};
#endif //I960SXCHIPSET_MCUPLATFORM_H
