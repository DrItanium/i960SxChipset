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
#ifdef ARDUINO_SAMD_FEATHER_M0
#define ADAFRUIT_FEATHER_M0
#ifdef HAS_BUILTIN_SDCARD
#define ADAFRUIT_FEATHER_M0_ADALOGGER
#else /* !defined(HAS_BUILTIN_SDCARD) */
#define ADAFRUIT_FEATHER_M0_BASIC
#endif
#endif

#if defined(ARDUINO_SAMD_FEATHER_M0)
#define ADAFRUIT_FEATHER
#endif

#ifndef NUM_ANALOG_OUTPUTS
#define NUM_ANALOG_OUTPUTS 0
#endif
enum class TargetMCU {
    ATmega1284p,
    GrandCentralM4,
    FeatherM0Basic,
    FeatherM0Adalogger,
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
                               bool hasBuiltinSDCard,
                               bool usesDisplayShield) noexcept : sramAmount_(sramSize),
                             dataCacheLineCount_(dataCacheLineCount),
                             dataCacheLineSize_(dataCacheLineSize),
                             instructionCacheLineCount_(instructionCacheLineCount),
                             instructionCacheLineSize_(instructionCacheLineSize),
                             maximumNumberOfOpenFiles_(maxOpenFiles),
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
private:
    uint32_t sramAmount_;
    uint32_t dataCacheLineCount_;
    uint32_t dataCacheLineSize_;
    uint32_t instructionCacheLineCount_;
    uint32_t instructionCacheLineSize_;
    uint32_t maximumNumberOfOpenFiles_;
    bool builtinSDCard_;
    bool usesDisplayShield_;
};
template<TargetMCU mcu>
constexpr MCUConfiguration BoardDescription = {0, 8, 512, 8, 512, 32, false, false};
template<> constexpr MCUConfiguration BoardDescription<TargetMCU::ATmega1284p> = { 16_KB, 8, 512, 8, 512, 32, false, true};
template<> constexpr MCUConfiguration BoardDescription<TargetMCU::GrandCentralM4> = { 256_KB, 16, 512, 16, 512, 64, true, true};
// depends on the board used
template<> constexpr MCUConfiguration BoardDescription<TargetMCU::FeatherM0Basic> = { 32_KB, 16, 512, 16, 512, 64, false, false};
template<> constexpr MCUConfiguration BoardDescription<TargetMCU::FeatherM0Adalogger> = { 32_KB, 16, 512, 16, 512, 64, true, false };
[[nodiscard]] constexpr auto inDebugMode() noexcept {
#if defined(__PLATFORMIO_BUILD_DEBUG__) || defined(DEBUG) || defined(__DEBUG__)
    return true;
#else
    return false;
#endif
}

class TargetBoard {
public:
    [[nodiscard]] static constexpr auto cpuIsARMArchitecture() noexcept {
#ifdef __arm__
        return true;
#else
        return false;
#endif
    }
    [[nodiscard]] static constexpr auto cpuIsAVRArchitecture() noexcept {
#if defined(__AVR) || defined(__AVR__)
        return true;
#else
        return false;
#endif
    }
    [[nodiscard]] static constexpr auto getCPUFrequency() noexcept { return F_CPU; }
    [[nodiscard]] static constexpr auto getDigitalPinCount() noexcept { return NUM_DIGITAL_PINS; }
    [[nodiscard]] static constexpr auto getAnalogInputCount() noexcept { return NUM_ANALOG_INPUTS; }
    [[nodiscard]] static constexpr auto getAnalogOutputCount() noexcept { return NUM_ANALOG_OUTPUTS; }
    [[nodiscard]] static constexpr auto getMCUTarget() noexcept {
#ifdef ARDUINO_AVR_ATmega1284
        return TargetMCU::ATmega1284p;
#elif defined(ARDUINO_GRAND_CENTRAL_M4)
        return TargetMCU::GrandCentralM4;
#elif defined(ADAFRUIT_FEATHER_M0_ADALOGGER)
        return TargetMCU::FeatherM0Adalogger;
#elif defined(ADAFRUIT_FEATHER_M0_BASIC)
    return TargetMCU::FeatherM0Basic;
#elif defined(ADAFRUIT_METRO_M4_EXPRESS)
    return TargetMCU::MetroM4;
#else
    return TargetMCU::Unknown;
#endif
    }
    [[nodiscard]] static constexpr auto onAtmega1284p() noexcept { return getMCUTarget() == TargetMCU::ATmega1284p; }
    [[nodiscard]] static constexpr auto onGrandCentralM4() noexcept { return getMCUTarget() == TargetMCU::GrandCentralM4; }
    [[nodiscard]] static constexpr auto onUnknownTarget() noexcept { return getMCUTarget() == TargetMCU::Unknown; }
    [[nodiscard]] static constexpr auto onFeatherM0Basic() noexcept { return getMCUTarget() == TargetMCU::FeatherM0Basic; }
    [[nodiscard]] static constexpr auto onFeatherM0Adalogger() noexcept { return getMCUTarget() == TargetMCU::FeatherM0Adalogger; }
    [[nodiscard]] static constexpr auto onFeatherM0() noexcept { return onFeatherM0Basic() || onFeatherM0Adalogger(); }
    [[nodiscard]] static constexpr auto onFeatherBoard() noexcept {
#ifdef ADAFRUIT_FEATHER
        return true;
#else
        return false;
#endif
    }
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

#endif //I960SXCHIPSET_MCUPLATFORM_H
