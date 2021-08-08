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
    RaspberryPiPico,
    Unknown,
};
class MCUConfiguration final {
public:
    constexpr MCUConfiguration(uint32_t sramSize,
                               uint32_t cacheLineCount,
                               uint32_t cacheLineSize,
                               uint32_t maxOpenFiles,
                               uint32_t ioExpanderSpeedCap,
                               uint32_t psramSpeedCap,
                               bool hasBuiltinSDCard,
                               bool usesDisplayShield) noexcept : sramAmount_(sramSize),
                               cacheLineCount_(cacheLineCount),
                               cacheLineSize_(cacheLineSize),
                             maximumNumberOfOpenFiles_(maxOpenFiles),
                             ioExpanderPeripheralSpeed_(ioExpanderSpeedCap > 10_MHz ? 10_MHz : ioExpanderSpeedCap),
                             psramSpeedCap_(psramSpeedCap > 33_MHz ? 33_MHz : psramSpeedCap),
                             builtinSDCard_(hasBuiltinSDCard),
                             usesDisplayShield_(usesDisplayShield) { }
    [[nodiscard]] constexpr uint32_t getSramAmount() const noexcept { return sramAmount_; }
    [[nodiscard]] constexpr uint32_t getCacheLineCount() const noexcept { return cacheLineCount_; }
    [[nodiscard]] constexpr uint32_t getCacheLineSize() const noexcept { return cacheLineSize_; }
    [[nodiscard]] constexpr uint32_t getMaximumNumberOfOpenFiles() const noexcept { return maximumNumberOfOpenFiles_; }
    [[nodiscard]] constexpr auto hasBuiltinSDCard() const noexcept { return builtinSDCard_; }
    [[nodiscard]] constexpr auto usesDisplayShield() const noexcept { return usesDisplayShield_; }
    [[nodiscard]] constexpr auto runIOExpanderSPIInterfaceAt() const noexcept  { return ioExpanderPeripheralSpeed_; }
    [[nodiscard]] constexpr auto runPSRAMAt() const noexcept { return psramSpeedCap_; }
private:
    uint32_t sramAmount_;
    uint32_t cacheLineCount_;
    uint32_t cacheLineSize_;
    uint32_t maximumNumberOfOpenFiles_;
    uint32_t ioExpanderPeripheralSpeed_;
    uint32_t psramSpeedCap_;
    bool builtinSDCard_;
    bool usesDisplayShield_;
};
template<TargetMCU mcu>
constexpr MCUConfiguration BoardDescription = {
        0,
        8, 512,
        32,
        10_MHz,
        5_MHz,
        false,
        false
};
template<>
constexpr MCUConfiguration BoardDescription<TargetMCU::ATmega1284p> = {
        16_KB,
        256, 32,
        32,
        10_MHz,
        5_MHz, // due to the current design, we have to run the psram at 5 Mhz
        false,
        true
};
template<>
constexpr MCUConfiguration BoardDescription<TargetMCU::GrandCentralM4> = {
        256_KB,
        256, 64, // 256, 64 element lines
        64,
        10_MHz,
        5_MHz,
        true,
        true,
};
template<>
constexpr MCUConfiguration BoardDescription<TargetMCU::RaspberryPiPico> = {
        264_KB,
        256, 64, // 256, 64 element lines
        64,
        10_MHz,
        5_MHz,
        false,
        false,
};
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
#elif defined(ARDUINO_RASPBERRY_PI_PICO)
        return TargetMCU::RaspberryPiPico;
#else
    return TargetMCU::Unknown;
#endif
    }
    [[nodiscard]] static constexpr auto onAtmega1284p() noexcept { return getMCUTarget() == TargetMCU::ATmega1284p; }
    [[nodiscard]] static constexpr auto onGrandCentralM4() noexcept { return getMCUTarget() == TargetMCU::GrandCentralM4; }
    [[nodiscard]] static constexpr auto onRaspberryPiPico() noexcept { return getMCUTarget() == TargetMCU::RaspberryPiPico; }
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
    [[nodiscard]] static constexpr auto numberOfCacheLines() noexcept { return BoardDescription<getMCUTarget()>.getCacheLineCount(); }
    [[nodiscard]] static constexpr auto cacheLineSize() noexcept { return BoardDescription<getMCUTarget()>.getCacheLineSize(); }
    [[nodiscard]] static constexpr auto maximumNumberOfOpenFilesFromSDCard() noexcept { return BoardDescription<getMCUTarget()>.getMaximumNumberOfOpenFiles(); }
    [[nodiscard]] static constexpr auto runIOExpanderSPIInterfaceAt() noexcept { return BoardDescription<getMCUTarget()>.runIOExpanderSPIInterfaceAt(); }
    [[nodiscard]] static constexpr auto runPSRAMAt() noexcept { return BoardDescription<getMCUTarget()>.runPSRAMAt(); }
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
union SplitWord16 {
    explicit constexpr SplitWord16(uint16_t value = 0) noexcept : wholeValue_(value) { }
    constexpr SplitWord16(uint8_t lower, uint8_t upper) noexcept : bytes{lower, upper} { }
    [[nodiscard]] constexpr auto getWholeValue() const noexcept { return wholeValue_; }
    uint16_t wholeValue_ = 0;
    uint8_t bytes[2];
};
union SplitWord32 {
    // adding this dropped program size by over 500 bytes!
    explicit constexpr SplitWord32(uint32_t value = 0) noexcept : wholeValue_(value) { }
    constexpr SplitWord32(uint8_t lowest, uint8_t lower, uint8_t higher, uint8_t highest) noexcept : bytes{lowest, lower, higher, highest} {}
    constexpr SplitWord32(uint16_t lower, uint16_t upper) noexcept : halves{lower, upper} { }
    uint32_t wholeValue_ = 0;
    int32_t signedWholeValue;
    uint16_t halves[sizeof(uint32_t) / sizeof(uint16_t)];
    byte bytes[sizeof(uint32_t)];
    struct {
        uint16_t lowerHalf_;
        uint16_t upperHalf_;
    };
};
union SplitWord128 {
    uint8_t bytes[16] = { 0 };
    uint16_t shorts[16/sizeof(uint16_t)];
    uint32_t words[16/sizeof(uint32_t)];
    uint64_t quads[16/sizeof(uint64_t)];
};
static_assert(TargetBoard::cpuIsAVRArchitecture() || TargetBoard::cpuIsARMArchitecture(), "ONLY AVR or ARM BASED MCUS ARE SUPPORTED!");
void invalidateGlobalCache() noexcept;
#endif //I960SXCHIPSET_MCUPLATFORM_H
