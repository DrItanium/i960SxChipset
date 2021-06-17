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

#if defined(ARDUINO_NRF52_ADAFRUIT) || defined(ARDUINO_SAMD_FEATHER_M0)
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
    Feather_nRF52832,
    MetroM4,
    Unknown,
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
#elif defined(ADAFRUIT_FEATHER_M0_ADALOGGER)
        return TargetMCU::FeatherM0Adalogger;
#elif defined(ADAFRUIT_FEATHER_M0_BASIC)
    return TargetMCU::FeatherM0Basic;
#elif defined(ADAFRUIT_METRO_M4_EXPRESS)
    return TargetMCU::MetroM4;
#elif defined(ARDUINO_NRF52_ADAFRUIT)
        return TargetMCU::Feather_nRF52832;
#else
    return TargetMCU::Unknown;
#endif
    }
    [[nodiscard]] static constexpr auto onAtmega1284p() noexcept { return getMCUTarget() == TargetMCU::ATmega1284p; }
    [[nodiscard]] static constexpr auto onGrandCentralM4() noexcept { return getMCUTarget() == TargetMCU::GrandCentralM4; }
    [[nodiscard]] static constexpr auto onUnknownTarget() noexcept { return getMCUTarget() == TargetMCU::Unknown; }
    [[nodiscard]] static constexpr auto onFeatherM0Basic() noexcept { return getMCUTarget() == TargetMCU::FeatherM0Basic; }
    [[nodiscard]] static constexpr auto onFeatherM0Adalogger() noexcept { return getMCUTarget() == TargetMCU::FeatherM0Adalogger; }
    [[nodiscard]] static constexpr auto onMetroM4() noexcept { return getMCUTarget() == TargetMCU::MetroM4; }
    [[nodiscard]] static constexpr auto onNRF52832() noexcept { return getMCUTarget() == TargetMCU::Feather_nRF52832; }
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
    [[nodiscard]] static constexpr auto hasBuiltinSDCard() noexcept {
#ifdef HAS_BUILTIN_SDCARD
        return true;
#else
        return false;
#endif
    }
    [[nodiscard]] static constexpr auto usesDisplayShield() noexcept {
#ifdef USES_DISPLAY_SHIELD
        return true;
#else
        return false;
#endif
    }
    [[nodiscard]] static constexpr auto getSPIMOSIPin() noexcept { return PIN_SPI_MOSI; }
    [[nodiscard]] static constexpr auto getSPIMISOPin() noexcept { return PIN_SPI_MISO; }
    [[nodiscard]] static constexpr auto getSPISCKPin() noexcept { return PIN_SPI_SCK; }
    [[nodiscard]] static constexpr auto getSDAPin() noexcept { return PIN_WIRE_SDA; }
    [[nodiscard]] static constexpr auto getSCLPin() noexcept { return PIN_WIRE_SCL; }
    [[nodiscard]] static constexpr auto getSRAMAmountInBytes() noexcept {
        switch (getMCUTarget())  {
            case TargetMCU::ATmega1284p:
                return 16_KB;
            case TargetMCU::FeatherM0Adalogger:
            case TargetMCU::FeatherM0Basic:
                return 32_KB;
            case TargetMCU::Feather_nRF52832:
                return 64_KB;
            case TargetMCU::MetroM4:
                return 192_KB;
            case TargetMCU::GrandCentralM4:
                return 256_KB;
            default:
                return 0ull;
        }
    }
    [[nodiscard]] static constexpr auto oneFourthSRAMAmountInBytes() noexcept {
        return getSRAMAmountInBytes() / 4;
    }
    [[nodiscard]] static constexpr auto oneEighthSRAMAmountInBytes() noexcept {
        return getSRAMAmountInBytes() / 8;
    }
    [[nodiscard]] static constexpr auto numberOfDataCacheLines() noexcept {
            switch (getMCUTarget())  {
                case TargetMCU::ATmega1284p:
                    return 8;
                case TargetMCU::FeatherM0Adalogger:
                case TargetMCU::FeatherM0Basic:
                    return 8;
                case TargetMCU::Feather_nRF52832:
                    return 8;
                case TargetMCU::MetroM4:
                    return 8;
                case TargetMCU::GrandCentralM4:
                    return 8;
                default:
                    return 8;
            }
    }
    [[nodiscard]] static constexpr auto getDataCacheLineSize() noexcept {
        switch (getMCUTarget())  {
            case TargetMCU::ATmega1284p:
                return 512;
            case TargetMCU::FeatherM0Adalogger:
            case TargetMCU::FeatherM0Basic:
                return 512;
            case TargetMCU::Feather_nRF52832:
                return 512;
            case TargetMCU::MetroM4:
                return 512;
            case TargetMCU::GrandCentralM4:
                return 512;
            default:
                return 512;
        }
    }
    [[nodiscard]] static constexpr auto numberOfInstructionCacheLines() noexcept {
        switch (getMCUTarget())  {
            case TargetMCU::ATmega1284p:
                return 8;
            case TargetMCU::FeatherM0Adalogger:
            case TargetMCU::FeatherM0Basic:
                return 8;
            case TargetMCU::Feather_nRF52832:
                return 8;
            case TargetMCU::MetroM4:
                return 8;
            case TargetMCU::GrandCentralM4:
                return 8;
            default:
                return 8;
        }
    }
    [[nodiscard]] static constexpr auto getInstructionCacheLineSize() noexcept {
        switch (getMCUTarget())  {
            case TargetMCU::ATmega1284p:
                return 512;
            case TargetMCU::FeatherM0Adalogger:
            case TargetMCU::FeatherM0Basic:
                return 512;
            case TargetMCU::Feather_nRF52832:
                return 512;
            case TargetMCU::MetroM4:
                return 512;
            case TargetMCU::GrandCentralM4:
                return 512;
            default:
                return 512;
        }
    }
public:
    TargetBoard() = delete;
    ~TargetBoard() = delete;
    TargetBoard(const TargetBoard&) = delete;
    TargetBoard(TargetBoard&&) = delete;
    TargetBoard& operator=(const TargetBoard&) = delete;
    TargetBoard& operator=(TargetBoard&&) = delete;
};

static_assert(!TargetBoard::onUnknownTarget(), "ERROR: Target Board has not been defined, please define to continue");

#endif //I960SXCHIPSET_MCUPLATFORM_H
