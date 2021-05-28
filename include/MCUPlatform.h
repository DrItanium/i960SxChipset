// contains routines for handling microcontroller platform detection
// Created by jwscoggins on 5/18/21.
//

#ifndef I960SXCHIPSET_MCUPLATFORM_H
#define I960SXCHIPSET_MCUPLATFORM_H
#include <Arduino.h>
#ifdef ARDUINO_SAMD_FEATHER_M0
#define ADAFRUIT_FEATHER_M0
#ifdef HAS_BUILTIN_SDCARD
#define ADAFRUIT_FEATHER_M0_ADALOGGER
#else /* !defined(HAS_BUILTIN_SDCARD) */
#define ADAFRUIT_FEATHER_M0_BASIC
#endif
#endif

#ifndef NUM_ANALOG_OUTPUTS
#define NUM_ANALOG_OUTPUTS 0
#endif
enum class TargetMCU {
    ATmega1284p,
    GrandCentralM4,
    FeatherM0Basic,
    FeatherM0Adalogger,
    MetroM4,
    Unknown,
};


class TargetBoard {
public:
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
    [[nodiscard]] static constexpr auto onMetroM4() noexcept { return getMCUTarget() == TargetMCU::MetroM4; }
    [[nodiscard]] static constexpr auto onFeatherM0() noexcept { return onFeatherM0Basic() || onFeatherM0Adalogger(); }
    [[nodiscard]] static constexpr auto onFeatherBoard() noexcept { return onFeatherM0(); }
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
private:
    TargetBoard() = delete;
    ~TargetBoard() = delete;
    TargetBoard(const TargetBoard&) = delete;
    TargetBoard(TargetBoard&&) = delete;
    TargetBoard& operator=(const TargetBoard&) = delete;
    TargetBoard& operator=(TargetBoard&&) = delete;
};

static_assert(!TargetBoard::onUnknownTarget(), "ERROR: Target Board has not been defined, please define to continue");
#endif //I960SXCHIPSET_MCUPLATFORM_H
