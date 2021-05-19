// contains routines for handling microcontroller platform detection
// Created by jwscoggins on 5/18/21.
//

#ifndef I960SXCHIPSET_MCUPLATFORM_H
#define I960SXCHIPSET_MCUPLATFORM_H
enum class TargetMCU {
    ATmega1284p,
    GrandCentralM4,
    NRF52830,
    Unknown,
};
[[nodiscard]] constexpr auto getMCUTarget() noexcept {
#ifdef ARDUINO_AVR_ATmega1284
    return TargetMCU::ATmega1284p;
#elif defined(ARDUINO_GRAND_CENTRAL_M4)
    return TargetMCU::GrandCentralM4;
#elif defined(ARDUINO_NRF52_ADAFRUIT)
    return TargetMCU::NRF52830;
    // @todo add more targets here
#else
    return TargetMCU::Unknown;
#endif
}
[[nodiscard]] constexpr auto onAtmega1284p() noexcept {
    return getMCUTarget() == TargetMCU::ATmega1284p;
}
[[nodiscard]] constexpr auto onGrandCentralM4() noexcept {
    return getMCUTarget() == TargetMCU::GrandCentralM4;
}

[[nodiscard]] constexpr auto onNRF52830() noexcept {
    return getMCUTarget() == TargetMCU::NRF52830;
}

[[nodiscard]] constexpr auto onUnknownTarget() noexcept {
    return getMCUTarget() == TargetMCU::Unknown;
}

#endif //I960SXCHIPSET_MCUPLATFORM_H
