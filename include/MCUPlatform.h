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

enum class Pinout1284p : int {
    // this is described in digial pin order!
    // leave this one alone
    None = -1,
    PORT_B0 = 0,
    PORT_B1,
    PORT_B2,
    PORT_B3,
    PORT_B4,
    PORT_B5,
    PORT_B6,
    PORT_B7,
    PORT_D0,
    PORT_D1,
    PORT_D2,
    PORT_D3,
    PORT_D4,
    PORT_D5,
    PORT_D6,
    PORT_D7,
    PORT_C0,
    PORT_C1,
    PORT_C2,
    PORT_C3,
    PORT_C4,
    PORT_C5,
    PORT_C6,
    PORT_C7,
    PORT_A0,
    PORT_A1,
    PORT_A2,
    PORT_A3,
    PORT_A4,
    PORT_A5,
    PORT_A6,
    PORT_A7,
    Count,          // special, must be last
};
enum class PinoutRaspberryPiPico : int {
    GPIO0 = 0,
    GPIO1,
    GPIO2,
    GPIO3,
    GPIO4,
    GPIO5,
    GPIO6,
    GPIO7,
    GPIO8,
    GPIO9,
    GPIO10,
    GPIO11,
    GPIO12,
    GPIO13,
    GPIO14,
    GPIO15,
    GPIO16,
    GPIO17,
    GPIO18,
    GPIO19,
    GPIO20,
    GPIO21,
    GPIO22,
    GPIO23,
    GPIO24,
    GPIO25,
    GPIO26,
    GPIO27,
    GPIO28,
    GPIO29,
    Count,
    None = -1,
};
static_assert(static_cast<int>(PinoutRaspberryPiPico::Count) == 30, "Raspberry Pi Pico Has 30 GPIO");
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
    RaspberryPiPico,
    Unknown,
};
template<typename T>
class MCUConfiguration final {
public:
    using UnderlyingPinoutType = T;
    constexpr MCUConfiguration(uint32_t sramSize,
                               uint32_t cacheLineCount,
                               uint32_t cacheLineSize,
                               uint32_t maxOpenFiles,
                               uint32_t ioExpanderSpeedCap,
                               uint32_t psramSpeedCap,
                               bool hasBuiltinSDCard,
                               bool usesDisplayShield,
                               int readyPin = -1,
                               int clockOutPin = -1,
                               int addressStatePin = -1,
                               int psramEnPin = -1,
                               int gpioSelectPin = -1,
                               int misoPin = -1,
                               int mosiPin = -1,
                               int sckPin = -1,
                               int denPin = -1,
                               int cacheEnPin = -1,
                               int reset960Pin = -1,
                               int int0Pin = -1,
                               int sclPin = -1,
                               int sdaPin = -1,
                               int spiOffset0Pin = -1,
                               int spiOffset1Pin = -1,
                               int spiOffset2Pin = -1,
                               int displayEnPin = -1,
                               int dcPin = -1,
                               int sdEnablePin = -1,
                               int wrPin = -1,
                               int burstAddress1Pin = -1,
                               int burstAddress2Pin = -1,
                               int burstAddress3Pin = -1,
                               int byteEnable0Pin = -1,
                               int byteEnable1Pin = -1,
                               int blastPin = -1,
                               int failPin = -1
    ) noexcept : sramAmount_(sramSize),
                 cacheLineCount_(cacheLineCount),
                 cacheLineSize_(cacheLineSize),
                 maximumNumberOfOpenFiles_(maxOpenFiles),
                 ioExpanderPeripheralSpeed_(ioExpanderSpeedCap > 10_MHz ? 10_MHz : ioExpanderSpeedCap),
                 psramSpeedCap_(psramSpeedCap > 33_MHz ? 33_MHz : psramSpeedCap),
                 builtinSDCard_(hasBuiltinSDCard),
                 usesDisplayShield_(usesDisplayShield),
                 readyPin_(readyPin),
                 clockOutPin_(clockOutPin),
                 addressStatePin_(addressStatePin),
                 psramEnPin_(psramEnPin),
                 gpioSelectPin_(gpioSelectPin),
                 misoPin_(misoPin),
                 mosiPin_(mosiPin),
                 sckPin_(sckPin),
                 denPin_(denPin),
                 cacheEnPin_(cacheEnPin),
                 reset960Pin_(reset960Pin),
                 int0Pin_(int0Pin),
                 sclPin_(sclPin),
                 sdaPin_(sdaPin),
                 spiOffset0Pin_(spiOffset0Pin),
                 spiOffset1Pin_(spiOffset1Pin),
                 spiOffset2Pin_(spiOffset2Pin),
                 displayEnPin_(displayEnPin),
                 dcPin_(dcPin),
                 sdEnablePin_(sdEnablePin),
                 wrPin_(wrPin),
                 burstAddress1Pin_(burstAddress1Pin),
                 burstAddress2Pin_(burstAddress2Pin),
                 burstAddress3Pin_(burstAddress3Pin),
                 byteEnable0Pin_(byteEnable0Pin),
                 byteEnable1Pin_(byteEnable1Pin),
                 blastPin_(blastPin),
                 failPin_(failPin) {}
    constexpr MCUConfiguration(uint32_t sramSize,
                               uint32_t cacheLineCount,
                               uint32_t cacheLineSize,
                               uint32_t maxOpenFiles,
                               uint32_t ioExpanderSpeedCap,
                               uint32_t psramSpeedCap,
                               bool hasBuiltinSDCard,
                               bool usesDisplayShield,
                               T readyPin = T::None,
                               T clockOutPin = T::None,
                               T addressStatePin = T::None,
                               T psramEnPin = T::None,
                               T gpioSelectPin = T::None,
                               T misoPin = T::None,
                               T mosiPin = T::None,
                               T sckPin = T::None,
                               T denPin = T::None,
                               T cacheEnPin = T::None,
                               T reset960Pin = T::None,
                               T int0Pin = T::None,
                               T sclPin = T::None,
                               T sdaPin = T::None,
                               T spiOffset0Pin = T::None,
                               T spiOffset1Pin = T::None,
                               T spiOffset2Pin = T::None,
                               T displayEnPin = T::None,
                               T dcPin = T::None,
                               T sdEnablePin = T::None,
                               T wrPin = T::None,
                               T burstAddress1Pin = T::None,
                               T burstAddress2Pin = T::None,
                               T burstAddress3Pin = T::None,
                               T byteEnable0Pin = T::None,
                               T byteEnable1Pin = T::None,
                               T blastPin = T::None,
                               T failPin = T::None
    ) noexcept : MCUConfiguration(sramSize,
                                  cacheLineCount,
                                  cacheLineSize,
                                  maxOpenFiles,
                                  ioExpanderSpeedCap,
                                  psramSpeedCap,
                                  hasBuiltinSDCard,
                                  usesDisplayShield,
                                  static_cast<int>(readyPin),
                                  static_cast<int>(clockOutPin),
                                  static_cast<int>(addressStatePin),
                                  static_cast<int>(psramEnPin),
                                  static_cast<int>(gpioSelectPin),
                                  static_cast<int>(misoPin),
                                  static_cast<int>(mosiPin),
                                  static_cast<int>(sckPin),
                                  static_cast<int>(denPin),
                                  static_cast<int>(cacheEnPin),
                                  static_cast<int>(reset960Pin),
                                  static_cast<int>(int0Pin),
                                  static_cast<int>(sclPin),
                                  static_cast<int>(sdaPin),
                                  static_cast<int>(spiOffset0Pin),
                                  static_cast<int>(spiOffset1Pin),
                                  static_cast<int>(spiOffset2Pin),
                                  static_cast<int>(displayEnPin),
                                  static_cast<int>(dcPin),
                                  static_cast<int>(sdEnablePin),
                                  static_cast<int>(wrPin),
                                  static_cast<int>(burstAddress1Pin),
                                  static_cast<int>(burstAddress2Pin),
                                  static_cast<int>(burstAddress3Pin),
                                  static_cast<int>(byteEnable0Pin),
                                  static_cast<int>(byteEnable1Pin),
                                  static_cast<int>(blastPin),
                                  static_cast<int>(failPin)) {}

    [[nodiscard]] constexpr uint32_t getSramAmount() const noexcept { return sramAmount_; }
    [[nodiscard]] constexpr uint32_t getCacheLineCount() const noexcept { return cacheLineCount_; }
    [[nodiscard]] constexpr uint32_t getCacheLineSize() const noexcept { return cacheLineSize_; }
    [[nodiscard]] constexpr uint32_t getMaximumNumberOfOpenFiles() const noexcept { return maximumNumberOfOpenFiles_; }
    [[nodiscard]] constexpr auto hasBuiltinSDCard() const noexcept { return builtinSDCard_; }
    [[nodiscard]] constexpr auto usesDisplayShield() const noexcept { return usesDisplayShield_; }
    [[nodiscard]] constexpr auto runIOExpanderSPIInterfaceAt() const noexcept  { return ioExpanderPeripheralSpeed_; }
    [[nodiscard]] constexpr auto runPSRAMAt() const noexcept { return psramSpeedCap_; }
    [[nodiscard]] constexpr auto getReadyPin() const noexcept { return readyPin_; }
    [[nodiscard]] constexpr auto getClockOutPin() const noexcept { return clockOutPin_; }
    [[nodiscard]] constexpr auto getAddressStatePin() const noexcept { return addressStatePin_; }
    [[nodiscard]] constexpr auto getPsramEnPin() const noexcept { return psramEnPin_; }
    [[nodiscard]] constexpr auto getGpioSelectPin() const noexcept { return gpioSelectPin_; }
    [[nodiscard]] constexpr auto getMisoPin() const noexcept { return misoPin_; }
    [[nodiscard]] constexpr auto getMosiPin() const noexcept { return mosiPin_; }
    [[nodiscard]] constexpr auto getSckPin() const noexcept { return sckPin_; }
    [[nodiscard]] constexpr auto getDenPin() const noexcept { return denPin_; }
    [[nodiscard]] constexpr auto getCacheEnPin() const noexcept { return cacheEnPin_; }
    [[nodiscard]] constexpr auto getReset960Pin() const noexcept { return reset960Pin_; }
    [[nodiscard]] constexpr auto getInt0Pin() const noexcept { return int0Pin_; }
    [[nodiscard]] constexpr auto getSclPin() const noexcept { return sclPin_; }
    [[nodiscard]] constexpr auto getSdaPin() const noexcept { return sdaPin_; }
    [[nodiscard]] constexpr auto getSpiOffset0Pin() const noexcept { return spiOffset0Pin_; }
    [[nodiscard]] constexpr auto getSpiOffset1Pin() const noexcept { return spiOffset1Pin_; }
    [[nodiscard]] constexpr auto getSpiOffset2Pin() const noexcept { return spiOffset2Pin_; }
    [[nodiscard]] constexpr auto getDisplayEnPin() const noexcept { return displayEnPin_; }
    [[nodiscard]] constexpr auto getDcPin() const noexcept { return dcPin_; }
    [[nodiscard]] constexpr auto getSdEnablePin() const noexcept { return sdEnablePin_; }
    [[nodiscard]] constexpr auto getWrPin() const noexcept { return wrPin_; }
    [[nodiscard]] constexpr auto getBurstAddress1Pin() const noexcept { return burstAddress1Pin_; }
    [[nodiscard]] constexpr auto getBurstAddress2Pin() const noexcept { return burstAddress2Pin_; }
    [[nodiscard]] constexpr auto getBurstAddress3Pin() const noexcept { return burstAddress3Pin_; }
    [[nodiscard]] constexpr auto getByteEnable0Pin() const noexcept { return byteEnable0Pin_; }
    [[nodiscard]] constexpr auto getByteEnable1Pin() const noexcept { return byteEnable1Pin_; }
    [[nodiscard]] constexpr auto getBlastPin() const noexcept { return blastPin_; }
    [[nodiscard]] constexpr auto getFailPin() const noexcept { return failPin_; }
    [[nodiscard]] constexpr auto getNoneSpecifier() const noexcept { return T::None; }
private:
    uint32_t sramAmount_;
    uint32_t cacheLineCount_;
    uint32_t cacheLineSize_;
    uint32_t maximumNumberOfOpenFiles_;
    uint32_t ioExpanderPeripheralSpeed_;
    uint32_t psramSpeedCap_;
    bool builtinSDCard_;
    bool usesDisplayShield_;
    int readyPin_ = -1;
    int clockOutPin_ = -1;
    int addressStatePin_ = -1;
    int psramEnPin_ = -1;
    int gpioSelectPin_ = -1;
    int misoPin_ = -1;
    int mosiPin_ = -1;
    int sckPin_ = -1;
    int denPin_ = -1;
    int cacheEnPin_ = -1;
    int reset960Pin_ = -1;
    int int0Pin_ = -1;
    int sclPin_ = -1;
    int sdaPin_ = -1;
    int spiOffset0Pin_ = -1;
    int spiOffset1Pin_ = -1;
    int spiOffset2Pin_ = -1;
    int displayEnPin_ = -1;
    int dcPin_ = -1;
    int sdEnablePin_ = -1;
    int wrPin_ = -1;
    int burstAddress1Pin_ = -1;
    int burstAddress2Pin_ = -1;
    int burstAddress3Pin_ = -1;
    int byteEnable0Pin_ = -1;
    int byteEnable1Pin_ = -1;
    int blastPin_ = -1;
    int failPin_ = -1;
};
template<TargetMCU mcu>
constexpr MCUConfiguration<size_t> BoardDescription = {
        0,
        8, 512,
        32,
        10_MHz,
        5_MHz,
        false,
        false,
        -1, -1, -1, -1, -1, -1, -1,
        -1, -1, -1, -1, -1, -1, -1,
        -1, -1, -1, -1, -1, -1, -1,
        -1, -1, -1, -1, -1, -1, -1
};
template<>
constexpr MCUConfiguration<Pinout1284p> BoardDescription<TargetMCU::ATmega1284p> = {
        16_KB,
        256, 32,
        32,
        10_MHz,
        5_MHz, // due to the current design, we have to run the psram at 5 Mhz
        false,
        true,
        Pinout1284p::PORT_B0,
        Pinout1284p::PORT_B1,
        Pinout1284p::None, //Pinout1284p::PORT_B2, // unused but originally B2
        Pinout1284p::PORT_B3,
        Pinout1284p::PORT_B4,
        Pinout1284p::PORT_B5,
        Pinout1284p::PORT_B6,
        Pinout1284p::PORT_B7,
        Pinout1284p::PORT_D2,
        Pinout1284p::PORT_D3,
        Pinout1284p::PORT_D5,
        Pinout1284p::PORT_D6,
        Pinout1284p::PORT_C0,
        Pinout1284p::PORT_C1,
        Pinout1284p::PORT_C2,
        Pinout1284p::PORT_C3,
        Pinout1284p::PORT_C4,
        Pinout1284p::PORT_C5,
        Pinout1284p::PORT_C6,
        Pinout1284p::PORT_C7,
        Pinout1284p::PORT_A0,
        Pinout1284p::PORT_A1,
        Pinout1284p::PORT_A2,
        Pinout1284p::PORT_A3,
        Pinout1284p::PORT_A4,
        Pinout1284p::PORT_A5,
        Pinout1284p::PORT_A6,
        Pinout1284p::PORT_A7
};
template<>
constexpr MCUConfiguration<PinoutRaspberryPiPico> BoardDescription<TargetMCU::RaspberryPiPico> = {
        264_KB,
        256, 64, // 256, 64 element lines
        64,
        10_MHz,
        5_MHz,
        false,
        false,
        PinoutRaspberryPiPico::GPIO0,
        PinoutRaspberryPiPico::GPIO21,
        PinoutRaspberryPiPico::None,
        PinoutRaspberryPiPico::GPIO1
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
    [[nodiscard]] static constexpr TargetMCU getMCUTarget() noexcept {
#ifdef ARDUINO_AVR_ATmega1284
        return TargetMCU::ATmega1284p;
#elif defined(ARDUINO_RASPBERRY_PI_PICO)
        return TargetMCU::RaspberryPiPico;
#else
    return TargetMCU::Unknown;
#endif
    }
    [[nodiscard]] static constexpr auto onAtmega1284p() noexcept { return getMCUTarget() == TargetMCU::ATmega1284p; }
    [[nodiscard]] static constexpr auto onRaspberryPiPico() noexcept { return getMCUTarget() == TargetMCU::RaspberryPiPico; }
    [[nodiscard]] static constexpr auto onUnknownTarget() noexcept { return getMCUTarget() == TargetMCU::Unknown; }
/**
 * @brief Is there an onboard sdcard slot?
 * @return True if defined via the command line
 */
    [[nodiscard]] static constexpr auto hasBuiltinSDCard() noexcept { return BoardDescription<getMCUTarget()>.hasBuiltinSDCard(); }
    [[nodiscard]] static constexpr auto usesDisplayShield() noexcept { return BoardDescription<getMCUTarget()>.usesDisplayShield(); }
    [[nodiscard]] static constexpr auto getSRAMAmountInBytes() noexcept { return BoardDescription<getMCUTarget()>.getSramAmount(); }
    [[nodiscard]] static constexpr auto oneFourthSRAMAmountInBytes() noexcept { return getSRAMAmountInBytes() / 4; }
    [[nodiscard]] static constexpr auto oneEighthSRAMAmountInBytes() noexcept { return getSRAMAmountInBytes() / 8; }
    [[nodiscard]] static constexpr auto numberOfCacheLines() noexcept { return BoardDescription<getMCUTarget()>.getCacheLineCount(); }
    [[nodiscard]] static constexpr auto cacheLineSize() noexcept { return BoardDescription<getMCUTarget()>.getCacheLineSize(); }
    [[nodiscard]] static constexpr auto maximumNumberOfOpenFilesFromSDCard() noexcept { return BoardDescription<getMCUTarget()>.getMaximumNumberOfOpenFiles(); }
    [[nodiscard]] static constexpr auto runIOExpanderSPIInterfaceAt() noexcept { return BoardDescription<getMCUTarget()>.runIOExpanderSPIInterfaceAt(); }
    [[nodiscard]] static constexpr auto runPSRAMAt() noexcept { return BoardDescription<getMCUTarget()>.runPSRAMAt(); }
    [[nodiscard]] static constexpr auto getReadyPin() noexcept { return BoardDescription<getMCUTarget()>.getReadyPin(); }
    [[nodiscard]] static constexpr auto getClockOutPin() noexcept { return BoardDescription<getMCUTarget()>.getClockOutPin(); }
    [[nodiscard]] static constexpr auto getAddressStatePin() noexcept { return BoardDescription<getMCUTarget()>.getAddressStatePin(); }
    [[nodiscard]] static constexpr auto getPsramEnPin() noexcept { return BoardDescription<getMCUTarget()>.getPsramEnPin(); }
    [[nodiscard]] static constexpr auto getGpioSelectPin() noexcept { return BoardDescription<getMCUTarget()>.getGpioSelectPin(); }
    [[nodiscard]] static constexpr auto getMisoPin() noexcept { return BoardDescription<getMCUTarget()>.getMisoPin(); }
    [[nodiscard]] static constexpr auto getMosiPin() noexcept { return BoardDescription<getMCUTarget()>.getMosiPin(); }
    [[nodiscard]] static constexpr auto getSckPin() noexcept { return BoardDescription<getMCUTarget()>.getSckPin(); }
    [[nodiscard]] static constexpr auto getDenPin() noexcept { return BoardDescription<getMCUTarget()>.getDenPin(); }
    [[nodiscard]] static constexpr auto getCacheEnPin() noexcept { return BoardDescription<getMCUTarget()>.getCacheEnPin(); }
    [[nodiscard]] static constexpr auto getReset960Pin() noexcept { return BoardDescription<getMCUTarget()>.getReset960Pin(); }
    [[nodiscard]] static constexpr auto getInt0Pin() noexcept { return BoardDescription<getMCUTarget()>.getInt0Pin(); }
    [[nodiscard]] static constexpr auto getSclPin() noexcept { return BoardDescription<getMCUTarget()>.getSclPin(); }
    [[nodiscard]] static constexpr auto getSdaPin() noexcept { return BoardDescription<getMCUTarget()>.getSdaPin(); }
    [[nodiscard]] static constexpr auto getSpiOffset0Pin() noexcept { return BoardDescription<getMCUTarget()>.getSpiOffset0Pin(); }
    [[nodiscard]] static constexpr auto getSpiOffset1Pin() noexcept { return BoardDescription<getMCUTarget()>.getSpiOffset1Pin(); }
    [[nodiscard]] static constexpr auto getSpiOffset2Pin() noexcept { return BoardDescription<getMCUTarget()>.getSpiOffset2Pin(); }
    [[nodiscard]] static constexpr auto getDisplayEnPin() noexcept { return BoardDescription<getMCUTarget()>.getDisplayEnPin(); }
    [[nodiscard]] static constexpr auto getDcPin() noexcept { return BoardDescription<getMCUTarget()>.getDcPin(); }
    [[nodiscard]] static constexpr auto getSdEnablePin() noexcept { return BoardDescription<getMCUTarget()>.getSdEnablePin(); }
    [[nodiscard]] static constexpr auto getWrPin() noexcept { return BoardDescription<getMCUTarget()>.getWrPin(); }
    [[nodiscard]] static constexpr auto getBurstAddress1Pin() noexcept { return BoardDescription<getMCUTarget()>.getBurstAddress1Pin(); }
    [[nodiscard]] static constexpr auto getBurstAddress2Pin() noexcept { return BoardDescription<getMCUTarget()>.getBurstAddress2Pin(); }
    [[nodiscard]] static constexpr auto getBurstAddress3Pin() noexcept { return BoardDescription<getMCUTarget()>.getBurstAddress3Pin(); }
    [[nodiscard]] static constexpr auto getByteEnable0Pin() noexcept { return BoardDescription<getMCUTarget()>.getByteEnable0Pin(); }
    [[nodiscard]] static constexpr auto getByteEnable1Pin() noexcept { return BoardDescription<getMCUTarget()>.getByteEnable1Pin(); }
    [[nodiscard]] static constexpr auto getBlastPin() noexcept { return BoardDescription<getMCUTarget()>.getBlastPin(); }
    [[nodiscard]] static constexpr auto getFailPin() noexcept { return BoardDescription<getMCUTarget()>.getFailPin(); }
    [[nodiscard]] static constexpr auto getNonePin() noexcept { return static_cast<int>(BoardDescription<getMCUTarget()>.getNoneSpecifier()); }
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
using UnderlyingPinoutType = decltype(BoardDescription<TargetBoard::getMCUTarget()>.getNoneSpecifier());
#endif //I960SXCHIPSET_MCUPLATFORM_H
