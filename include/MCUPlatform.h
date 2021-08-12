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
#ifdef ARDUINO_AVR_ATmega1284
#define PACKED_ATTRIBUTE
#else
#define PACKED_ATTRIBUTE __attribute__((packed))
#endif
#ifdef ARDUINO_AVR_ATmega1284
#ifndef PIN_SERIAL_RX
#define PIN_SERIAL_RX 8
#endif // end !defined(PIN_SERIAL_RX)
#ifndef PIN_SERIAL_TX
#define PIN_SERIAL_TX 9
#endif // end !defined(PIN_SERIAL_TX)
#endif // end defined(ARDUINO_AVR_ATmega1284)
#define DEFINE_PINOUT_REQUIREMENTS \
    Count,                         \
    None,                          \
    IOEXPANDER_PA0,                \
    IOEXPANDER_PA1,                \
    IOEXPANDER_PA2,                \
   IOEXPANDER_PA3,                 \
   IOEXPANDER_PA4,                 \
   IOEXPANDER_PA5,                 \
   IOEXPANDER_PA6,                 \
   IOEXPANDER_PA7,                 \
   IOEXPANDER_PB0,                 \
   IOEXPANDER_PB1,                 \
   IOEXPANDER_PB2,                 \
   IOEXPANDER_PB3,                 \
   IOEXPANDER_PB4,                 \
   IOEXPANDER_PB5,                 \
   IOEXPANDER_PB6,                 \
   IOEXPANDER_PB7,                 \
   NODISPLAY,                      \
   NODC,                           \
   NOCACHE_EN_


enum class UndefinedPinout : int {
    DEFINE_PINOUT_REQUIREMENTS,
    MISO,
    MOSI,
    SCK,
    CS,
    SCL,
    SDA,
    CLKO,
    READY_,
    DEN_,
    PSRAM_EN_,
    CACHE_EN_,
    RESET960_,
    Int0_,
    SPI_OFFSET0,
    SPI_OFFSET1,
    SPI_OFFSET2,
    SD_EN_,
    W_R_,
    BA1,
    BA2,
    BA3,
    BE0_,
    BE1_,
    BLAST_,
    FAIL960,
    DISPLAY_EN_,
    DC,
};
enum class Pinout1284p : int {
    // this is described in digial pin order!
    // leave this one alone
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
    DEFINE_PINOUT_REQUIREMENTS,
    RX0 = PIN_SERIAL_RX,
    TX0 = PIN_SERIAL_TX,
    CS = PIN_SPI_SS,
    SCK = PIN_SPI_SCK,
    MOSI = PIN_SPI_MOSI,
    MISO = PIN_SPI_MISO,
    SCL = PIN_WIRE_SCL,
    SDA = PIN_WIRE_SDA,
    CLKO = PORT_B1,
    READY_ = PORT_B0,
    PSRAM_EN_ = PORT_B3,
    DEN_ = PORT_D2,
    CACHE_EN_ = PORT_D3,
    RESET960_ = PORT_D5,
    Int0_ = PORT_D6,
    SPI_OFFSET0 = PORT_C2,
    SPI_OFFSET1 = PORT_C3,
    SPI_OFFSET2 = PORT_C4,
    DISPLAY_EN_ = PORT_C5,
    DC = PORT_C6,
    SD_EN_ = PORT_C7,
    W_R_ = PORT_A0,
    BA1 = PORT_A1,
    BA2 = PORT_A2,
    BA3 = PORT_A3,
    BE0_ = PORT_A4,
    BE1_ = PORT_A5,
    BLAST_ = PORT_A6,
    FAIL960 = PORT_A7,
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
    DEFINE_PINOUT_REQUIREMENTS,
    RX0 = PIN_SERIAL_RX,
    TX0 = PIN_SERIAL_TX,
    MISO = PIN_SPI_MISO,
    MOSI = PIN_SPI_MOSI,
    SCK = PIN_SPI_SCK,
    CS = PIN_SPI_SS,
    SCL = PIN_WIRE_SCL,
    SDA = PIN_WIRE_SDA,
    SD_EN_ = GPIO8,
    SPI_OFFSET0 = GPIO9,
    SPI_OFFSET1 = GPIO10,
    SPI_OFFSET2 = GPIO11,
    PSRAM_EN_ = GPIO12,
    // GPIO13 Unused
    FAIL960 = GPIO14,
    BLAST_ = GPIO15,
    RESET960_ = GPIO16,
    W_R_ = GPIO17,
    Int0_ = GPIO18,
    READY_ = GPIO19,
    DEN_ = GPIO20,
    CLKO = GPIO21,
    BA1 = IOEXPANDER_PA0,
    BA2 = IOEXPANDER_PA1,
    BA3 = IOEXPANDER_PA2,
    BE0_ = IOEXPANDER_PA3,
    BE1_ = IOEXPANDER_PA4,
    LED = GPIO25,
    DISPLAY_EN_ = NODISPLAY,
    DC = NODC,
    CACHE_EN_ = NOCACHE_EN_,
};
static_assert(static_cast<int>(PinoutRaspberryPiPico::Count) == 30, "Raspberry Pi Pico Has 30 GPIO");

template<typename E>
constexpr bool isValidPin(E pin) noexcept {
    return static_cast<int>(pin) < static_cast<int>(E::Count) &&
           static_cast<int>(pin) >= 0;
}
template<auto pin>
constexpr bool isValidPin_v = isValidPin(pin);
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
                               bool usesDisplayShield
    ) noexcept : sramAmount_(sramSize),
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
    [[nodiscard]] constexpr auto getReadyPin() const noexcept { return static_cast<int>(T::READY_); }
    [[nodiscard]] constexpr auto getClockOutPin() const noexcept { return static_cast<int>(T::CLKO); }
    [[nodiscard]] constexpr auto getPsramEnPin() const noexcept { return static_cast<int>(T::PSRAM_EN_); }
    [[nodiscard]] constexpr auto getGpioSelectPin() const noexcept { return static_cast<int>(T::CS); }
    [[nodiscard]] constexpr auto getMisoPin() const noexcept { return static_cast<int>(T::MISO); }
    [[nodiscard]] constexpr auto getMosiPin() const noexcept { return static_cast<int>(T::MOSI); }
    [[nodiscard]] constexpr auto getSckPin() const noexcept { return static_cast<int>(T::SCK); }
    [[nodiscard]] constexpr auto getDenPin() const noexcept { return static_cast<int>(T::DEN_); }
    [[nodiscard]] constexpr auto getCacheEnPin() const noexcept { return static_cast<int>(T::CACHE_EN_); }
    [[nodiscard]] constexpr auto getReset960Pin() const noexcept { return static_cast<int>(T::RESET960_); }
    [[nodiscard]] constexpr auto getInt0Pin() const noexcept { return static_cast<int>(T::Int0_); }
    [[nodiscard]] constexpr auto getSclPin() const noexcept { return static_cast<int>(T::SCL); }
    [[nodiscard]] constexpr auto getSdaPin() const noexcept { return static_cast<int>(T::SDA); }
    [[nodiscard]] constexpr auto getSpiOffset0Pin() const noexcept { return static_cast<int>(T::SPI_OFFSET0); }
    [[nodiscard]] constexpr auto getSpiOffset1Pin() const noexcept { return static_cast<int>(T::SPI_OFFSET1); }
    [[nodiscard]] constexpr auto getSpiOffset2Pin() const noexcept { return static_cast<int>(T::SPI_OFFSET2); }
    [[nodiscard]] constexpr auto getDisplayEnPin() const noexcept { return static_cast<int>(T::DISPLAY_EN_); }
    [[nodiscard]] constexpr auto getDcPin() const noexcept { return static_cast<int>(T::DC); }
    [[nodiscard]] constexpr auto getSdEnablePin() const noexcept { return static_cast<int>(T::SD_EN_); }
    [[nodiscard]] constexpr auto getWrPin() const noexcept { return static_cast<int>(T::W_R_); }
    [[nodiscard]] constexpr auto getBurstAddress1Pin() const noexcept { return static_cast<int>(T::BA1); }
    [[nodiscard]] constexpr auto getBurstAddress2Pin() const noexcept { return static_cast<int>(T::BA2); }
    [[nodiscard]] constexpr auto getBurstAddress3Pin() const noexcept { return static_cast<int>(T::BA3); }
    [[nodiscard]] constexpr auto getByteEnable0Pin() const noexcept { return static_cast<int>(T::BE0_); }
    [[nodiscard]] constexpr auto getByteEnable1Pin() const noexcept { return static_cast<int>(T::BE1_); }
    [[nodiscard]] constexpr auto getBlastPin() const noexcept { return static_cast<int>(T::BLAST_); }
    [[nodiscard]] constexpr auto getFailPin() const noexcept { return static_cast<int>(T::FAIL960); }
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
};
template<TargetMCU mcu>
constexpr MCUConfiguration<UndefinedPinout> BoardDescription = {
        0,
        8, 512,
        32,
        10_MHz,
        8_MHz,
        false,
        false
};
template<>
constexpr MCUConfiguration<Pinout1284p> BoardDescription<TargetMCU::ATmega1284p> = {
        16_KB,
        256, 32,
        32,
        10_MHz,
        8_MHz, // due to the current design, we have to run the psram at 5 Mhz
        false,
        true
};
template<>
constexpr MCUConfiguration<PinoutRaspberryPiPico> BoardDescription<TargetMCU::RaspberryPiPico> = {
        264_KB,
        256, 32, // 256, 32 element lines
        64,
        10_MHz,
        8_MHz,
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
