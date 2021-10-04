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

// comment this out to disable sram cache support

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
   IOEXPANDER_PC0,                 \
   IOEXPANDER_PC1,                 \
   IOEXPANDER_PC2,                 \
   IOEXPANDER_PC3,                 \
   IOEXPANDER_PC4,                 \
   IOEXPANDER_PC5,                 \
   IOEXPANDER_PC6,                 \
   IOEXPANDER_PC7,                 \
   IOEXPANDER_PD0,                 \
   IOEXPANDER_PD1,                 \
   IOEXPANDER_PD2,                 \
   IOEXPANDER_PD3,                 \
   IOEXPANDER_PD4,                 \
   IOEXPANDER_PD5,                 \
   IOEXPANDER_PD6,                 \
   IOEXPANDER_PD7,                 \
   IOEXPANDER_PE0,                 \
   IOEXPANDER_PE1,                 \
   IOEXPANDER_PE2,                 \
   IOEXPANDER_PE3,                 \
   IOEXPANDER_PE4,                 \
   IOEXPANDER_PE5,                 \
   IOEXPANDER_PE6,                 \
   IOEXPANDER_PE7,                 \
   IOEXPANDER_PF0,                 \
   IOEXPANDER_PF1,                 \
   IOEXPANDER_PF2,                 \
   IOEXPANDER_PF3,                 \
   IOEXPANDER_PF4,                 \
   IOEXPANDER_PF5,                 \
   IOEXPANDER_PF6,                 \
   IOEXPANDER_PF7,                 \
   IOEXPANDER_PG0,                 \
   IOEXPANDER_PG1,                 \
   IOEXPANDER_PG2,                 \
   IOEXPANDER_PG3,                 \
   IOEXPANDER_PG4,                 \
   IOEXPANDER_PG5,                 \
   IOEXPANDER_PG6,                 \
   IOEXPANDER_PG7,                 \
   IOEXPANDER_PH0,                 \
   IOEXPANDER_PH1,                 \
   IOEXPANDER_PH2,                 \
   IOEXPANDER_PH3,                 \
   IOEXPANDER_PH4,                 \
   IOEXPANDER_PH5,                 \
   IOEXPANDER_PH6,                 \
   IOEXPANDER_PH7


template<typename E>
constexpr bool attachedToIOExpander(E value) noexcept {
    switch (value) {
        case E::IOEXPANDER_PA0:
        case E::IOEXPANDER_PA1:
        case E::IOEXPANDER_PA2:
        case E::IOEXPANDER_PA3:
        case E::IOEXPANDER_PA4:
        case E::IOEXPANDER_PA5:
        case E::IOEXPANDER_PA6:
        case E::IOEXPANDER_PA7:
        case E::IOEXPANDER_PB0:
        case E::IOEXPANDER_PB1:
        case E::IOEXPANDER_PB2:
        case E::IOEXPANDER_PB3:
        case E::IOEXPANDER_PB4:
        case E::IOEXPANDER_PB5:
        case E::IOEXPANDER_PB6:
        case E::IOEXPANDER_PB7:
        case E::IOEXPANDER_PC0:
        case E::IOEXPANDER_PC1:
        case E::IOEXPANDER_PC2:
        case E::IOEXPANDER_PC3:
        case E::IOEXPANDER_PC4:
        case E::IOEXPANDER_PC5:
        case E::IOEXPANDER_PC6:
        case E::IOEXPANDER_PC7:
        case E::IOEXPANDER_PD0:
        case E::IOEXPANDER_PD1:
        case E::IOEXPANDER_PD2:
        case E::IOEXPANDER_PD3:
        case E::IOEXPANDER_PD4:
        case E::IOEXPANDER_PD5:
        case E::IOEXPANDER_PD6:
        case E::IOEXPANDER_PD7:
        case E::IOEXPANDER_PE0:
        case E::IOEXPANDER_PE1:
        case E::IOEXPANDER_PE2:
        case E::IOEXPANDER_PE3:
        case E::IOEXPANDER_PE4:
        case E::IOEXPANDER_PE5:
        case E::IOEXPANDER_PE6:
        case E::IOEXPANDER_PE7:
        case E::IOEXPANDER_PF0:
        case E::IOEXPANDER_PF1:
        case E::IOEXPANDER_PF2:
        case E::IOEXPANDER_PF3:
        case E::IOEXPANDER_PF4:
        case E::IOEXPANDER_PF5:
        case E::IOEXPANDER_PF6:
        case E::IOEXPANDER_PF7:
        case E::IOEXPANDER_PG0:
        case E::IOEXPANDER_PG1:
        case E::IOEXPANDER_PG2:
        case E::IOEXPANDER_PG3:
        case E::IOEXPANDER_PG4:
        case E::IOEXPANDER_PG5:
        case E::IOEXPANDER_PG6:
        case E::IOEXPANDER_PG7:
        case E::IOEXPANDER_PH0:
        case E::IOEXPANDER_PH1:
        case E::IOEXPANDER_PH2:
        case E::IOEXPANDER_PH3:
        case E::IOEXPANDER_PH4:
        case E::IOEXPANDER_PH5:
        case E::IOEXPANDER_PH6:
        case E::IOEXPANDER_PH7:
            return true;
        default:
            return false;
    }
}
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


enum class TargetMCU {
    ATmega1284p_Type1,
    Unknown,
};
class MCUConfiguration final {
public:
    constexpr MCUConfiguration(uint32_t sramSize,
                               uint32_t ioExpanderSpeedCap,
                               uint32_t psramSpeedCap
    ) noexcept : sramAmount_(sramSize),
                 ioExpanderPeripheralSpeed_(ioExpanderSpeedCap > 10_MHz ? 10_MHz : ioExpanderSpeedCap),
                 psramSpeedCap_(psramSpeedCap > 33_MHz ? 33_MHz : psramSpeedCap) { }

    [[nodiscard]] constexpr uint32_t getSramAmount() const noexcept { return sramAmount_; }
    [[nodiscard]] constexpr auto runIOExpanderSPIInterfaceAt() const noexcept  { return ioExpanderPeripheralSpeed_; }
    [[nodiscard]] constexpr auto runPSRAMAt() const noexcept { return psramSpeedCap_; }
private:
    uint32_t sramAmount_;
    uint32_t ioExpanderPeripheralSpeed_;
    uint32_t psramSpeedCap_;
};
template<TargetMCU mcu>
constexpr MCUConfiguration BoardDescription = {
        0,
        10_MHz,
        8_MHz
};
template<>
constexpr MCUConfiguration BoardDescription<TargetMCU::ATmega1284p_Type1> = {
        16_KB,
        10_MHz,
        5_MHz // due to the current design, we have to run the psram at 5 Mhz
};

class TargetBoard {
public:
    [[nodiscard]] static constexpr auto getCPUFrequency() noexcept { return F_CPU; }
    [[nodiscard]] static constexpr TargetMCU getMCUTarget() noexcept {
#ifdef ARDUINO_AVR_ATmega1284
#ifdef CHIPSET_TYPE1
        return TargetMCU::ATmega1284p_Type1;
#else
        return TargetMCU::Unknown;
#endif
#else
    return TargetMCU::Unknown;
#endif
    }
    [[nodiscard]] static constexpr auto onAtmega1284p_Type1() noexcept { return getMCUTarget() == TargetMCU::ATmega1284p_Type1; }
    [[nodiscard]] static constexpr auto onAtmega1284p() noexcept { return onAtmega1284p_Type1(); }
    [[nodiscard]] static constexpr auto onUnknownTarget() noexcept { return getMCUTarget() == TargetMCU::Unknown; }
    [[nodiscard]] static constexpr auto getSRAMAmountInBytes() noexcept { return BoardDescription<getMCUTarget()>.getSramAmount(); }
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
    uint8_t bytes[sizeof(uint16_t) / sizeof(uint8_t)];
};
union SplitWord32 {
    // adding this dropped program size by over 500 bytes!
    explicit constexpr SplitWord32(uint32_t value = 0) noexcept : wholeValue_(value) { }
    constexpr SplitWord32(uint8_t lowest, uint8_t lower, uint8_t higher, uint8_t highest) noexcept : bytes{lowest, lower, higher, highest} {}
    [[nodiscard]] constexpr auto getWholeValue() const noexcept { return wholeValue_; }
    [[nodiscard]] constexpr auto getSignedRepresentation() const noexcept { return signedRepresentation_; }
    uint32_t wholeValue_ = 0;
    int32_t signedRepresentation_;
    byte bytes[sizeof(uint32_t)];
    uint16_t halves[sizeof(uint32_t) / sizeof(uint16_t)];
    SplitWord16 words_[sizeof(uint32_t) / sizeof(SplitWord16)];
};
#endif //I960SXCHIPSET_MCUPLATFORM_H
