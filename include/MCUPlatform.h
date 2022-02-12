/*
i960SxChipset
Copyright (c) 2020-2022, Joshua Scoggins
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
#include "type_traits.h"
#include "DependentFalse.h"

// comment this out to disable sram cache support
#ifdef __AVR__
using int24_t = __int24;
using uint24_t = __uint24;
#endif


constexpr unsigned long long int operator "" _KB(unsigned long long int value) noexcept { return value * 1024; }
constexpr unsigned long long int operator "" _MB(unsigned long long int value) noexcept { return value * 1024 * 1024; }
constexpr unsigned long long int operator "" _KHz(unsigned long long int value) noexcept { return value * 1000; }
constexpr unsigned long long int operator "" _MHz(unsigned long long int value) noexcept { return value * 1000 * 1000; }
static_assert(2_KHz == 2'000);
static_assert(2_MHz == 2'000'000);
static_assert(20_MHz == 20'000'000);

constexpr byte BitMaskTable_Byte[8] {
        0b0000'0001,
        0b0000'0010,
        0b0000'0100,
        0b0000'1000,
        0b0001'0000,
        0b0010'0000,
        0b0100'0000,
        0b1000'0000,
};

constexpr byte pow2(byte value) noexcept {
    if (value == 0) {
        return 1;
    } else {
        return pow2(value - 1) * 2;
    }
}

constexpr byte numberOfBitsForCount(uint16_t count) noexcept {
    switch (count) {
        case 2: return 1;
        case 4: return 2;
        case 8: return 3;
        case 16: return 4;
        case 32: return 5;
        case 64: return 6;
        case 128: return 7;
        case 256: return 8;
        case 512: return 9;
        case 1024: return 10;
        case 2048: return 11;
        case 4096: return 12;
        case 8192: return 13;
        case 16384: return 14;
        case 32768: return 15;
        default: return 0;
    }
}
constexpr byte getNumberOfBitsForNumberOfEntries(uint16_t count) noexcept { return numberOfBitsForCount(count); }

static_assert(getNumberOfBitsForNumberOfEntries(512/4) == 7);
static_assert(getNumberOfBitsForNumberOfEntries(256/4) == 6);

template<byte numBits>
using ClosestBitValue_t = conditional_t<numBits <= 8, byte,
                                        conditional_t<numBits <= 16, uint16_t,
                                        conditional_t<numBits <= 24, uint24_t,
                                        conditional_t<numBits <= 32, uint32_t, uint64_t>>>>;

static_assert(is_same_v<ClosestBitValue_t<1>, ClosestBitValue_t<4>>);
static_assert(is_same_v<ClosestBitValue_t<4>, byte>);
static_assert(is_same_v<ClosestBitValue_t<10>, uint16_t>);
static_assert(!is_same_v<ClosestBitValue_t<10>, ClosestBitValue_t<4>>);

/**
 * @brief Describe the target microcontroller being used
 */
enum class TargetMCU {
    /**
     * @brief
     */
    ATmega1284p_Type1,
    Unknown,
};
/**
 * @brief Describes some of the important aspects tied to a specific TargetMCU
 */
class MCUConfiguration final {
public:
    /**
     * @brief Describes how important aspects of this mcu are configured
     * @param sramSize How much SRAM this MCU has  (16K, 256K, etc)
     * @param ioExpanderSpeedCap What is the maximum SPI bus speed we can run MCP23S17s at in Hz. If over 10MHz then 10MHz is used!
     * @param psramSpeedCap What is the maximum SPI bus speed we can run the PSRAM64H array at? If over 33MHz then 33MHz is selected
     */
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
/**
 * @brief A generic configuration for an unknown MCU target. This is the fallback case and is designed to generate an error. Designed to be specialized!
 * @tparam mcu The target mcu
 */
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

/**
 * @brief Common interface to query aspects specific to a chipset target. Uses the BoardDescription object to populate fields.
 */
class TargetBoard {
public:
    /**
     * @brief Get the cpu clock frequency in Hz
     * @return The macro F_CPU (or equivalent)
     */
    [[nodiscard]] static constexpr auto getCPUFrequency() noexcept { return F_CPU; }
    /**
     * @brief Return the target microcontroller board currently being used based off of information supplied by the compiler
     * @return The microcontroller board being used.
     */
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
    /**
     * @brief Is this code running on the specific mcu?
     * @tparam mcu The expected mcu
     * @return True if the mcu we are running on is the same as the expected one
     */
    template<TargetMCU mcu>
    [[nodiscard]] static constexpr auto targetMCUIs() noexcept { return getMCUTarget() == mcu; }
    /**
     * @brief Are we running on any of the specified microcontroller targets?
     * @tparam rest the set of targets to check against
     * @return True if we are running any of the specified boards
     */
    template<TargetMCU ... rest>
    [[nodiscard]] static constexpr auto targetMCUIsOneOfThese() noexcept {
        return (targetMCUIs<rest>() || ...);
    }
    /**
     * @brief Are we running on an atmega1284p in a type 1 board configuration?
     * @return True if we are running on a 1284p in a type1 config
     */
    [[nodiscard]] static constexpr auto onAtmega1284p_Type1() noexcept { return targetMCUIs<TargetMCU::ATmega1284p_Type1>(); }
    /**
     * @brief Regardless of configuration are we running on a atmega1284p?
     * @return True if the microcontroller is an atmega1284p
     */
    [[nodiscard]] static constexpr auto onAtmega1284p() noexcept { return targetMCUIsOneOfThese<TargetMCU::ATmega1284p_Type1>(); }
    /**
     * @brief Are we on an undeclared microcontroller target?
     * @return True if we were unable to determine the microcontroller target
     */
    [[nodiscard]] static constexpr auto onUnknownTarget() noexcept { return targetMCUIs<TargetMCU::Unknown>(); }
    /**
     * @brief How much sram does this MCU have for us to maximally play with?
     * @return The amount of sram available to us in bytes
     */
    [[nodiscard]] static constexpr auto getSRAMAmountInBytes() noexcept { return BoardDescription<getMCUTarget()>.getSramAmount(); }
    /**
     * @brief What fastest speed can we run the MCP23S17s at?
     * @return The highest clock speed (in hz) that this target's SPI bus can run the io expanders at
     */
    [[nodiscard]] static constexpr auto runIOExpanderSPIInterfaceAt() noexcept { return BoardDescription<getMCUTarget()>.runIOExpanderSPIInterfaceAt(); }
    /**
     * @brief What fastest speed can we run the PSRAM64Hs at?
     * @return The highest clock speed (in hz) that this target's SPI bus can run the psram at
     */
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
 * @brief A view of a 16-bit number which can be broken up into different components transparently
 */
union SplitWord16 {
    explicit constexpr SplitWord16(uint16_t value = 0) noexcept : wholeValue_(value) { }
    constexpr SplitWord16(uint8_t lower, uint8_t upper) noexcept : bytes{lower, upper} { }
    [[nodiscard]] constexpr auto getWholeValue() const noexcept { return wholeValue_; }
    [[nodiscard]] constexpr auto getLowerHalf() const noexcept { return bytes[0]; }
    [[nodiscard]] constexpr auto getUpperHalf() const noexcept { return bytes[1]; }
    uint16_t wholeValue_ = 0;
    uint8_t bytes[sizeof(uint16_t) / sizeof(uint8_t)];
};

/**
 * @brief A view of a 32-bit number which can be broken up into different components transparently
 */
union SplitWord32 {
    // adding this dropped program size by over 500 bytes!
    explicit constexpr SplitWord32(uint32_t value = 0) noexcept : wholeValue_(value) { }
    /**
     * @brief Build a SplitWord32 from two 16-bit values
     * @param lower The lower half
     * @param upper The upper half
     */
    constexpr SplitWord32(uint16_t lower, uint16_t upper) noexcept : halves{lower, upper} {}
    /**
     * @brief Build a SplitWord32 from four 8-bit values
     * @param lowest Bits 0-7
     * @param lower Bits 8-15
     * @param higher Bits 16-23
     * @param highest Bits 24-31
     */
    constexpr SplitWord32(uint8_t lowest, uint8_t lower, uint8_t higher, uint8_t highest) noexcept : bytes{lowest, lower, higher, highest} {}
    /**
     * @brief Build a SplitWord32 from two SplitWord16s
     * @param lower The lower half of the number
     * @param upper The upper half of the number
     */
    constexpr SplitWord32(const SplitWord16& lower, const SplitWord16& upper) noexcept : words_{lower, upper} { }
    /**
     * @brief Get the backing 32-bit value
     * @return The backing store 32-bit value as is
     */
    [[nodiscard]] constexpr auto getWholeValue() const noexcept { return wholeValue_; }
    /**
     * @brief View this value as a 32-bit signed number
     * @return The backing store as a 32-bit signed number
     */
    [[nodiscard]] constexpr auto getSignedRepresentation() const noexcept { return signedRepresentation_; }
    /**
     * @brief Constexpr method meant to get the target page byte (which is made up of bits 8-15)
     * @return The target page as an 8-bit value
     */
    [[nodiscard]] constexpr auto getTargetPage() const noexcept { return static_cast<byte>(wholeValue_ >> 8); }
    /**
     * @brief Constexpr method meant to allow one to get the most significant byte.
     * @return The most significant byte
     */
    [[nodiscard]] constexpr auto getMostSignificantByte() const noexcept { return static_cast<byte>(wholeValue_ >> 24); }
    /**
     * @brief Get the lower 16-bits as a raw 16-bit number
     * @return The lower half as a plain 16-bit number
     */
    [[nodiscard]] constexpr auto getLowerHalf() const noexcept { return halves[0]; }
    /**
     * @brief Get the upper 16-bits as a raw 16-bit number
     * @return The upper half as a plain 16-bit number
     */
    [[nodiscard]] constexpr auto getUpperHalf() const noexcept { return halves[1]; }
    /**
     * @brief Set the lower 16-bits of the backing store through the use of a SplitWord16
     * @param value The new lower half value
     */
    void setLowerHalf(SplitWord16 value) noexcept { words_[0] = value; }
    /**
     * @brief Set the upper 16-bits of the backing store through the use of a SplitWord16
     * @param value The new upper half value
     */
    void setUpperHalf(SplitWord16 value) noexcept { words_[1] = value; }
    /**
     * @brief View the lower half of this number as a SplitWord16
     * @return The lower half of this number as a SplitWord16
     */
    [[nodiscard]] constexpr auto getLowerWord() const noexcept { return words_[0]; }
    /**
     * @brief View the upper half of this number as a SplitWord16
     * @return The upper half of this number as a SplitWord16
     */
    [[nodiscard]] constexpr auto getUpperWord() const noexcept { return words_[1]; }
    uint32_t wholeValue_ = 0;
    int32_t signedRepresentation_;
    byte bytes[sizeof(uint32_t)];
    uint16_t halves[sizeof(uint32_t) / sizeof(uint16_t)];
    SplitWord16 words_[sizeof(uint32_t) / sizeof(SplitWord16)];
    float floatingPointRepresentation_;
};
/**
 * @brief Sx Load/Store styles that the processor will request
 */
enum class LoadStoreStyle : uint8_t {
    // based off of BE0,BE1 pins
    Full16 = 0,
    Upper8,
    Lower8,
    None,
};
#endif //I960SXCHIPSET_MCUPLATFORM_H
