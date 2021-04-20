/**
 * @file 
 * Additional classes and functions that make working with pins easier
 * @copyright
 * Copyright (c) 2019 Joshua Scoggins 
 * 
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * 
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */
#ifndef LIB_ICS_X74SERIES_H__
#define LIB_ICS_X74SERIES_H__
#include "Arduino.h"
#include "../core/concepts.h"
namespace bonuspin
{
/**
 * Makes working with HC595 chips easier
 * @tparam ST_CP the pin connected to ST_CP of 74HC595
 * @tparam SH_CP the pin connected to SH_CP of 74HC595
 * @tparam DS the pin connected to DS of 74HC595
 * @todo add support for the OE line to be controlled if desired
 */
template<int ST_CP, int SH_CP, int DS>
class HC595 {
    public:
        static_assert(ST_CP != DS, "The latch and data pins are defined as the same pins!");
        static_assert(ST_CP != SH_CP, "The clock and latch pins are defined as the same pins!!");
        static_assert(SH_CP != DS, "The clock and data pins are defined as the same pins!");
        using Self = HC595<ST_CP, SH_CP, DS>;
        using LatchHolder = HoldPinLow<ST_CP>;
    public:
        /**
         * Setup the pins associated with this device
         */
        HC595() {
            setupPins();
        }

        ~HC595() = default;
        constexpr auto getLatchPin() const noexcept { return ST_CP; }
        constexpr auto getClockPin() const noexcept { return SH_CP; }
        constexpr auto getDataPin() const noexcept { return DS; }
        /**
         * Provided in case pins get reset in between init and the setup
         * function
         */
        void setupPins() {
            pinMode(ST_CP, OUTPUT);
            pinMode(SH_CP, OUTPUT);
            pinMode(DS, OUTPUT);
        }
        /**
         * Hold the latch low and shift out a single byte of data!
         */
        void shiftOut(byte value) noexcept {
            LatchHolder latch;
            ::shiftOut(DS, SH_CP, MSBFIRST, value);
        }
        /**
         * Hold the latch low and shift out two bytes of data!
         */
        void shiftOut(uint16_t value) noexcept {
            LatchHolder latch;
            ::shiftOut(DS, SH_CP, MSBFIRST, value >> 8);
            ::shiftOut(DS, SH_CP, MSBFIRST, value);
        }
        void shiftOut(uint8_t lower, uint8_t upper) noexcept {
            LatchHolder latch;
            ::shiftOut(DS, SH_CP, MSBFIRST, upper);
            ::shiftOut(DS, SH_CP, MSBFIRST, lower);

        }
        /**
         * Hold the latch low and shift out two bytes of data!
         */
        void shiftOut(int16_t value) noexcept {
            LatchHolder latch;
            ::shiftOut(DS, SH_CP, MSBFIRST, (value >> 8) & 0x00FF);
            ::shiftOut(DS, SH_CP, MSBFIRST, value);
        }
        /**
         * Hold the latch low and shift 4 bytes of data!
         */
        void shiftOut(uint32_t value) noexcept {
            LatchHolder latch;
            ::shiftOut(DS, SH_CP, MSBFIRST, (value >> 24) & 0xFF);
            ::shiftOut(DS, SH_CP, MSBFIRST, (value >> 16) & 0xFF);
            ::shiftOut(DS, SH_CP, MSBFIRST, (value >> 8) & 0xFF);
            ::shiftOut(DS, SH_CP, MSBFIRST, value & 0xFF);
        }
        /**
         * Hold the latch low and shift 4 bytes of data!
         */
        void shiftOut(int32_t value) noexcept {
            LatchHolder latch;
            ::shiftOut(DS, SH_CP, MSBFIRST, (value >> 24) & 0xFF);
            ::shiftOut(DS, SH_CP, MSBFIRST, (value >> 16) & 0xFF);
            ::shiftOut(DS, SH_CP, MSBFIRST, (value >> 8) & 0xFF);
            ::shiftOut(DS, SH_CP, MSBFIRST, value & 0xFF);
        }
        /**
         * Hold the latch low and shift 8 bytes of data!
         */
        void shiftOut(uint64_t value) noexcept {
            LatchHolder latch;
            ::shiftOut(DS, SH_CP, MSBFIRST, (value >> 56) & 0xFF);
            ::shiftOut(DS, SH_CP, MSBFIRST, (value >> 48) & 0xFF);
            ::shiftOut(DS, SH_CP, MSBFIRST, (value >> 40) & 0xFF);
            ::shiftOut(DS, SH_CP, MSBFIRST, (value >> 32) & 0xFF);
            ::shiftOut(DS, SH_CP, MSBFIRST, (value >> 24) & 0xFF);
            ::shiftOut(DS, SH_CP, MSBFIRST, (value >> 16) & 0xFF);
            ::shiftOut(DS, SH_CP, MSBFIRST, (value >> 8) & 0xFF);
            ::shiftOut(DS, SH_CP, MSBFIRST, value);
        }
        /**
         * Hold the latch low and shift 8 bytes of data!
         */
        void shiftOut(int64_t value) noexcept {
            LatchHolder latch;
            ::shiftOut(DS, SH_CP, MSBFIRST, (value >> 56) & 0xFF);
            ::shiftOut(DS, SH_CP, MSBFIRST, (value >> 48) & 0xFF);
            ::shiftOut(DS, SH_CP, MSBFIRST, (value >> 40) & 0xFF);
            ::shiftOut(DS, SH_CP, MSBFIRST, (value >> 32) & 0xFF);
            ::shiftOut(DS, SH_CP, MSBFIRST, (value >> 24) & 0xFF);
            ::shiftOut(DS, SH_CP, MSBFIRST, (value >> 16) & 0xFF);
            ::shiftOut(DS, SH_CP, MSBFIRST, (value >> 8) & 0xFF);
            ::shiftOut(DS, SH_CP, MSBFIRST, value);
        }
        template<typename T, typename ... Args>
        void shiftOutMultiple(T current, Args&& ... rest) noexcept {
            shiftOut(current);
            if (sizeof...(rest) > 0) {
                shiftOutMultiple(rest...);
            }
        }
        template<typename T>
        Self& operator<<(T value) noexcept {
            shiftOut(value);
            return *this;
        }
};

template<int selA, int selB, int selC, int enablePin = -1>
class HC138 {
    public:
        static_assert(selA != selB, "SelA and SelB are the same!");
        static_assert(selA != selC, "SelA and SelC are the same!");
        static_assert(selA != enablePin, "SelA and enablePin are the same!");
        static_assert(selB != selC, "SelB and SelC are the same!");
        static_assert(selB != enablePin, "SelB and enablePin are the same!");
        static_assert(selC != enablePin, "SelC and enablePin are the same!");
        using DigitalPinSignal = decltype(HIGH);
        using TemporaryDisabler = HoldPinLow<enablePin>;
        using TemporaryEnabler = HoldPinHigh<enablePin>;
        using Self = HC138<selA, selB, selC, enablePin>;
    public:
        HC138() {
            setupPins();
        }
        constexpr auto getSelAPin() const noexcept { return selA; }
        constexpr auto getSelBPin() const noexcept { return selB; }
        constexpr auto getSelCPin() const noexcept { return selC; }
        constexpr auto getEnablePin() const noexcept { return enablePin; }
        void setupPins() {
            pinMode(selA, OUTPUT);
            pinMode(selB, OUTPUT);
            pinMode(selC, OUTPUT);
            pinMode(enablePin, OUTPUT);

            digitalWrite(selA, HIGH);
            digitalWrite(selB, HIGH);
            digitalWrite(selB, HIGH);
            digitalWrite(enablePin, LOW); // turn off the connection to the chip for the 
            // time being

        }
        template<DigitalPinSignal a, DigitalPinSignal b, DigitalPinSignal c>
        void generateSignal() {
            // turn off the chip while we send our signal and then reactivate
            // it once we have the pins setup for the right stuff
            TemporaryDisabler disabler;
            digitalWrite(selA, a);
            digitalWrite(selB, b);
            digitalWrite(selC, c);
        }
        template<byte signal>
        void enableLine() {
            switch (signal) {
                case 0:
                    generateSignal<LOW, LOW, LOW>();
                    break;
                case 1:
                    generateSignal<HIGH, LOW, LOW>();
                    break;
                case 2:
                    generateSignal<LOW, HIGH, LOW>();
                    break;
                case 3:
                    generateSignal<HIGH, HIGH, LOW>();
                    break;
                case 4:
                    generateSignal<LOW, LOW, HIGH>();
                    break;
                case 5: 
                    generateSignal<HIGH, LOW, HIGH>();
                    break;
                case 6:
                    generateSignal<LOW, HIGH, HIGH>();
                    break;
                case 7:
                    generateSignal<HIGH, HIGH, HIGH>();
                    break;
                default:
                    enableLine<signal & 0x7>();
                    break;
            }
        }
        /**
         * Non compile time deduced version of enableLine.
         * @param signal the line to enable
         */
        void enableLine(byte signal) {
            switch (signal) {
                case 0:
                    generateSignal<LOW, LOW, LOW>();
                    break;
                case 1:
                    generateSignal<HIGH, LOW, LOW>();
                    break;
                case 2:
                    generateSignal<LOW, HIGH, LOW>();
                    break;
                case 3:
                    generateSignal<HIGH, HIGH, LOW>();
                    break;
                case 4:
                    generateSignal<LOW, LOW, HIGH>();
                    break;
                case 5: 
                    generateSignal<HIGH, LOW, HIGH>();
                    break;
                case 6:
                    generateSignal<LOW, HIGH, HIGH>();
                    break;
                case 7:
                    generateSignal<HIGH, HIGH, HIGH>();
                    break;
                default:
                    enableLine(signal & 0x7);
                    break;
            }
        }
        void enableChip() {
            digitalWrite(enablePin, HIGH);
        }
        void disableChip() {
            digitalWrite(enablePin, LOW);
        }


};

template<int input, int clock, int shld, int enable>
class HC165 {
    public:
        static_assert(input != clock, "input and clock pins are equal");
        static_assert(input != shld, "input and shld pins are equal");
        static_assert(input != enable, "input and enable pins are equal");
        static_assert(clock != shld, "clock and shld pins are the same!");
        static_assert(clock != enable, "clock and enable pins are the same!");
        static_assert(shld != enable, "shld and enable pins are the same!");
        using ChipEnabler = HoldPinHigh<enable>;
        using ParallelLoadAction = HoldPinLow<shld>;
        using ClockPulser = HoldPinHigh<enable>;
        static constexpr auto pulseWidthUSec = 5;
    public:
        HC165() {
            setupPins();
        }
        constexpr auto getInputPin() const noexcept { return input; }
        constexpr auto getClockPin() const noexcept { return clock; }
        constexpr auto getSHLDPin() const noexcept { return shld; }
        constexpr auto getEnablePin() const noexcept { return enable; }

        void setupPins() {
            pinMode(input, INPUT);
            pinMode(clock, OUTPUT);
            pinMode(shld, OUTPUT);
            pinMode(enable, OUTPUT);

            digitalWrite(clock, LOW);
            digitalWrite(shld, HIGH);
        }
        void parallelLoad() {
            ChipEnabler activateChip;
            {
                ParallelLoadAction pload;
                delayMicroseconds(pulseWidthUSec);
            }
        }
        void pulseClock() {
            ClockPulser pulser;
            delayMicroseconds(pulseWidthUSec);
        }

        byte shiftIn() noexcept {
            parallelLoad();
            auto bytesVal = 0;
            for (auto i = 0; i < 8; ++i) {
                /// MSBFIRST
                bytesVal |= (digitalRead(input) << (7 - i));
                pulseClock();
            }
            return bytesVal;
        }

};
template<int ST_CP, int SH_CP, int DS>
using SN74HC595 = HC595<ST_CP, SH_CP, DS>;
} // end namespace bonuspin
#endif // end LIB_ICS_X74SERIES_H__
