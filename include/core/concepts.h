/**
 * @file 
 * Basic concepts
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
#ifndef LIB_CORE_CONCEPTS_H__
#define LIB_CORE_CONCEPTS_H__
#include "Arduino.h"
void operator delete (void* ptr, unsigned int sz) noexcept;
namespace bonuspin
{
/**
 * RAII-style class that handles holding a digital pin to a specific value for the
 * lifetime of this object; this object stores nothing so it should not consume
 * space at runtime.
 * @tparam pin The pin to be held
 * @tparam holdPinAs the value to hold the pin to during the lifetime of the object
 * @tparam restorePinTo the value to return the pin to when this object goes out of scope.
 */
template<int pin, decltype(LOW) holdPinAs, decltype(HIGH) restorePinTo>
class DigitalPinHolder final
{
    public:
        DigitalPinHolder() {
            if constexpr (pin >= 0) {
                digitalWrite(pin, holdPinAs);
            }
        }
        ~DigitalPinHolder() {
            if constexpr (pin >= 0) {
                digitalWrite(pin, restorePinTo);
            }
        }
        inline constexpr decltype(pin) getPin() const noexcept { return pin; }
        inline constexpr decltype(holdPinAs) getHeldPinValue() const noexcept { return holdPinAs; }
        inline constexpr decltype(restorePinTo) getPinValueOnDestruction() const noexcept { return restorePinTo; }
        inline constexpr auto willNotFire() const noexcept { return pin < 0; }
        DigitalPinHolder(const DigitalPinHolder&) = delete;
        DigitalPinHolder(DigitalPinHolder&&) = delete;
        DigitalPinHolder& operator=(const DigitalPinHolder&) = delete;
        DigitalPinHolder& operator=(DigitalPinHolder&&) = delete;
};

template<int pin>
using HoldPinLow = DigitalPinHolder<pin, LOW, HIGH>;
template<int pin>
using HoldPinHigh = DigitalPinHolder<pin, HIGH, LOW>;


} // end namespace bonuspin
#endif // end LIB_CORE_CONCEPTS_H__
