/**
 * @file 
 * LED Related Routines
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
#ifndef LIB_CORE_LEDS_H__
#define LIB_CORE_LEDS_H__
#include "Arduino.h"
#include "concepts.h"
namespace bonuspin
{
    /** 
     * use tag dispatch
     */
    class CommonAnodeLED_t final { };
    class CommonCathodeLED_t final { };
    class DigitalWrite_t final { };
    class AnalogWrite_t final { };

    template<int pin>
    void emitIntensity(int value, DigitalWrite_t) { 
        digitalWrite(pin, value);
    }

    template<int pin>
    void emitIntensity(int value, AnalogWrite_t) {
        analogWrite(pin, value);
    }

    constexpr uint8_t redComponent(uint32_t color) noexcept {
        return (color & 0xFF0000) >> 16;
    }
    constexpr uint8_t greenComponent(uint32_t color) noexcept {
        return (color & 0x00FF00) >> 8;
    }
    constexpr uint8_t blueComponent(uint32_t color) noexcept {
        return (color & 0xFF);
    }
    template<int rPin, int gPin, int bPin>
    void emitColor(uint32_t color, CommonAnodeLED_t) {
        emitIntensity<rPin>( ~redComponent(color), AnalogWrite_t());
        emitIntensity<gPin>( ~greenComponent(color), AnalogWrite_t());
        emitIntensity<bPin>( ~blueComponent(color), AnalogWrite_t());
    }
    template<int rPin, int gPin, int bPin>
    void emitColor(uint32_t color, CommonCathodeLED_t) {
        emitIntensity<rPin>( redComponent(color), AnalogWrite_t());
        emitIntensity<gPin>( greenComponent(color), AnalogWrite_t());
        emitIntensity<bPin>( blueComponent(color), AnalogWrite_t());
    }



} // end namespace bonuspin
#endif // end LIB_CORE_LEDS_H__
