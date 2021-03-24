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

#ifndef ARDUINO_BOARD_H
#define ARDUINO_BOARD_H
#include "Architecture.h"
#include "BoardSpecific.h"
#ifdef CPU_IS_AVR
#ifdef __NO_INTERRUPTS__
#error "i960 Chipset requires interrupts on AVR"
#endif
#endif
#ifdef ARDUINO_AVR_ATmega1284
#include "Board_ATmega1284.h"
#elif defined(ARDUINO_AVR_UNO)
#include "Board_ArduinoUno.h"
#elif defined(ARDUINO_AVR_ATTINYX61)
#include "Board_Attiny861.h"
#elif defined(ARDUINO_AVR_ATTINYX8)
#include "Board_Attiny88.h"
#else
#error "Unknown board!"
#endif
#ifndef AS_ISR
#error "AS_ Must be bound to an interrupt service routine!"
#endif

#ifndef DEN_ISR
#error "DEN_ Must be bound to an interrupt service routine!"
#endif
constexpr bool hasSerial() noexcept {
#ifndef NO_SERIAL
    return true;
#else
    return false;
#endif
}
constexpr bool hasSoftwareSerial() noexcept {
#ifdef SOFTWARE_IS_SERIAL
    return true;
#else
    return false;
#endif
}
constexpr auto getCPUFrequency() noexcept { return F_CPU; }
#endif //ARDUINO_BOARD_H
