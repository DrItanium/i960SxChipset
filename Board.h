//
// Created by jwscoggins on 3/19/21.
//

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
#else
#error "Unknown board!"
#endif
#ifndef AS_ISR
#error "AS_ Must be bound to an interrupt service routine!"
#endif

#ifndef DEN_ISR
#error "DEN_ Must be bound to an interrupt service routine!"
#endif

constexpr bool hasSoftwareSerial() noexcept {
#ifdef SOFTWARE_IS_SERIAL
    return true;
#else
    return false;
#endif
}
constexpr auto getCPUFrequency() noexcept { return F_CPU; }
#endif //ARDUINO_BOARD_H
