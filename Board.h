//
// Created by jwscoggins on 3/19/21.
//

#ifndef ARDUINO_BOARD_H
#define ARDUINO_BOARD_H
#include "BoardSpecific.h"

#ifdef ARDUINO_AVR_ATmega1284
#include "Board_ATmega1284.h"
#elif defined(ARDUINO_AVR_UNO)
#include "Board_ArduinoUno.h"
#else
#error "Unknown board!"
#endif

#endif //ARDUINO_BOARD_H
