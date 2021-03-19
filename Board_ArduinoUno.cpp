//
// Created by jwscoggins on 3/19/21.
//
#ifdef ARDUINO_AVR_UNO
#include "Board_ArduinoUno.h"

void boardSpecificSetup() {
}
void boardSpecificLoopBody() {
}

void setupInterrupts() {
    EIMSK |= 0b11; // enable INT1 and INT0 pin
    EICRA |= 0b1010; // trigger on falling edge
}

#endif