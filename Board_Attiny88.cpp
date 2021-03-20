//
// Created by jwscoggins on 3/20/21.
//
#ifdef ARDUINO_AVR_ATTINYX8
#include "Board_Attiny88.h"

void boardSpecificSetup() {
}
void boardSpecificLoopBody() {
}

void setupInterrupts() {
#if 0
    EIMSK |= 0b11; // enable INT1 and INT0 pin
    EICRA |= 0b1010; // trigger on falling edge
#endif
}
#endif
