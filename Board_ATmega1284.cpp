//
// Created by jwscoggins on 3/19/21.
//

#ifdef ARDUINO_AVR_ATmega1284
#include "Board_ATmega1284.h"

void boardSpecificSetup() {
    t.oscillate(static_cast<int>(i960Pinout::Led), 1000, HIGH);
}
void boardSpecificLoopBody() {
    t.update();
}

void setupInterrupts() {
    EIMSK |= 0b101; // enable INT2 and INT0 pin
    EICRA |= 0b100010; // trigger on falling edge
}

Timer t;
#endif