//
// Created by jwscoggins on 3/19/21.
//

#ifndef ARDUINO_BOARD_ARDUINOUNO_H
#define ARDUINO_BOARD_ARDUINOUNO_H
#include <Arduino.h>
#include "BoardSpecific.h"
enum class i960Pinout : decltype(A0) {
    RX0 = 0,
    TX0,
    AS_,
    DEN_,
    Ready,
    Int0_,
    W_R_,
    Reset960,
    BLAST_,
    FAIL,
    GPIOSelect,
    MOSI,
    MISO,
    SCK,
    Count,
    SPI_BUS_EN = A0,
    Analog1 = A1,
    Analog2 = A2,
    Analog3 = A3,
    Analog4 = A4,
    Analog5 = A5,
    SCL = A5,
    SDA = A4,
    Led = LED_BUILTIN,
};
static_assert(static_cast<i960Pinout::Count> == 14);
#define AS_ISR INT0_vect
#define DEN_ISR INT1_vect
#endif //ARDUINO_BOARD_ARDUINOUNO_H
