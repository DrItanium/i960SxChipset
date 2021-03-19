//
// Created by jwscoggins on 3/19/21.
//

#ifndef ARDUINO_BOARD_ARDUINOUNO_H
#define ARDUINO_BOARD_ARDUINOUNO_H
#include <Arduino.h>
#include "BoardSpecific.h"
enum class i960Pinout : decltype(A0) {
    Led = LED_BUILTIN,
    SCL = A5,
    SDA = A4,
    AS_ = 2,
    DEN_ = 3,
    Ready = 4,
    Int0_ = 5,
    W_R_ = 6,
    Reset960 = 7,
    BLAST_ = 8,
    FAIL = 9,
    GPIOSelect = 10,
    MOSI = 11,
    MISO = 12,
    SCK = 13,
    SPI_BUS_EN = A0,
    Analog1 = A1,
    Analog2 = A2,
    Analog3 = A3,
    Analog4 = A4,
    Analog5 = A5,
};
#define AS_ISR INT0_vect
#define DEN_ISR INT1_vect
#endif //ARDUINO_BOARD_ARDUINOUNO_H
