//
// Created by jwscoggins on 3/20/21.
//

#ifndef ARDUINO_BOARD_ATTINY88_H
#define ARDUINO_BOARD_ATTINY88_H
#include <Arduino.h>
#include "BoardSpecific.h"
enum class i960Pinout : decltype(A0) {
    Ready = 0,
    Int0_,
    AS_,
    DEN_,
    SPI_BUS_EN,
    W_R_,
    // Software Serial
    TXD,
    RXD,
    Reset960,
    BLAST_,
    GPIOSelect,
    MOSI,
    MISO,
    SCK,
    FAIL,
    Digital15,
    Digital16,
    Analog0,
    Analog1,
    Analog2,
    Analog3,
    Analog4,
    Analog5,
    Count,
    SCL = A5,
    SDA = A4,
};
static_assert(static_cast<int>(i960Pinout::Count) == 23);
#endif //ARDUINO_BOARD_ATTINY88_H
