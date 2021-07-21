//
// Created by jwscoggins on 7/20/21.
//

#ifndef CHIPSET_PINOUT_MEGA2560_H
#define CHIPSET_PINOUT_MEGA2560_H
#ifdef ARDUINO_AVR_MEGA2560
#include <Arduino.h>
enum class i960Pinout {
    // this is described in digial pin order!
    // leave this one alone
    // PORT B
    GPIOSelect = PIN_SPI_SS,
    MOSI = PIN_SPI_MOSI,          // reserved
    MISO = PIN_SPI_MISO,          // reserved
    SCK = PIN_SPI_SCK,          // reserved
// PORT D
    RX0 = 0,          // reserved
    TX0 = 1,          // reserved
    SD_EN = 4,      // output
    DC = 8,
    DISPLAY_EN = 10,
// PORT C
    SCL = PIN_WIRE_SCL,          // reserved
    SDA = PIN_WIRE_SDA,          // reserved
    NEW_REQUEST_ = 19, // interrupt falling edge AVR_INT2
// PORT A, used to select the spi bus address (not directly used)
    W_R_ = 49, // input, PL0
    BA1 = 48, // input, PL1
    BA2 = 47, // input, PL2
    BA3 = 46, // input, PL3
    BE0 = 45, // input, PL4
    BE1 = 44, // input, PL5
    BOOT_NORMAL_ = 43, // input, PL6
    SYSTEM_FAIL_ = 42, // input, PL7
    CYCLE_READY_ = 41, // output
    Count,          // special, must be last
};
#endif // end ARDUINO_AVR_MEGA2560

#endif //CHIPSET_PINOUT_MEGA2560_H
