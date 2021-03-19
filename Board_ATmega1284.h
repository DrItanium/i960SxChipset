//
// Created by jwscoggins on 3/19/21.
//

#ifndef ARDUINO_BOARD_ATMEGA1284_H
#define ARDUINO_BOARD_ATMEGA1284_H
#include "BoardSpecific.h"

enum class i960Pinout : decltype(A0) {
    // PORT B
	Led = 0, 	  // output
   	CLOCK_OUT, // output, unusable
	AS_,     // input, AVR Int2
	PWM4, 	 // unused
	GPIOSelect,		// output
	MOSI,		  // reserved
	MISO,		  // reserved
	SCK, 		  // reserved
// PORT D
	RX0, 		  // reserved
	TX0, 		  // reserved
	DEN_,	  // AVR Interrupt INT0
	AVR_INT1, 		// AVR Interrupt INT1
	PWM0,	  // input
	PWM1, 		  // unused
	PWM2, 		  // unused
	PWM3, 		  // unused
// PORT C
	SCL,		  // reserved
	SDA, 		  // reserved
	Ready, 	  // output
	Int0_,		  // output
	W_R_, 		  // input
	Reset960,		  // output
	BLAST_, 	 // input
	FAIL, 	     // input
// PORT A
    SPI_BUS_EN, // output
    Analog1,
    Analog2,
    Analog3,
    Analog4,
	Analog5,
	Analog6,
	Analog7,
	Count,		  // special
};
static_assert(static_cast<decltype(HIGH)>(i960Pinout::Count) <= 32);
#endif //ARDUINO_BOARD_ATMEGA1284_H
