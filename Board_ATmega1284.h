/*
i960SxChipset
Copyright (c) 2020-2021, Joshua Scoggins
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef ARDUINO_BOARD_ATMEGA1284_H
#define ARDUINO_BOARD_ATMEGA1284_H
#include <Arduino.h>
#include "BoardSpecific.h"
#include <Timer.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <SD.h>

enum class i960Pinout : decltype(A0) {
    // PORT B
	Led = 0, 	  // output
   	CLOCK_OUT, // output, unusable
	AS_,     // input, AVR Int2
	TFT_RST, 	 // output
	GPIOSelect,		// output
	MOSI,		  // reserved
	MISO,		  // reserved
	SCK, 		  // reserved
// PORT D
	RX0, 		  // reserved
	TX0, 		  // reserved
	DEN_,	  // AVR Interrupt INT0
	AVR_INT1, 		// AVR Interrupt INT1
	TFT_DC,	       //
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
class MightyCore_1284p : public Board {
public:
    MightyCore_1284p();
    ~MightyCore_1284p() override = default;
    void begin() noexcept override;
    void loopBody() noexcept override;
    void setupInterrupts() noexcept override;
    uint16_t load(uint32_t address, LoadStoreStyle style) noexcept override;
    void store(uint32_t address, uint16_t value, LoadStoreStyle style) noexcept override;
    bool sdcardInstalled(uint8_t index = 0) const noexcept override;
private:
    Timer t_;
    Adafruit_ILI9341 tft_;
};
extern MightyCore_1284p TheBoard;
#endif //ARDUINO_BOARD_ATMEGA1284_H
