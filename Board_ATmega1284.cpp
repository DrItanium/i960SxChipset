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

#ifdef ARDUINO_AVR_ATmega1284
#include "Board_ATmega1284.h"
MightyCore_1284p::MightyCore_1284p() : tft_(static_cast<int>(i960Pinout::SPI_BUS_EN),
                                            static_cast<int>(i960Pinout::TFT_DC),
                                            static_cast<int>(i960Pinout::MOSI),
                                            static_cast<int>(i960Pinout::SCK),
                                            static_cast<int>(i960Pinout::TFT_RST),
                                            static_cast<int>(i960Pinout::MISO)) {

}
void
MightyCore_1284p::begin() noexcept {
    pinMode(static_cast<int>(i960Pinout::Led), OUTPUT);
    t_.oscillate(static_cast<int>(i960Pinout::Led), 1000, HIGH);
    tft_.begin();
}
void
MightyCore_1284p::loopBody() noexcept {
    t_.update();
}

void
MightyCore_1284p::setupInterrupts() noexcept {
}

uint16_t
MightyCore_1284p::load(uint32_t address, LoadStoreStyle style) noexcept {
    return 0;
}
void
MightyCore_1284p::store(uint32_t address, uint16_t value, LoadStoreStyle style) noexcept {

}
bool
MightyCore_1284p::sdcardInstalled(uint8_t) const noexcept {
    return false;
}

void
MightyCore_1284p::runAtBottomOfSetup() noexcept {

}

MightyCore_1284p TheBoard;

#endif