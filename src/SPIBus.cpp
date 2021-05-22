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

#include "SPIBus.h"
#include "IOExpanders.h"
#include "MCUPlatform.h"
void
SPIBus::setup() noexcept {
    if (!_initialized) {
        _initialized = true;
#ifdef ARDUINO_AVR_ATmega1284
        DDRA = 0xFF; // make sure all pins on PORTA are outputs
#elif defined(ARDUINO_NRF52_ADAFRUIT)
        extraMemoryCommit.setPortBDirection(0xFF); // make these output
#elif defined(ARDUINO_GRAND_CENTRAL_M4)
        // this will be completely different and could even be ignored because
        // the grand central has so many pins
#else
#endif
        setId(_id);
    }
}
void
SPIBus::setId(byte id) noexcept {
    setup();
#ifdef ARDUINO_AVR_ATmega1284
    // atmega 1284p specific but so damn simple
    if (PORTA != id) {
        PORTA = id;
    }
#elif defined(ARDUINO_GRAND_CENTRAL_M4)
    /// @todo implement
#else
    // use the IO Expander by Default
    if (_id != id) {
        extraMemoryCommit.writePortB(id);
        _id = id;
    }
#endif
}
byte
SPIBus::getId() const noexcept {
#ifdef ARDUINO_AVR_ATmega1284
    return static_cast<byte>(PORTA);
#elif defined(ARDUINO_GRAND_CENTRAL_M4)
    /// @todo implement
    return _id;
#else
    // by default just read from the extra memory commit io expander
    return extraMemoryCommit.readPortB();
#endif
}