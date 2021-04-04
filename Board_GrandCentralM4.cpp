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

#ifdef ARDUINO_GRAND_CENTRAL_M4
#include "Board_GrandCentralM4.h"
GrandCentralM4::GrandCentralM4() : onboardPixel_(OnboardNeoPixelCount, OnboardNeoPixelPin, NEO_GRB+NEO_KHZ800),
                                   flashTransport_(),
                                   onboardFlash_(&flashTransport_) {

}
void GrandCentralM4::begin() {
    onboardPixel_.begin();
    onboardPixel_.show();
    onboardFlash_.begin();
    onboardHasSd_ = onboardSDCard_.begin(SDCARD_SS_PIN);
    /// @todo implement support for accessing the onboard SdCard
}
void
GrandCentralM4::loopBody() {

}
void
GrandCentralM4::setupInterrupts() {
}

uint16_t
GrandCentralM4::load(uint32_t address, LoadStoreStyle style) {
    /// @todo implement
    return 0;
}

void
GrandCentralM4::store(uint32_t address, uint16_t value, LoadStoreStyle style) {
    // do nothing for now
}

bool
GrandCentralM4::sdcardInstalled(uint8_t index) const noexcept {
    switch (index) {
        case 0: return onboardHasSd_;
        case 1: return wifiHasSd_;
        case 2: return displayHasSd_;
        default: return false;
    }
}

GrandCentralM4 TheBoard;

#endif
