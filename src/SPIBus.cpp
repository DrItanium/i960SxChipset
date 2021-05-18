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
volatile byte busId = 0;
void setSPIBusId(byte id) noexcept {
#if 0
    if constexpr (onAtmega1284p()) {
        // atmega 1284p specific but so damn simple
        if (PORTA != id) {
            PORTA = id;
        }
    }
#endif
}

byte getCurrentSPIBusDevice() noexcept {
    if constexpr (onAtmega1284p()) {
        return static_cast<byte>(PORTA);
    } else {
        /// @todo this must be implemented by different targets
        return 0;
    }
}

void setupSPIBus(byte initialBusId) noexcept {
    static bool initialized = false;
    if (!initialized) {
       initialized = true;
#if 0
        if constexpr (onAtmega1284p()) {
            DDRA = 0xFF; // make sure all pins on PORTA are outputs
        }
#endif
    }
    setSPIBusId(initialBusId);
}
