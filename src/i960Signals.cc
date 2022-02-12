/*
i960SxChipset
Copyright (c) 2020-2022, Joshua Scoggins
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
//
// Created by jwscoggins on 2/11/22.
//

#include "ExternalHardwareInterface.h"
#include "Pinout.h"

namespace ExternalHardware {
    void
    select(DeviceIs<Devices::GPIO>) noexcept {
        // nothing to do for this target but who knows in the future
    }

    void
    begin(DeviceIs<Devices::GPIO>) noexcept {
        DigitalPin<i960Pinout::GPIOSelect>::assertPin();
    }
    void
    end(DeviceIs<Devices::GPIO>) noexcept {
        DigitalPin<i960Pinout::GPIOSelect>::deassertPin();
    }

    void
    configure(DeviceIs<Devices::GPIO>) noexcept {
        pinMode(i960Pinout::GPIOSelect, OUTPUT);
        DigitalPin<i960Pinout::GPIOSelect>::deassertPin();
    }

    void
    pulse(DeviceIs<Devices::Ready>) noexcept {
        DigitalPin<i960Pinout::Ready>::pulse();
    }

    void
    pulse(DeviceIs<Devices::Int0>) noexcept {
        if (TargetBoard::onAtmega1284p_Type1()) {
            ::pulse<i960Pinout::Int0_>();
        }
    }
    void
    begin(DeviceIs<Devices::Reset>) noexcept {
        DigitalPin<i960Pinout::Reset960>::assertPin();
    }
    void
    end(DeviceIs<Devices::Reset>) noexcept {
        DigitalPin<i960Pinout::Reset960>::deassertPin();
    }
    void
    configure(DeviceIs<Devices::Reset>) noexcept {
        pinMode(i960Pinout::Reset960, OUTPUT);
        DigitalPin<i960Pinout::Reset960>::deassertPin();
    }
}
