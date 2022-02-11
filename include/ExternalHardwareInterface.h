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

#ifndef SXCHIPSET_EXTERNALHARDWAREINTERFACE_H
#define SXCHIPSET_EXTERNALHARDWAREINTERFACE_H

/**
 * @brief Wrapper around a future theoretical decoder interface to reduce pin pressure on the 1284p. For example, instead of binding
 * ~READY directly to a GPIO, we instead bind it to the decoder array because it is only ever toggled to tell the CPU to either read the
 * data on the bus or get the next value ready. Same thing for INT0, it is always a toggle.
 */
class ExternalHardware {
public:
    /**
     * @brief The list of devices that are known to be connected to this chipset regardless of actual interface
     */
    enum class Devices {
#define X(v, i) BD ## v ## i
#define Y(v) X(v, 0), X(v, 1), X(v, 2), X(v, 3), X(v, 4), X(v, 5), X(v, 6), X(v, 7)
        Y(A), Y(B), Y(C), Y(D), Y(E), Y(F), Y(G), Y(H),
#undef Y
#undef X
        OnboardFlash0 = BDH5,
        OnboardFlash1 = BDH6,
        OnboardPSRAM0 = BDH7,
    };
public:
    ExternalHardware() = delete;
    ~ExternalHardware() = delete;
    ExternalHardware(const ExternalHardware&) = delete;
    ExternalHardware(ExternalHardware&&) = delete;
    ExternalHardware& operator=(const ExternalHardware&) = delete;
    ExternalHardware& operator=(ExternalHardware&&) = delete;

};

#endif //SXCHIPSET_EXTERNALHARDWAREINTERFACE_H
