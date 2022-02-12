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
//
// Created by jwscoggins on 2/11/22.
//
#include "PSRAMChip.h"
#include "ExternalHardwareInterface.h"
namespace ExternalHardware
{
    void
    begin(DeviceIs<Devices::PSRAM>) noexcept {
        digitalWrite<OnboardPSRAMBlock::EnablePin, LOW>();
    }
    void
    end(DeviceIs<Devices::PSRAM>) noexcept {
        digitalWrite<OnboardPSRAMBlock::EnablePin, HIGH>();
    }
    void
    configure(DeviceIs<Devices::PSRAM>) noexcept {
        OnboardPSRAMBlock::begin();
    }
    namespace
    {
        constexpr byte computePortLookup(byte value) noexcept {
            return (value & 0b111) << 2;
        }
        void
        setChipId(byte index) noexcept {
            static constexpr byte theItemMask = 0b00011100;
            static constexpr byte theInvertedMask = ~theItemMask;
            static constexpr byte LookupTable[8]{
                    computePortLookup(0),
                    computePortLookup(1),
                    computePortLookup(2),
                    computePortLookup(3),
                    computePortLookup(4),
                    computePortLookup(5),
                    computePortLookup(6),
                    computePortLookup(7),
            };
            // since this is specific to type 1 we should speed this up significantly
            auto contents = PORTC;
            contents &= theInvertedMask;
            contents |= LookupTable[index & 0b111];
            PORTC = contents;
        }
    }
    void
    select(byte index, DeviceIs<Devices::PSRAM>) noexcept {
        static byte currentIndex = 0xFF;
        if (currentIndex != index) {
            setChipId(index);
        }
    }

}