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

#include "MCUPlatform.h"
namespace {
    template<typename T>
    constexpr bool compileTimeSanityCheck() noexcept {
#define ERR_STATE(msg) static_assert(false_v< T > , "Sanity check failed: " msg ); return false

    if constexpr (TargetBoard::onAtmega1284p()) {
        if constexpr(!TargetBoard::usesDisplayShield()) {
            ERR_STATE("Expected 1284p to use the tft shield");
        }
        if constexpr(TargetBoard::hasBuiltinSDCard()) {
            ERR_STATE("The 1284p does not have a builtin SD Card slot");
        }
        if constexpr (TargetBoard::getCPUFrequency() != 20_MHz) {
            ERR_STATE("Expecting the 1248p to run at 20MHz");
        }
    }
    else if constexpr (TargetBoard::onGrandCentralM4()) {
        if constexpr(!TargetBoard::hasBuiltinSDCard())
        {
            ERR_STATE("Grand Central M4 has an onboard SD CARD slot");
        }
        if constexpr(!TargetBoard::usesDisplayShield())
        {
            ERR_STATE("Expected Grand Central M4 to use the tft shield");
        }
    }
    else if constexpr (TargetBoard::onMetroM4()) {
        if constexpr(!TargetBoard::usesDisplayShield())
        {
            ERR_STATE("Expected the Metro M4 Express to use the tft shield");
        }
        if constexpr(TargetBoard::hasBuiltinSDCard())
        {
            ERR_STATE("The Metro M4 Express does not have a builtin SD Card slot");
        }
    }
    else if constexpr (TargetBoard::onFeatherM0Adalogger()) {
        if constexpr(!TargetBoard::hasBuiltinSDCard())
        {
            ERR_STATE("Feather M0 Adalogger has an onboard SD CARD slot");
        }
        if constexpr(TargetBoard::usesDisplayShield())
        {
            ERR_STATE("Feather M0 Adalogger cannot use the display shield");
        }
    }
#undef ERR_STATE
    return true;
}
}
static_assert(compileTimeSanityCheck<TargetMCU>(), "Sanity Check FAILED");

