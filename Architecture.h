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
#ifndef ARCHITECTURE_H
#define ARCHITECTURE_H
#if defined(__arm__) || defined(__thumb__)
#define CPU_IS_ARM
#endif
#if defined(__AVR) || defined(AVR)
#define CPU_IS_AVR
#endif

#if defined(CPU_IS_ARM) && defined(CPU_IS_AVR)
#error "Multiple CPU architectures detected!"
#endif

enum class CPUArchitecture {
    ISA_Unknown = 0,
    ISA_ARM,
    ISA_AVR,
};

constexpr auto architectureIsArm() noexcept {
#ifdef CPU_IS_ARM
    return true;
#else
    return false;
#endif
}
constexpr auto architectureIsAVR() noexcept {
#ifdef CPU_IS_AVR
    return true;
#else
    return false;
#endif
}

constexpr auto getArchitecture() noexcept {
    if constexpr (architectureIsAVR()) {
        return CPUArchitecture::ISA_AVR;
    } else if constexpr (architectureIsArm()) {
        return CPUArchitecture::ISA_ARM;
    } else {
        return CPUArchitecture::ISA_Unknown;
    }
}
#ifndef CPU_IS_ARM
#ifndef CPU_IS_AVR
#warning "Unknown cpu architecture being used."
#endif
#endif
#endif //ARCHITECTURE_H
