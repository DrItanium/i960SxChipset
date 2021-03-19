//
// Created by jwscoggins on 3/19/21.
//

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
