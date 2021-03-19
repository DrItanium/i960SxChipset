//
// Created by jwscoggins on 3/19/21.
//

#ifndef ARCHITECTURE_H
#define ARCHITECTURE_H
enum class CPUArchitecture {
    Unknown = 0,
    ARM,
    AVR,
};

constexpr auto architectureIsArm() noexcept {
#if defined(__arm__) || defined(__thumb__)
    return true;
#else
    return false;
#endif
}
constexpr auto architectureIsAVR() noexcept {
#if defined(__AVR) || defined(AVR)
    return true;
#else
    return false;
#endif
}

constexpr auto getArchitecture() noexcept {
    if constexpr (architectureIsAVR()) {
        return CPUArchitecture::AVR;
    } else if constexpr (architectureIsArm()) {
        return CPUArchitecture::ARM;
    } else {
        return CPUArchitecture::Unknown;
    }
}
#endif //ARCHITECTURE_H
