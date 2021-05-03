//
// Created by jwscoggins on 5/2/21.
//

#ifndef I960SXCHIPSET_PERIPHERALS_H
#define I960SXCHIPSET_PERIPHERALS_H
#include <stdint.h>
inline uint32_t getIOBase0Address() { return 0xFE000000; }
inline uint32_t getCPUInternalMemoryBaseAddress() { return 0xFF000000; }

#endif //I960SXCHIPSET_PERIPHERALS_H
