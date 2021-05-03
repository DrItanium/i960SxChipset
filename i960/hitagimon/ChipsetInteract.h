//
// Created by jwscoggins on 5/2/21.
//

#ifndef I960SXCHIPSET_PERIPHERALS_H
#define I960SXCHIPSET_PERIPHERALS_H
#include <stdint.h>
template<typename T>
inline volatile T& memory(const uint32_t address) { return *reinterpret_cast<T*>(address); }
inline uint32_t getIOBase0Address(const uint32_t offset) { return 0xFE000000 + (offset & 0x00FFFFFF); }
inline volatile uint8_t& iobase0Memory(const uint32_t offset = 0) { return memory<uint8_t>(getIOBase0Address(offset)); }
#endif //I960SXCHIPSET_PERIPHERALS_H
