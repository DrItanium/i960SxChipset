//
// Created by jwscoggins on 5/2/21.
//

#ifndef I960SXCHIPSET_PERIPHERALS_H
#define I960SXCHIPSET_PERIPHERALS_H
#include <stdint.h>
template<typename T>
inline T* computeIOAddress(uint32_t byteOffset = 0) { return reinterpret_cast<T*>(0xFE000000 + (byteOffset & 0xFFFFFF)); }
inline uint8_t* getLEDAddress() { return getIOBase0Address<uint8_t>(0); }
#endif //I960SXCHIPSET_PERIPHERALS_H
