//
// Created by jwscoggins on 5/3/21.
//

#ifndef I960SXCHIPSET_IODEVICE_H
#define I960SXCHIPSET_IODEVICE_H
#include <stdint.h>
#include "ChipsetInteract.h"

/**
 * @brief Manages the builtin led provided by the chipset
 */
class BuiltinLED {
public:
    bool getValue();
    void setValue(bool value);
    void toggle();
};

class BuiltinPWM {
public:
    BuiltinPWM(uint32_t offset);
    inline uint32_t getOffset() const { return offset_; }
    uint32_t getBaseAddress() const { return baseAddress_; }
    uint16_t getValue();
    void setValue(uint16_t value);
private:
    uint32_t offset_;
    uint32_t baseAddress_;
};

class BuiltinAnalogInput {
public:
    BuiltinAnalogInput(uint32_t offset);
    inline uint32_t getOffset() const { return offset_; }
    uint32_t getBaseAddress() const { return baseAddress_; }
    uint16_t getValue();
private:
    uint32_t offset_;
    uint32_t baseAddress_;

};
#endif //I960SXCHIPSET_IODEVICE_H
