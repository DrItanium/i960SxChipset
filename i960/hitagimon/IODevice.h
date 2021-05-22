//
// Created by jwscoggins on 5/3/21.
//

#ifndef I960SXCHIPSET_IODEVICE_H
#define I960SXCHIPSET_IODEVICE_H
#include <stdint.h>
#include "ChipsetInteract.h"

class BuiltinIOBaseDevice {
public:
    BuiltinIOBaseDevice(uint32_t offset);
    uint32_t getBaseAddress() const { return baseAddress_; }
    uint32_t getOffset() const { return offset_; }
protected:
    uint32_t offset_;
    uint32_t baseAddress_;
};
/**
 * @brief Manages the builtin led provided by the chipset
 */
class BuiltinLED : public BuiltinIOBaseDevice {
public:
    BuiltinLED(uint32_t offset);
    bool getValue();
    void setValue(bool value);
    void toggle();
private:
    volatile uint8_t& _memory;
};

class BuiltinPWM : public BuiltinIOBaseDevice {
public:
    BuiltinPWM(uint32_t offset);
    uint16_t getValue();
    void setValue(uint16_t value);
private:
    volatile uint16_t& _memory;
};

class BuiltinAnalogInput : public BuiltinIOBaseDevice {
public:
    BuiltinAnalogInput(uint32_t offset);
    uint16_t getValue();
};

/**
 * @brief Interface with the i2c unit found in the chipset
 */
class BuiltinI2CUnit : public BuiltinIOBaseDevice {
public:
    BuiltinI2CUnit(uint32_t offset);
    void beginTransmission(uint8_t address);
    void write(uint8_t value);
    void write(uint8_t* bytes, uint8_t length);
    uint8_t read();
    bool available();
    void endTransmission();
};
#endif //I960SXCHIPSET_IODEVICE_H
