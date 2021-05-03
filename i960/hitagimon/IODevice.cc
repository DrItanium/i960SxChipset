//
// Created by jwscoggins on 5/3/21.
//

#include "IODevice.h"
#include "ChipsetInteract.h"


BuiltinIOBaseDevice::BuiltinIOBaseDevice(uint32_t offset) : offset_(offset), baseAddress_(getIOBase0Address(offset)) { }
BuiltinLED::BuiltinLED(uint32_t offset) : BuiltinIOBaseDevice(offset) { }
void
BuiltinLED::toggle() {
    volatile uint8_t& mem = memory<uint8_t>(baseAddress_);
    mem = (mem != 0) ? 0 : 0xFF;
}

bool
BuiltinLED::getValue() {
    return memory<uint8_t>(baseAddress_) != 0;
}

void
BuiltinLED::setValue(bool value) {
    memory<uint8_t>(baseAddress_) = (value ? 0xFF : 0x00);
}

BuiltinPWM::BuiltinPWM(uint32_t offset) : BuiltinIOBaseDevice(offset) { }
void
BuiltinPWM::setValue(uint16_t value) {
    volatile uint16_t& mem = memory<uint16_t>(baseAddress_);
    mem = value;
}
uint16_t
BuiltinPWM::getValue() {
    return memory<uint16_t>(baseAddress_);
}

BuiltinAnalogInput::BuiltinAnalogInput(uint32_t offset) : BuiltinIOBaseDevice(offset) { }

uint16_t
BuiltinAnalogInput::getValue() {
    return memory<uint16_t>(baseAddress_);
}