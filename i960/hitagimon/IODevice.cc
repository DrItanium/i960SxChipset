//
// Created by jwscoggins on 5/3/21.
//

#include "IODevice.h"
#include "ChipsetInteract.h"


BuiltinIOBaseDevice::BuiltinIOBaseDevice(uint32_t offset) : offset_(offset), baseAddress_(getIOBase0Address(offset)) { }
BuiltinLED::BuiltinLED(uint32_t offset) : BuiltinIOBaseDevice(offset), _memory(memory<uint8_t>(baseAddress_)) {

}
void
BuiltinLED::toggle() {
    _memory = (_memory != 0) ? 0 : 0xFF;
}

bool
BuiltinLED::getValue() {
    return _memory != 0;
}

void
BuiltinLED::setValue(bool value) {
    _memory = (value ? 0xFF : 0x00);
}

BuiltinPWM::BuiltinPWM(uint32_t offset) : BuiltinIOBaseDevice(offset), _memory(memory<uint16_t>(baseAddress_)) { }
void
BuiltinPWM::setValue(uint16_t value) {
    _memory = value;
}
uint16_t
BuiltinPWM::getValue() {
    return _memory;
}

BuiltinAnalogInput::BuiltinAnalogInput(uint32_t offset) : BuiltinIOBaseDevice(offset) { }

uint16_t
BuiltinAnalogInput::getValue() {
    return memory<uint16_t>(baseAddress_);
}