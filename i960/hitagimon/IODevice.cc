//
// Created by jwscoggins on 5/3/21.
//

#include "IODevice.h"
#include "ChipsetInteract.h"

const uint32_t ledAddress= getIOBase0Address(0);
void
BuiltinLED::toggle() {
    volatile uint8_t& mem = memory<uint8_t>(ledAddress);
    mem = (mem != 0) ? 0 : 0xFF;
}

bool
BuiltinLED::getValue() {
    return memory<uint8_t>(ledAddress) != 0;
}

void
BuiltinLED::setValue(bool value) {
    memory<uint8_t>(ledAddress) = (value ? 0xFF : 0x00);
}

const uint32_t pwmAddress = getIOBase0Address(0x100);

BuiltinPWM::BuiltinPWM(uint32_t offset) : offset_(offset), baseAddress_(getIOBase0Address(offset)) { }
void
BuiltinPWM::setValue(uint16_t value) {
    volatile uint16_t& mem = memory<uint16_t>(baseAddress_);
    mem = value;
}
uint16_t
BuiltinPWM::getValue() {
    return memory<uint16_t>(baseAddress_);
}

BuiltinAnalogInput::BuiltinAnalogInput(uint32_t offset) : offset_(offset), baseAddress_(getIOBase0Address(offset)) { }

uint16_t
BuiltinAnalogInput::getValue() {
    return memory<uint16_t>(baseAddress_);
}