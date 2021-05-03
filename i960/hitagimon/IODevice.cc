//
// Created by jwscoggins on 5/3/21.
//

#include "IODevice.h"
#include "ChipsetInteract.h"

const uint32_t ioOffset = 0;
const uint32_t address = getIOBase0Address(ioOffset);
void
BuiltinLED::toggle() {
    volatile uint8_t& mem = memory<uint8_t>(address);
    mem = (mem != 0) ? 0 : 0xFF;
}

bool
BuiltinLED::getValue() {
    return memory<uint8_t>(address) != 0;
}

void
BuiltinLED::setValue(bool value) {
    memory<uint8_t>(address) = (value ? 0xFF : 0x00);
}
