//
// Created by jwscoggins on 3/27/21.
//
#include "ChipsetInteract.h"
extern "C" void ISR0(void);
extern "C" void ISR_NMI(void);

extern "C"
void
ISR0(void) {
    volatile uint16_t& con = memory<uint16_t>(getIOBase0Address(0x100));
    const char* msg = "ISR0 Triggered\n";
    for (const char* ptr = msg; *ptr; ++ptr) {
        con = *ptr;
    }
}

extern "C"
void
ISR_NMI(void) {
    volatile uint16_t& con = memory<uint16_t>(getIOBase0Address(0x100));
    const char* msg = "NMI Triggered\n";
    for (const char* ptr = msg; *ptr; ++ptr) {
        con = *ptr;
    }
}
