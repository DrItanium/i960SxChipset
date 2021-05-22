//
// Created by jwscoggins on 5/2/21.
//
#include <string>
#include <iostream>
#include "IODevice.h"
int wait(int count) {
    int result = 0;
    for (int i = 0; i < count; ++i) {
        result += i;
    }
    return result;
}
int main() {
    BuiltinLED theLed(0);
    BuiltinPWM thePWM(0x10);
    while(true) {
        theLed.toggle();
        for (int i = 0; i < 1024; ++i) {
            thePWM.setValue(i);
        }
        theLed.toggle();
        for (int i = 1023; i >= 0; ++i) {
            thePWM.setValue(i);
        }
    }
    return 0;
}

extern "C"
int atexit(void (*function)(void))
{

}