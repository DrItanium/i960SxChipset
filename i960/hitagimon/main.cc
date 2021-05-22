//
// Created by jwscoggins on 5/2/21.
//
#include <string>
#include <iostream>
#include "IODevice.h"
BuiltinLED theLed(0);
int wait(int count) {
    int result = 0;
    for (int i = 0; i < count; ++i) {
        result += i;
    }
    return result;
}
int main() {
    std::string donuts("donuts");
    std::cout << donuts << std::endl;
    theLed.setValue(false);
    wait(1000);
    while(true) {
        theLed.toggle();
        wait(1000000);
    }
    return 0;
}

extern "C"
int atexit(void (*function)(void))
{

}