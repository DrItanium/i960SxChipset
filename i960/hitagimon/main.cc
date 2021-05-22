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
    uint8_t pwmIndex = 0;
    std::cout << "donuts" << std::endl;
    while(true) {
        thePWM.setValue(pwmIndex);
        theLed.toggle();
        wait(1000);
        theLed.toggle();
        wait(1000);
        ++pwmIndex;
    }
    return 0;
}

extern "C"
int atexit(void (*function)(void))
{

}

extern "C"
void abort() {
    std::cout << "Aborting Execution!" << std::endl;
    while (true) {
        // do thing
    }
    // do nothing
}

extern "C"
ssize_t write(int fildes, const void* buf, size_t nbyte) {
    const char* theBuf = (const char*)buf;
    volatile uint16_t& con = memory<uint16_t>(0xFE000100);
    for (size_t i = 0; i < nbyte; ++i) {
        con = theBuf[i];
    }
    return nbyte;
}

extern "C"
ssize_t read(int fildes, void* buf, size_t nbyte) {
    char* theBuf = (char*)buf;
    volatile uint16_t& con = memory<uint16_t>(0xFE000100);
    for (int i = 0; i < nbyte; ++i) {
        theBuf[i] = con;
    }
    return nbyte;
}

extern "C"
off_t lseek(int fildes, off_t offset, int whence) {
    return 0;
}

extern "C"
int fstat(int fildes, struct stat* buf) {
   return 0;
}

extern "C"
int open(const char* path, int oflag, ...) {
    // fake it for now
    return 0;
}

extern "C"
void* sbrk(intptr_t increment) {
    return 0;
}

extern "C"
int close(int fildes) {
    return 0;
}

extern "C"
int isatty(int fd) {
    return 0;
}
