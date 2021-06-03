//
// Created by jwscoggins on 5/2/21.
//
#include "IODevice.h"
int wait(int count) {
    int result = 0;
    for (int i = 0; i < count; ++i) {
        result += i;
    }
    return result;
}
volatile uint16_t& getConsoleReadWritePort() {
    return memory<uint16_t>(getIOBase0Address(0x102));
}
void conPrint(const char* msg, bool addNewline = false) {
    volatile uint16_t& con = getConsoleReadWritePort();
    for (const char* ptr = msg; *ptr; ++ptr) {
        con = *ptr;
    }
    if (addNewline) {
        con = '\n';
    }
}
void conPrint(char value) {
    volatile uint16_t& con = getConsoleReadWritePort();
    con = value;
}
void conPrintln(const char* ptr) {
    conPrint(ptr, true);
}
int main() {
    BuiltinLED theLed(0);
    volatile uint8_t pwmIndex = 127;
    conPrint("hello, world\n");
    conPrintln("donuts");
    //printf("%s!\n", "Printf test");
    //std::cout << "std::cout test" << std::endl;
    //std::cout << "0x" << std::hex << 0xFDED << std::endl;
    while(true) {
        theLed.toggle();
        pwmIndex += wait(1000000);
        theLed.toggle();
        pwmIndex += wait(1000000);
    }
    return 0;
}

extern "C"
int atexit(void (*function)(void))
{
     function();
}
#if 0
// functions for back end testing
extern "C"
ssize_t write(int fildes, const void* buf, size_t nbyte) {
    const char* theBuf = (const char*)buf;
    volatile uint16_t& con = getConsoleReadWritePort();
    for (size_t i = 0; i < nbyte; ++i) {
        con = (uint16_t)theBuf[i];
    }
    return nbyte;
}

extern "C"
ssize_t read(int fildes, void* buf, size_t nbyte) {
    char* theBuf = (char*)buf;
    volatile uint16_t& con = getConsoleReadWritePort();
    for (int i = 0; i < nbyte; ++i) {
        theBuf[i] = (char)con;
    }
    return nbyte;
}
extern "C"
int open(const char* path, int oflag, ...) {
    // fake it for now
    return 0;
}

extern "C"
void abort() {
    while (true) {
        // do thing
    }
    // do nothing
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
#endif
