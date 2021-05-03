//
// Created by jwscoggins on 5/2/21.
//
#include <unistd.h>
#include <errno.h>

extern "C"
int brk(void* addr) {
    return -1;
}