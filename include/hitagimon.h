//
// Created by jwscoggins on 8/12/21.
//

#ifndef SXCHIPSET_HITAGIMON_H
#define SXCHIPSET_HITAGIMON_H
#include <Arduino.h>
#include "Pinout.h"
byte const * const getBootRom() noexcept;
byte const * const getBootData() noexcept;
Address getBootRomLength() noexcept;
Address getBootDataLength() noexcept;
#endif //SXCHIPSET_HITAGIMON_H
