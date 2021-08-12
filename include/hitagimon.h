//
// Created by jwscoggins on 8/12/21.
//

#ifndef SXCHIPSET_HITAGIMON_H
#define SXCHIPSET_HITAGIMON_H
const unsigned char* getBootRom() noexcept;
const unsigned char* getBootDat() noexcept;
unsigned int getBootRomLength() noexcept;
unsigned int getBootDataLength() noexcept;
#endif //SXCHIPSET_HITAGIMON_H
