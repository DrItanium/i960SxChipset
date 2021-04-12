/*
i960SxChipset
Copyright (c) 2020-2021, Joshua Scoggins
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef NINDYPP_IOROUTINES_H
#define NINDYPP_IOROUTINES_H
#include <stdint.h>

/**
 * @brief Memory mapped I/O routine, writes 8 bit value
 * @tparam T The type of the port to write to
 * @param port The memory address to write to
 * @param value The value to write
 */
template<typename T>
void output(T* port, T value) {
    *port = value;
}
inline void output(uint8_t* port, uint8_t value) { output<uint8_t>(port, value); }
inline void output(uint16_t* port, uint16_t value) { output<uint16_t>(port, value); }
inline void output(uint32_t* port, uint32_t value) { output<uint32_t>(port, value); }
/**
 * @brief Memory mapped input routine
 * @tparam T The type of the port to read from
 * @param port The memory address to load from
 * @return 8-bit value from port
 */
template<typename T>
T input(T* port) {
    return *port;
}

/************************************************/
/* console io                                   */
/*                                              */
/* Provide system services console io.		*/
/* This routine will be entered from "calls 0"  */
/* in the supervisior table, thus allowing an   */
/* application to execute code and do I/O to    */
/* the serial device of this monitor through    */
/* run-time binding 				*/
/************************************************/
/**
 * @brief Provide system services console io. This routine will be entered from 'calls 0'
 * in the supervisor table, thus allowing an application to execute and do I/O to
 * the serial device of this monitor through run-time binding
 * @param type
 * @param chr
 */
void console_io(int type, int chr);

#endif //NINDYPP_IOROUTINES_H
