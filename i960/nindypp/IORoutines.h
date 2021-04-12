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

void init_sio_buf();

/************************************************/
/* co:						*/
/*						*/
/* Send character to console, checking for	*/
/* XON/XOFF (don't check for XON/XOFF if an	*/
/* xmodem binary download is in progress	*/
/************************************************/

/**
 * @brief Send character to console, checking for XON/XOFF (don't check for XON/XOFF if an xmodem binary download is in progress).
 * @param c The character to send
 */
void co(unsigned char c);

/************************************************/
/* ci:						*/
/*						*/
/* If characters are buffered, return next one.	*/
/* Otherwise read a character from the console.	*/
/************************************************/
/**
 * @brief If characters are buffered, return next one. Otherwise read a character from the console.
 * @return A character from the console or buffer depending on the situation
 */
unsigned char ci();


/***********************************************/
/* This routine allows the PRCB to be returned */
/* to the io libraries, in order to set        */
/* interrupts, modify control table, etc.      */
/***********************************************/
/**
 * @brief Get the PRCB address. Allows IO libraries to modify interrupts, control tables, etc.
 * @return The PRCB address
 */
unsigned int get_prcb();

/**
 * @brief Change the baud rate
 * @param dummy Ignored
 * @param dummy2 Ignored
 * @param baudrate The baud rate to set
 * @return True on success
 */
bool baud( int dummy, int dummy2, char* baudrate );
/************************************************/
/* Return the offset of a baudrate in the table	*/
/* baudtable[], ERROR if not found.		*/
/*                           			*/
/************************************************/
/**
 * @brief Return the offset of a baudrate in the table baudtable[], ERROR if not found
 * @param baudrate The baud rate to convert from
 * @return The baud rate
 */
int lookup_baud(int baudrate);
/************************************************/
/* prtf						*/
/*   (1) provides a simple-minded equivalent	*/
/*       of the printf function.		*/
/*   (2) translates '\n' to \n\r'.		*/
/*   (3) does nothing (suppresses output) if we	*/
/*	 are running in gdb mode (i.e.,		*/
/*	 processing a remote debugger commmand).*/
/*						*/
/* In addition to the format string, up to 4	*/
/* arguments can be passed.			*/
/*						*/
/* Only the following specifications are	*/
/* recognized in the format string:		*/
/*						*/
/*	%B	output a single hex Byte, as 2	*/
/*		  hex digits with lead 0s.	*/
/*	%b	output a single hex Byte, up to	*/
/*		   2 digits without lead 0s.	*/
/*	%c	output a single character	*/
/*	%H	output a hex Half (short) word,	*/
/*		  as 4 hex digits with lead 0s.	*/
/*	%h	output a hex Half (short) word,	*/
/*		  up to 4 digits w/o lead 0s.	*/
/*	%s	output a character string.	*/
/*	%X	output a heX word, as 8 hex	*/
/*		  digits with lead 0s.		*/
/*	%x	output a heX word: up to 8 hex	*/
/*		  digits without lead 0s.	*/
/*	%%	output the character '%'.	*/
/************************************************/
/**
 * @brief A simple-minded equivalent to the printf function.
 *        It translates \n to \n\r.
 *        It does nothing (suppresses output) if we are running in gdb mode (i.e., processing a remote debugger command)
 *        In addition to the format string, up to 4 arguments can be passed.
 *        Only the following specifications are recognized in the format string:
 *
 *        %B - output a single hex Byte, as 2 hex digits with leading 0s.
 *        %b - output a single hex Byte, up to 2 digits without leading 0s.
 *        %c - output a single character
 *        %H - Output a hex Half (short) word, as 4 hex digits with leading 0s.
 *        %h - output a hex Half (short) word, up to 4 digits w/o leading 0s.
 *        %s - output a character string.
 *        %X - output a heX word, as 8 hex digits with leading 0s
 *        %x - output a heX word: up to 8 hex digits without leading 0s
 *        %% - output the character '%'
 * @param fmt The format string to use
 * @param arg0 The first argument
 * @param arg1 The second argument
 * @param arg2 The third argument
 * @param arg3 The fourth argument
 */
void prtf( char* fmt, int arg0 = 0, int arg1 = 0, int arg2 = 0, int arg3 = 0);
/**
 * @brief A version of prtf that does suppress output while in gdb mode, and
 * so can be used to format replies to a debugger on a remote host
 * @param fmt The format string to use
 * @param arg0 The first argument
 * @param arg1 The second argument
 * @param arg2 The third argument
 * @param arg3 The fourth argument
 */
void gprtf( char* fmt, int arg0 = 0, int arg1 = 0, int arg2 = 0, int arg3 = 0);
#endif //NINDYPP_IOROUTINES_H
