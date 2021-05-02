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
/******************************************************************/
/* 		Copyright (c) 1989, Intel Corporation

   Intel hereby grants you permission to copy, modify, and 
   distribute this software and its documentation.  Intel grants
   this permission provided that the above copyright notice 
   appears in all copies and that both the copyright notice and
   this permission notice appear in supporting documentation.  In
   addition, Intel grants this permission provided that you
   prominently mark as not part of the original any modifications
   made to this software or documentation, and that the name of 
   Intel Corporation not be used in advertising or publicity 
   pertaining to distribution of the software or the documentation 
   without specific, written prior permission.  

   Intel Corporation does not warrant, guarantee or make any 
   representations regarding the use of, or the results of the use
   of, the software and documentation in terms of correctness, 
   accuracy, reliability, currentness, or otherwise; and you rely
   on the software, documentation and results solely at your own 
   risk.							  */
/******************************************************************/
#include "defines.h"
#include "globals.h"
#include "block_io.h"

extern unsigned prcb_ram;

unsigned char ci();

#define LINESIZE	80

static unsigned char input_buffer[LINESIZE];
static int in = 0;	/* Offset in buffer at which next input char goes */
static int out = 0;	/* Offset in buffer from which next output char taken */

/* Table of legal baudrates, terminated with illegal rate "0" */
int baudtable[] = {
        1200,
        2400,
        4800,
        9600,
        19200,
        38400,
        57600,
        74880,
        115200,
        230400,
        250000,
        500000,
        1000000,
        2000000,
        0 };



/************************************************/
/* read byte (timeout)                          */
/*                                              */
/* This routine waits until a character is      */
/* available at the console input.  If the char */
/* is available before the timeout on the serial*/
/* channel, it returns the character. Otherwise,*/
/* return TIMEOUT error.  			*/
/************************************************/

extern int readbyte_ticks_per_second;	/* Board-specific constant */

int 
readbyte(int seconds) {
    for (int ticks = seconds*readbyte_ticks_per_second; ticks > 0; ticks--){
        if (csts() != 0){
            return static_cast<int>(ci());
        }
    }

    /* if timeout error, return TIMEOUT */
    return TIMEOUT;
}

void
console_io(int type, int chr)
{
	switch (type) {
	case CI:
	    *reinterpret_cast<unsigned char*>(chr) = ci();
		break;
	case CO:
		co(chr);
		break;
	default:
		return (ERROR);
	}

	return 0;
}


/************************************************/
/* Output Hex Number         			*/
/*                           			*/
/* output a 32 bit value in hex to the console  */
/* leading determines whether or not to print   */
/* leading zeros				*/
/************************************************/
namespace {
    /**
     * @brief Output a given value in hexadecimal (32-bit value). Output a 32-bit value in hex to the console
     * leading determines whether or not to print leading zeros.
     * @param value The value to emit
     * @param bits Number of low-order bits in 'value' to be output
     * @param leading Leading 0 flag?
     */
    void
    out_hex(unsigned int value, int bits, int leading) {
        static char tohex[] = "0123456789abcdef";
        unsigned char out;

        for (bits -= 4; bits >= 0; bits -= 4) {
            out = ((value >> bits) & 0xf); /* capture digit */
            if ( out == 0 ){
                if ( leading || (bits == 0) ){
                    co('0');
                }
            } else {
                co( tohex[out] );
                leading = TRUE;
            }
        }
    }

/**
 * @brief local routine to add character to input buffer
 * @param c The character to add
 */
    void
    buffer_char(char c) {
        input_buffer[in] = c;
        if ( ++in >= LINESIZE ){
            in = 0;
        }
    }

    void
    badfmt( char* f )
    {
        prtf( "\nInternal error: bad prtf format: %s\n", f );
    }
}
void
init_sio_buf()
{
    in = 0;
    out = 0;
}

void
co(unsigned char c)
{
	if ( !downld ){
		/* we need to check for flow control before transmitting */
		while ( csts() ){
			/* there is a character at the input queue, read it */
			unsigned char tmp = board_ci();
			if ( tmp == XOFF ){
				while ( (tmp=board_ci()) != XON ){
					buffer_char(tmp);
				}
			} else {
				buffer_char(tmp);
			}
		}
	}

	/* transmit character */
	board_co(c);
}


/************************************************/
/* ci:						*/
/*						*/
/* If characters are buffered, return next one.	*/
/* Otherwise read a character from the console.	*/
/************************************************/
unsigned char
ci()
{
	if ( out != in ){
		/* Input buffer not empty: get character from there. */
		unsigned char c = input_buffer[out];
		if ( ++out >= LINESIZE ){
			out = 0;
		}
        return c;
	} else {
        // get character from keyboard
	    return board_ci();
	}
}
#if 0
/************************************************/
/* laser io                                     */
/*                                              */
/* Provide laser printer direct lpt I/O.	*/
/* This routine will be entered from "calls 2"  */
/* in the supervisior table, thus allowing an   */
/* application to execute code and do I/O to    */
/* the serial device of this monitor through    */
/* run-time binding 				*/
/************************************************/
lpt_io(type, chr)
int type, chr;
{

	switch (type) {
	case CI:
		*(unsigned char *)chr = ci();
		break;
	case CO:
		co(chr);
		break;
	default:
		return (ERROR);
	}

	return 0;
}
#endif

unsigned int
get_prcb()
{
	return((int)&prcb_ram);
}

bool
baud( int dummy, int dummy2, char* baudrate )
{
int i;
int baud;

	if ( atod(baudrate,&baud) && (i=lookup_baud(baud)) != ERROR ){
		change_baud(baud);
		end.r_baud = i;
		return true;
	} else {
		prtf( "Bad baud rate specification" );
		return false;
	}
}


int
lookup_baud(int baudrate)
{
    for (int i=0; baudtable[i]; i++ ){
        if ( baudtable[i] == baudrate ){
            return i;
        }
    }
    return ERROR;
}


#define MAX_PRTF_ARGS 4
void
prtf(char* fmt, int arg0, int arg1, int arg2, int arg3 )
{
	if ( gdb ){
		return;		/* suppress output */
	}

	int args[MAX_PRTF_ARGS] {
	    arg0, arg1, arg2, arg3,
	};
	int argc = 0;
	for (char* p = fmt; *p; p++ ){
		if ( *p != '%' ){
			co( *p );
			if ( *p == '\n' ){
				co('\r');
			}
			continue;
		}

		/* Everything past this point is processing a '%'
		 * format specification.
		 */

		p++;	/* p -> character after the '%'	*/

		if ( *p == '\0' ){
			badfmt( fmt );
			return;
		}
		
		if ( *p == '%' ){
			co( *p );
			continue;
		}

		if ( argc >= MAX_PRTF_ARGS ){
			badfmt( fmt );
			return;
		}
		switch (*p){
		case 'B':
		case 'b':
			out_hex (args[argc], BYTE, (*p=='B') ? TRUE : FALSE);
			break;
		case 'c':
			co( args[argc] );
			if ( args[argc] == '\n' ){
				co('\r');
			}
			break;
		case 'H':
		case 'h':
			out_hex (args[argc], SHORT, (*p=='H') ? TRUE : FALSE);
			break;
		case 's':
			for (char* q = (char*)args[argc]; *q; q++ ){
				co(*q);
				if ( *q == '\n' ){
					co('\r');
				}
			}
			break;
		case 'X':
		case 'x':
			out_hex (args[argc], INT, (*p=='X') ? TRUE : FALSE);
			break;
		default:
			badfmt( fmt );
			return;
		}
		argc++;
	}
}


void
gprtf( char* fmt, int arg0, int arg1, int arg2, int arg3 )
{
	char old_gdb = gdb;
	gdb = FALSE;
	prtf(fmt,arg0,arg1,arg2,arg3);
	gdb = old_gdb;
}
