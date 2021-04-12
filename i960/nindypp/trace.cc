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
#include "regs.h"

/* Mask for appropriate bit in register tc for each trace type
 */
#define	BRANCH 	0x04		/* branch trace		*/
#define	CALL 	0x08		/* call trace		*/
#define	RET 	0x10		/* return trace		*/
#define	SUP 	0x40		/* supervisor trace	*/


/************************************************/
/* Trace Flags Set or Cleared        	 	*/
/*                           			*/
/************************************************/
trace( dummy, nargs, type, state )
int dummy;	/* Ignored */
int nargs;	/* Number of the following arguments that are valid (0,1,2) */
char *type;	/* Optional trace type: "br", "ca", "re", or "su"	*/
char *state;	/* Desired trace state ("on" or "off")	*/
{
unsigned bitmask;

	if ( nargs > 0 ){

		/* First argument (trace type) */

		if ( !strncmp(type,"br",2) ){
			bitmask = BRANCH;
		} else if ( !strncmp(type,"ca",2) ){
			bitmask = CALL;
		} else if ( !strncmp(type,"re",2) ){
			bitmask = RET;
		} else if ( !strncmp(type,"su",2) ){
			bitmask = SUP;
		} else {
			badarg( type );
			return;
		}

		/* Second argument ("on"/"off") */

		if ( nargs < 2 ) {
			/* no 2nd argument, display trace status */
			display_trace(bitmask);
			return;
		} else if ( !strncmp(state,"of",2) ){
			register_set[REG_TC] &= ~bitmask;
		} else if ( !strncmp(state,"on",2) ){
			register_set[REG_TC] |= bitmask;
		} else {
			badarg( state );
			return;
		}
	}

	display_trace(BRANCH);
	display_trace(CALL);
	display_trace(RET);
	display_trace(SUP);
}

/************************************************/
/* Set Single Step Trace            	 	*/
/*                           			*/
/************************************************/
set_trace_step()
{
	/* set NIF flag for single step - CA A-step workaround */
	register_set[REG_AC] |= 0x8000;

	/* set single step in TC register */
	register_set[REG_TC] |= 0x2;
}

/************************************************/
/* Display Trace Status                    	*/
/*                           			*/
/************************************************/
static
display_trace( bitmask)
unsigned bitmask;	/* BRANCH, CALL, RET, or SUP */
{
	char *p;

	switch ( bitmask ){
	case BRANCH: p = "Branch";     break;
	case CALL:   p = "Call";       break;
	case RET:    p = "Return";     break;
	case SUP:    p = "Supervisor"; break;
	}

	prtf("\n %s trace %s",p,register_set[REG_TC] & bitmask ? "on" : "off" );
}
