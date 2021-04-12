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

/************************************************/
/* Integer power function			*/
/*                           			*/
/************************************************/
unsigned int int_pow(number,  power)
int number, power;
{
int i;
unsigned int result;


	if (power == 0)
		return (1);

	result = number;
	for (i=1; i<power; i++) {
		result *= number;
	}
	return (result);
}
/************************************************/
/* Ascii to Decimal Converter    		*/
/*                           			*/
/************************************************/
atod(s,n)
char *s;
int *n;
{
int num;

	if ( *s == '\0' ){
		return FALSE;
	}
	for ( num=0; *s; s++ ){
		if ( !strchr("0123456789",*s) ){
			return FALSE;
		}
		num = (num * 10) + (*s-'0');
	}
	*n = num;
	return TRUE;
}

/************************************************/
/* Ascii to Hex Converter    			*/
/*                           			*/
/************************************************/
atoh(s,n)
char *s;
int *n;
{
int val;
int h;

	if ( s[0] == '0' && (s[1] == 'x' || s[1] == 'X') ){
		s += 2;
	}

	if ( *s == '\0' ){
		return FALSE;
	}

	for ( val = 0; *s; s++ ){
		if ( (h=hexval(*s)) == ERROR) {
			return FALSE;
		}
		val = (val << 4) + h;
	}
	*n = val;
	return TRUE;
}

/************************************************/
/* Hex value of character passed		*/
/*                           			*/
/* This is a lazy way to get the value,but it   */
/* works!                                       */
/************************************************/
hexval(c)
char c;
{
	switch (c) {
		case '0': return(0);
		case '1': return(1);
		case '2': return(2);
		case '3': return(3);
		case '4': return(4);
		case '5': return(5);
		case '6': return(6);
		case '7': return(7);
		case '8': return(8);
		case '9': return(9);
		case 'a': 
		case 'A': return(10);
		case 'b': 
		case 'B': return(11);
		case 'c': 
		case 'C': return(12);
		case 'd': 
		case 'D': return(13);
		case 'e': 
		case 'E': return(14);
		case 'f': 
		case 'F': return(15);
		default : return(ERROR);
	}
}

/************************************************/
/* Hex to ASCII Decimal Converter      		*/
/*                           			*/
/************************************************/
htoad(number, size, string, neg)
int number;			/* number to be converted */
int size;			/* size or precision of number */
unsigned char string[];		/* return string */
int neg;			/* are negatives important? */
				/* if so, no leading 0's */
{
int first,large;
int i, val;
unsigned char *strptr; 

	large = int_pow(10, (size-1));
	first = TRUE;
	strptr = string;

	if (neg) {	/* check for negative numbers if appropriate */
		if (number < 0) {
			number = -number;
			*strptr++ = '-';
		}
	}

	for (i=0; i<size; i++) {
		val = ((number / large)) % 10;  /* get digit */
		large = large / 10;

		/* strip leading 0's if appropriate */
		if (val == 0) {	
			if (first && (i != size-1) && neg) {
				continue;
			}
		}

		else 
			first = FALSE;
		*strptr++ = val + '0';
	}
	*strptr = NUL;
}
