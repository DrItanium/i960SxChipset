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
/* Reset Board                         		*/
/*                           			*/
/************************************************/
reset()
{
	reinit_sysctl();  /* goes back to sys_init reinit */
}

/****************************************/
/* Disable interrupts on board		*/
/* 		           	        */
/*  this routine makes sure all of the  */
/* chips on the board are initialized to*/
/* a known state.			*/
/****************************************/
disable_ints()
{
}

/********************************************************/
/* FLASH support (none on EVCA)			  	*/
/********************************************************/
init_flash()
{
	fltype = flsize = 0;
}

erase_flash()
{
	check_flash();
}

check_flash ()
{
	prtf ("\n Flash not implemented on this board");
}

download_flash()
{
	return ERROR;
}

check_download(nsecs)
int nsecs;
{
int i;
	for (i=0; i<nsecs; i++) {
		downtype[i] = RAM;
	}
	return 0;
}
