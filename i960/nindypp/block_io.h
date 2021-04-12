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
/* 		Copyright (c) 1990, Intel Corporation

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

/*****************************************************************************
 * Structures and definitions supporting NINDY requests for services by a
 * remote host.  Used by NINDY monitor, library libnin, comm960, gdb960,
 * etc.  Also contains some defines for NINDY console I/O requests.
 *****************************************************************************/
#ifndef NINDY_BLOCK_IO_H__
#define NINDY_BLOCK_IO_H__
/* the following four are hardware dependent */
#define	BIT_16		short
#define	BIT_32		int
#define	UBIT_16		unsigned short
#define	UBIT_32		unsigned int

/* Service request numbers -- these are the services that can be asked of the
 * host.
 */
#define BS_ACCESS	0x10
#define BS_CLOSE	0x20
#define BS_CREAT	0x30
#define BS_SEEK		0x40
#define BS_OPEN		0x50
#define BS_READ		0x60
#define BS_STAT		0x70
#define BS_SYSTEMD	0x80
#define BS_TIME		0x90
#define BS_UNMASK	0xa0
#define BS_UNLINK	0xb0
#define BS_WRITE	0xc0


/* Maximum number of arguments to any of the above service requests
 * (in addition to the request number).
 */
#define MAX_SRQ_ARGS    3

/* Number of bytes of data that can be read or written by a single I/O request
 */
#define BUFSIZE		1024

/* NINDY console I/O requests:  CO sends a single character to stdout,
 * CI reads one.
 */
#define CI 		0
#define CO 		1
#endif // end NINDY_BLOCK_IO_H__
