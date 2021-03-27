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


// NOTE: this code is taken from the initialization code found in the i960Sx manual

/*
Below is the system initialization code and tables.
The code builds the PRCB (PRocessor Control Block) in memory, sets up the stack frame, the interrupt,
fault, and system procedure tables, and then vectors to a user defined routine. *main*
*/

// declare ahead of time

.globl system_address_table
.globl prcb_ptr
.globl _prcb_ram
.globl start_ip
.globl cs1

.globl _user_stack
.globl _sup_stack // supervisor stack
.globl _intr_stack // interrupt stack


// Core Initialization Block (located at address 0)
// 8 words

.text

system_address_table:
    .word system_address_table // SAT pointer
    .word prcb_ptr // prcb pointer
    .word 0
    .word start_ip // pointer to first ip
    .word cs1 // calculated at link time (bind ?cs1 (- (+ ?SAT ?PRCB ?startIP)))
    .word 0
    .word 0
    .word -1

    // now reserve 88 more bytes
    .space 88

    .word sys_proc_table        // initialization words
    .word 0x304000fb
    .space 8

    .word system_address_table
    .word 0x00fc00fb            // initialization words
    .space 8

    .word sys_proc_table
    .word 0x304000fb            // initialization words
    .space 8

    .word fault_proc_table
    .word 0x304000fb            // initialization words

// initial PRCB
// this is our startup PRCB. After initialization.
// this will be copied to RAM

prcb_ptr:
    .word 0x0 // 0 - reserved
    .word 0xc // 4 - initialize to 0xc
    .word 0x0 // 8 - reserved
    .word 0x0 // 12 - reserved
    .word 0x0 // 16 - reserved
    .word intr_table // 20 - interrupt table address
    .word _intr_stack // 24 - interrupt stack pointer
    .word 0x0 // 28 - reserved
    .word 0x000001ff  // 32 - pointer to offset zero
    .word 0x0000027f  // 36 - system procedure table pointer
    .word fault_table // 40 - fault table
    .space 0x0 // 44 - reserved
    .word 32 // 48 - reserved
    .word 92 // 80 - scratch space

// the system procedure table will _only_ be used if the user make a supervisor procedure call
.align 6
sys_proc_table:
.word 0 # Reserved
.word 0 # Reserved
.word 0 # Reserved
.word (_sup_stack + 0x1) # Supervisor stack pointer
.word 0 # Preserved
.word 0 # Preserved
.word 0 # Preserved
.word 0 # Preserved
.word 0 # Preserved
.word 0 # Preserved
.word 0 # Preserved
.word 0 # Preserved
.word 0 # Calls 0
.word 0 # Calls 1
.word 0 # Calls 2
.word 0 # Calls 3
