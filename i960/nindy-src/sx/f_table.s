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
/* fault table */
    .globl  fault_table
    .align  8
fault_table:
    .word   (0 << 2) | 0x2
    .word   0x2bf               # Reserved fault        entry 0 => _user_reserved
    .word   (1 << 2) | 0x2
    .word   0x2bf               # Trace fault           entry 1 => _user_trace
    .word   (2 << 2) | 0x2
    .word   0x2bf               # Operation fault       entry 2 => _user_operation
    .word   (3 << 2) | 0x2
    .word   0x2bf               # Arithmetic fault      entry 3 => _user_arithmetic
    .word   (4 << 2) | 0x2
    .word   0x2bf               # Floating Point fault  entry 4 => _user_real_arithmetic
    .word   (5 << 2) | 0x2
    .word   0x2bf               # Constraint fault      entry 5 => _user_constraint
    .word   (0 << 2) | 0x2
    .word   0x2bf               # Reserved fault        entry 0 => _user_reserved
    .word   (6 << 2) | 0x2
    .word   0x2bf               # Protection fault      entry 6 => _user_protection
    .word   (7 << 2) | 0x2
    .word   0x2bf               # Machine fault         entry 7 => _user_machine
    .word   (0 << 2) | 0x2
    .word   0x2bf               # Reserved fault        entry 0 => _user_reserved
    .word   (8 << 2) | 0x2
    .word   0x2bf               # Type fault            entry 8 => _user_type
    .word   (0 << 2) | 0x2
    .word   0x2bf               # Reserved fault        entry 0 => _user_reserved
    .word   (0 << 2) | 0x2
    .word   0x2bf               # Reserved fault        entry 0 => _user_reserved
    .word   (0 << 2) | 0x2
    .word   0x2bf               # Reserved fault        entry 0 => _user_reserved
    .word   (0 << 2) | 0x2
    .word   0x2bf               # Reserved fault        entry 0 => _user_reserved
    .word   (0 << 2) | 0x2
    .word   0x2bf               # Reserved fault        entry 0 => _user_reserved
    .word   (0 << 2) | 0x2
    .word   0x2bf               # Reserved fault        entry 0 => _user_reserved
    .word   (0 << 2) | 0x2
    .word   0x2bf               # Reserved fault        entry 0 => _user_reserved
    .word   (0 << 2) | 0x2
    .word   0x2bf               # Reserved fault        entry 0 => _user_reserved
    .word   (0 << 2) | 0x2
    .word   0x2bf               # Reserved fault        entry 0 => _user_reserved
    .word   (0 << 2) | 0x2
    .word   0x2bf               # Reserved fault        entry 0 => _user_reserved
    .word   (0 << 2) | 0x2
    .word   0x2bf               # Reserved fault        entry 0 => _user_reserved
    .word   (0 << 2) | 0x2
    .word   0x2bf               # Reserved fault        entry 0 => _user_reserved
    .word   (0 << 2) | 0x2
    .word   0x2bf               # Reserved fault        entry 0 => _user_reserved
    .word   (0 << 2) | 0x2
    .word   0x2bf               # Reserved fault        entry 0 => _user_reserved
    .word   (0 << 2) | 0x2
    .word   0x2bf               # Reserved fault        entry 0 => _user_reserved
    .word   (0 << 2) | 0x2
    .word   0x2bf               # Reserved fault        entry 0 => _user_reserved
    .word   (0 << 2) | 0x2
    .word   0x2bf               # Reserved fault        entry 0 => _user_reserved
    .word   (0 << 2) | 0x2
    .word   0x2bf               # Reserved fault        entry 0 => _user_reserved
    .word   (0 << 2) | 0x2
    .word   0x2bf               # Reserved fault        entry 0 => _user_reserved
    .word   (0 << 2) | 0x2
    .word   0x2bf               # Reserved fault        entry 0 => _user_reserved
    .word   (0 << 2) | 0x2
    .word   0x2bf               # Reserved fault        entry 0 => _user_reserved
