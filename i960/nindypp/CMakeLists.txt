cmake_minimum_required(VERSION 3.10)
project(nindypp VERSION 0.1)
enable_language(CXX C ASM-ATT)
set(CMAKE_CXX_STANDARD 98)

set(tools /home/jwscoggins/sys/i960-elf-gcc-3.4.6)
set(CMAKE_C_COMPILER ${tools}/bin/i960-elf-gcc)
set(CMAKE_CXX_COMPILER ${tools}/bin/i960-elf-g++)
set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} -msb")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msb -mnumerics")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -msb -mnumerics")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

add_library(bootsx
	sx/build
	sx/f_handle.c
	sx/f_table.s
	sx/i_handle.s
	sx/ISR.c
	sx/i_table.s
	sx/sx_init.s)
add_library(nin 
	libnin/branch.cc
	libnin/br_asm.s
	libnin/brk.cc
	libnin/copyrght.cc
	libnin/dto_stub.cc
	libnin/errno.cc
	libnin/fileio.cc
	libnin/gcc_stub.cc
	libnin/heapsize.s
	libnin/libasm.s
	libnin/makefile.ex
	libnin/newlib.cc
	libnin/pr_buck.s
	libnin/pr_end.s
	libnin/pr_freq.s
	libnin/profile.cc
	libnin/pr_start.s
	libnin/ver960.cc)
add_library(core break.cc
	convert.cc
	dis.cc
	disp_mod.cc
	download.cc
	errno.cc
	fault.cc
	fp.cc
	gdb.cc
	hello.cc
	int_rout.cc
	io.cc
	monitor.cc
	no_fp.cc
	parse.cc
	trace.cc
	block_io.h
	break.h
	bus.h
	defines.h
	globals.h
	iac.h
	IORoutines.h
	memmap.h
	qtcommon.h
	regs.h
	stop.h
	version.h)

add_executable(nindy main.cc)
