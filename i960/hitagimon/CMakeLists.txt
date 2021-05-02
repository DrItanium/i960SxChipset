cmake_minimum_required(VERSION 3.10)
project(nindypp VERSION 0.1)
enable_language(CXX C ASM-ATT)
set(CMAKE_CXX_STANDARD 98)

set(tools /home/jwscoggins/sys/i960-elf-gcc-3.4.6)
set(CMAKE_C_COMPILER ${tools}/bin/i960-elf-gcc)
set(CMAKE_CXX_COMPILER ${tools}/bin/i960-elf-g++)
set(CMAKE_ASM-ATT_COMPILER ${tools}/bin/i960-elf-as)
set(CMAKE_ASM-ATT_FLAGS "${CMAKE_ASM_FLAGS} -ASB")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msb -mnumerics")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -msb -mnumerics")
set(CMAKE_STATIC_LINKER_FLAGS "${CMAKE_LD_FLAGS} -T start.ld -msb -mnumerics")
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_LD_FLAGS} -T start.ld -msb -mnumerics")
set(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_LD_FLAGS} -T start.ld -msb -mnumerics")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

add_executable(hitagimon
		brk.cc
		f_handle.c
		f_table.s
		i_handle.s
		ISR.c
		i_table.s
		sx_init.s
		main.cc)