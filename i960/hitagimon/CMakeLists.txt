cmake_minimum_required(VERSION 3.10)
project(hitagimon VERSION 0.1)
enable_language(CXX C ASM-ATT)
set(CMAKE_CXX_STANDARD 98)

set(tools /home/jwscoggins/sys/i960-elf-gcc-3.4.6)
set(CMAKE_C_COMPILER ${tools}/bin/i960-elf-gcc)
set(CMAKE_CXX_COMPILER ${tools}/bin/i960-elf-g++)
set(CMAKE_ASM-ATT_COMPILER ${tools}/bin/i960-elf-as)
set(CMAKE_ASM-ATT_FLAGS "${CMAKE_ASM_FLAGS} -ASB")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msb -mnumerics -O1")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -msb -mnumerics -O1")
set(LINKER_SCRIPT "start.ld")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--as-needed -nostdlib -static -T ${CMAKE_SOURCE_DIR}/${LINKER_SCRIPT} -msb -mnumerics -ffreestanding")
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
add_executable(hitagimon
		sx_init.s # this must ALWAYS be first!
		f_table.s
		i_handle.s
		i_table.s
		f_handle.cc
		ISR.cc
        # interface libraries
		IODevice.cc
		IODevice.h
		ChipsetInteract.cc
		ChipsetInteract.h
        IORoutines.h
		console.cc
		console.h
		brk.cc
		main.cc)

#set_target_properties(hitagimon PROPERTIES LINK_DEPENDS ${CMAKE_SOURCE_DIR}/${LINKER_SCRIPT})
target_link_libraries(hitagimon
		gcc
		c
		m
		nosys # stub library
		stdc++
		c
		m
		gcc)