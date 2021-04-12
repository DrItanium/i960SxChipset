cmake_minimum_required(VERSION 3.10)

project(chipset VERSION 0.1)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_executable(chipset chipset.cc)

target_link_libraries(chipset PUBLIC boost_system boost_program_options pthread)
