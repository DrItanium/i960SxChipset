cmake_minimum_required(VERSION 3.10)

project(ramblock VERSION 0.1)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_executable(ramblock ramblock.cc)

target_link_libraries(ramblock PUBLIC boost_system pthread)
