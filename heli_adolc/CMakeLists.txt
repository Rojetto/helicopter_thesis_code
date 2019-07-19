cmake_minimum_required(VERSION 3.14)
project(heli_adolc)

set(CMAKE_CXX_STANDARD 11)

include_directories(include)
link_directories(adolc_lib)

add_library(heli_adolc adolc_test.cpp adolc_test.h)

target_link_libraries(heli_adolc adolc)