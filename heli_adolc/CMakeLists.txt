cmake_minimum_required(VERSION 3.14)
project(heli_adolc)

set(CMAKE_CXX_STANDARD 11)

include_directories(include)

add_library(heli_adolc heli_adolc.cpp heli_adolc.h)

link_directories(adolc)
add_executable(heli_adolc_test heli_adolc_test.cpp)
target_link_libraries(heli_adolc_test heli_adolc)
target_link_libraries(heli_adolc_test adolc)