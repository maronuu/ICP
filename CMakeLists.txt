cmake_minimum_required(VERSION 2.8)

project(myicp CXX)

set(CMAKE_CXX_FLAGS "-std=c++17 -Wall")

add_subdirectory(src)
add_subdirectory(test)