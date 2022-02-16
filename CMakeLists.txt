cmake_minimum_required(VERSION 3.12)

# C++17
set(CMAKE_CXX_STANDARD 17)

# for debug
# set(CMAKE_VERBOSE_MAKEFILE 1)

# packages
find_package(Eigen3 REQUIRED NO_MODULE)

# project name
project(00_demo2d CXX)

# Google Test
include(FetchContent)
FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/609281088cfefc76f9d0ce82e1ff6c30cc3591e5.zip
)
FetchContent_MakeAvailable(googletest)

# compile flags
set(CMAKE_CXX_FLAGS "-std=c++17 -Wall")

# subdirectories
add_subdirectory(00_demo2d)
add_subdirectory(my_icp)
add_subdirectory(test)
