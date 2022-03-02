cmake_minimum_required(VERSION 3.12)
project(test)

# google test
set(GTEST_ROOT ${PROJECT_SOURCE_DIR}/../external/googletestlib)
find_package( GTest REQUIRED )
# eigen
set(CMAKE_PREFIX_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../external/eigenlib)
find_package(Eigen3 REQUIRED)
# pthread
find_package(Threads REQUIRED)

enable_testing()

add_executable(icp_test icp_test.cpp)

target_link_libraries(icp_test 
    Eigen3::Eigen
    gtest
    gtest_main
    Threads::Threads
)