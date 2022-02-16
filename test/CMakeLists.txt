cmake_minimum_required(VERSION 3.12)

enable_testing()

add_executable(icp_test icp_test.cpp)

target_link_libraries(icp_test Eigen3::Eigen)
target_link_libraries(icp_test gtest_main)

include(GoogleTest)
gtest_discover_tests(icp_test)