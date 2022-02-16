cmake_minimum_required(VERSION 3.1.2)

add_executable(test_icp test_icp.cpp)

target_link_libraries(test_icp Eigen3::Eigen)