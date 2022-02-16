cmake_minimum_required(VERSION 3.12)

add_executable(main main.cpp)

target_include_directories(main
    PUBLIC ${PROJECT_SOURCE_DIR}/my_icp
)

target_link_libraries(main my_icp)
target_link_libraries(main Eigen3::Eigen)
