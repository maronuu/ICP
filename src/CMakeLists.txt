add_executable(main main.cpp icp.cpp)

target_include_directories(main
    PUBLIC ${PROJECT_SOURCE_DIR}/include
)
