cmake_minimum_required(VERSION 3.12)

######################################################

enable_language(CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
IF(MSVC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")
ELSE()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -g")
ENDIF()

######################################################

project(01_vis_test)

# glfw
set(CMAKE_PREFIX_PATH ../external/glfwlib)
find_package(glfw3 REQUIRED)

# opengl
find_package(OpenGL REQUIRED)

######################################################

add_executable(${PROJECT_NAME}
  main.cpp
  )

target_link_libraries(${PROJECT_NAME}
  OpenGL::GL
  glfw
  )