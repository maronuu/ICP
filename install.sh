git submodule update external/glfw
cd external/glfw
mkdir build
cd build
cmake ..
cmake --build .
cmake --install . --prefix ../../glfwlib
