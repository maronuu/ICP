git submodule update external/glfw
cd external/glfw
mkdir build
cd build
cmake ..
cmake --build .
cmake --install . --prefix ../../glfwlib

git submodule update external/eigen
cd external/eigen
mkdir build
cd build
cmake ..
cmake --build .
cmake --install . --prefix ../../eigenlib

git submodule update external/googletest
cd external/googletest
mkdir build
cd build
cmake ..
cmake --build .
cmake --install . --prefix ../../googletestlib

