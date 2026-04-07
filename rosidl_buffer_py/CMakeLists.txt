cmake_minimum_required(VERSION 3.20)
project(rosidl_buffer_py)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_buffer REQUIRED)

# Find python before pybind11
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)

# Build the pybind11 C++ extension module
pybind11_add_module(_rosidl_buffer_py
  src/rosidl_buffer_py.cpp
)

target_link_libraries(_rosidl_buffer_py PRIVATE
  rosidl_buffer::rosidl_buffer
)

# Install the Python package (must come before install(TARGETS) to set PYTHON_INSTALL_DIR)
ament_python_install_package(rosidl_buffer)

# Install the C++ extension module into the Python package directory
install(TARGETS _rosidl_buffer_py
  DESTINATION "${PYTHON_INSTALL_DIR}/rosidl_buffer"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
