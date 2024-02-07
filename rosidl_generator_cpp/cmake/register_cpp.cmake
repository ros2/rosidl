# Copyright 2023 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

macro(rosidl_generator_cpp_extras GENERATOR_FILES TEMPLATE_DIR)
  find_package(ament_cmake_core QUIET REQUIRED)
  find_package(rosidl_generator_type_description QUIET REQUIRED)

  ament_register_extension(
    "rosidl_write_generator_arguments_extensions"
    "rosidl_generator_cpp"
    "rosidl_generator_cpp_write_arguments.cmake")
  ament_register_extension(
    "rosidl_generate_idl_interfaces"
    "rosidl_generator_cpp"
    "rosidl_generator_cpp_generate_interfaces.cmake")

  normalize_path(GENERATOR_FILES "${GENERATOR_FILES}")
  set(rosidl_generator_cpp_GENERATOR_FILES "${GENERATOR_FILES}")

  normalize_path(TEMPLATE_DIR "${TEMPLATE_DIR}")
  set(rosidl_generator_cpp_TEMPLATE_DIR "${TEMPLATE_DIR}")
endmacro()
