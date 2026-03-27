// Copyright 2025 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <type_traits>

#include "rosidl_runtime_cpp/deprecated_helper_array.hpp"


// Suppress deprecation notices to allow unit testing of deprecated functions.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"


TEST(rosidl_generator_cpp, deprecated_helper_array_fill) {
  rosidl_runtime_cpp::DeprecatedHelperArray<3> dep_array;
  std::array<std::byte, 3> byte_array = {std::byte{10}, std::byte{10}, std::byte{10}};
  std::array<unsigned char, 3> char_array = {10, 10, 10};
  dep_array.fill(std::byte{10});
  EXPECT_EQ(dep_array, byte_array);
  EXPECT_EQ(dep_array, char_array);

  dep_array.fill(35);
  byte_array.fill(std::byte{35});
  char_array.fill(35);

  EXPECT_EQ(dep_array, byte_array);
  EXPECT_EQ(dep_array, char_array);
}

TEST(rosidl_generator_cpp, deprecated_helper_array_tuple_element) {
  rosidl_runtime_cpp::DeprecatedHelperArray<3> dep_array;
  using T = std::tuple_element<0, decltype(dep_array)>::type;
  static_assert(std::is_same_v<T, unsigned char>);

  using B = std::tuple_element<0, std::array<std::byte, 10>>::type;
  static_assert(std::is_same_v<B, std::byte>);
}

#pragma GCC diagnostic pop
