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

#include "rosidl_runtime_cpp/deprecated_helper_array.hpp"


// Suppress deprecation notices to allow unit testing of deprecated functions.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"


TEST(rosidl_generator_cpp, deprecated_helper_array_getter_setter) {
  rosidl_runtime_cpp::DeprecatedHelperArray<3> dep_array{};
  std::array<std::byte, 3> byte_array = {std::byte{0}, std::byte{0}, std::byte{10}};
  std::array<unsigned char, 3> char_array = {0, 0, 10};

  dep_array[2] = std::byte{10};

  EXPECT_EQ(dep_array, byte_array);
  EXPECT_EQ(dep_array, char_array);

  std::array<unsigned char, 3> char_array2 = {0, 0, 10};
  dep_array = char_array2;

  // v = {{std::byte{1}}};
  // ASSERT_EQ(static_cast<std::byte>(v), std::byte{1});
  // ASSERT_EQ(v, 1);
  // v = {{3}};
  // ASSERT_EQ(static_cast<std::byte>(v), std::byte{3});
  // ASSERT_EQ(v, 3);

  // v = std::byte{10};
  // unsigned char c = v;
  // ASSERT_EQ(c, 10);

  // std::byte d = static_cast<std::byte>(v);
  // ASSERT_EQ(d, std::byte{10});

  // auto e = v;
  // ASSERT_EQ(e, 10);
}


TEST(rosidl_generator_cpp, deprecated_helper_array_size) {
  rosidl_runtime_cpp::DeprecatedHelperArray<3> dep_array;
  EXPECT_EQ(dep_array.size(), 3);
}

TEST(rosidl_generator_cpp, deprecated_helper_fill) {
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

#pragma GCC diagnostic pop
