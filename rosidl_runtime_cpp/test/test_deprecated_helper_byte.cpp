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

#include "rosidl_runtime_cpp/deprecated_helper_byte.hpp"


// Suppress deprecation notices to allow unit testing of deprecated functions.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"


TEST(rosidl_generator_cpp, deprecated_helper_byte_getter_setter) {
  rosidl_runtime_cpp::DeprecatedHelperByte v;

  v = std::byte{1};
  ASSERT_EQ(static_cast<std::byte>(v), std::byte{1});
  ASSERT_EQ(v, 1);
  v = 3;
  ASSERT_EQ(static_cast<std::byte>(v), std::byte{3});
  ASSERT_EQ(v, 3);

  v = std::byte{10};
  unsigned char c = v;
  ASSERT_EQ(c, 10);

  std::byte d = static_cast<std::byte>(v);
  ASSERT_EQ(d, std::byte{10});

  auto e = v;
  ASSERT_EQ(e, 10);
}

TEST(rosidl_generator_cpp, deprecated_helper_byte_equal) {
  rosidl_runtime_cpp::DeprecatedHelperByte f = 10;
  rosidl_runtime_cpp::DeprecatedHelperByte g = std::byte{10};
  ASSERT_EQ(10, f);
  ASSERT_EQ(f, 10);
  ASSERT_EQ(std::byte{10}, static_cast<std::byte>(g));
  ASSERT_EQ(static_cast<std::byte>(g), std::byte{10});
  ASSERT_EQ(f, g);
}

TEST(rosidl_generator_cpp, deprecated_helper_byte_not_equal) {
  rosidl_runtime_cpp::DeprecatedHelperByte f = 8;
  rosidl_runtime_cpp::DeprecatedHelperByte g = std::byte{10};
  ASSERT_NE(9, f);
  ASSERT_NE(f, 9);
  ASSERT_NE(std::byte{11}, static_cast<std::byte>(g));
  ASSERT_NE(static_cast<std::byte>(g), std::byte{11});
  ASSERT_NE(f, g);
}

TEST(rosidl_generator_cpp, deprecated_helper_byte_to_integer) {
  rosidl_runtime_cpp::DeprecatedHelperByte g = std::byte{10};
  ASSERT_EQ(std::to_integer<int>(static_cast<std::byte>(g)), 10);
}

TEST(rosidl_generator_cpp, deprecated_helper_byte_function_input) {
  rosidl_runtime_cpp::DeprecatedHelperByte a = 10;
  auto from_deprecated_helper_byte = [](rosidl_runtime_cpp::DeprecatedHelperByte b) {return b;};
  auto from_unsigned_char = [](unsigned char b) {return b;};
  auto from_byte = [](std::byte b) {return b;};

  ASSERT_EQ(from_deprecated_helper_byte(a), 10);
  ASSERT_EQ(from_unsigned_char(a), 10);
  ASSERT_EQ(from_byte(static_cast<std::byte>(a)), std::byte{10});
}

TEST(rosidl_generator_cpp, deprecated_helper_byte_shifts) {
  rosidl_runtime_cpp::DeprecatedHelperByte v = std::byte{1};
  ASSERT_EQ(v << 3, 8);
  v <<= 3;
  ASSERT_EQ(v, 8);

  rosidl_runtime_cpp::DeprecatedHelperByte g = std::byte{2};
  ASSERT_EQ(static_cast<std::byte>(g) << 3, std::byte{16});
  static_cast<std::byte&>(g) <<= 3;
  ASSERT_EQ(static_cast<std::byte>(g), std::byte{16});
}

#pragma GCC diagnostic pop
