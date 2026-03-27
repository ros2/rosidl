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

#include "rosidl_runtime_cpp/byte_conversion_helpers.hpp"


// Suppress deprecation notices to allow unit testing of deprecated functions.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"


TEST(rosidl_generator_cpp, byte_ref) {
  std::byte b{10};

  rosidl_runtime_cpp::detail::ByteRef byte_ref{b};

  byte_ref = 3;

  ASSERT_EQ(byte_ref, 3);
  ASSERT_EQ(static_cast<std::byte>(byte_ref), std::byte{3});

  byte_ref = std::byte{4};

  ASSERT_EQ(byte_ref, 4);
  ASSERT_EQ(static_cast<std::byte>(byte_ref), std::byte{4});

  std::byte bb = static_cast<std::byte>(byte_ref);
  unsigned char uchar = byte_ref;

  ASSERT_EQ(bb, std::byte{4});
  ASSERT_EQ(uchar, 4);
}

TEST(rosidl_generator_cpp, const_byte_ref) {
  const std::byte b{5};
  rosidl_runtime_cpp::detail::ConstByteRef const_ref{b};

  EXPECT_EQ(const_ref, 5);
  EXPECT_EQ(static_cast<std::byte>(const_ref), std::byte{5});

  std::byte bb = static_cast<std::byte>(const_ref);
  unsigned char uchar = const_ref;

  EXPECT_EQ(bb, std::byte{5});
  EXPECT_EQ(uchar, 5);
}

TEST(rosidl_generator_cpp, byte_ptr) {
  std::byte b{42};
  rosidl_runtime_cpp::detail::BytePtr byte_ptr{&b};

  EXPECT_EQ(*byte_ptr, 42);

  *byte_ptr = std::byte{99};
  EXPECT_EQ(b, std::byte{99});

  std::byte byte_2{25};
  byte_ptr = &byte_2;
  EXPECT_EQ(*byte_ptr, 25);
  EXPECT_EQ(static_cast<std::byte>(*byte_ptr), std::byte{25});

  unsigned char uchar2 = 13;
  byte_ptr = &uchar2;
  EXPECT_EQ(*byte_ptr, 13);
  EXPECT_EQ(static_cast<std::byte>(*byte_ptr), std::byte{13});

  *byte_ptr = 100;
  EXPECT_EQ(*byte_ptr, 100);
  EXPECT_EQ(uchar2, 100);

  std::byte bb = static_cast<std::byte>(*byte_ptr);
  EXPECT_EQ(bb, std::byte{100});

  unsigned char uchar = *byte_ptr;
  EXPECT_EQ(uchar, 100);

  std::byte foo{25};
  std::byte *bstar = &foo;
  byte_ptr = bstar;
  EXPECT_EQ(static_cast<std::byte *>(byte_ptr), bstar);

  unsigned char uchar3 = 25;
  unsigned char *ustar = &uchar3;
  byte_ptr = ustar;
  EXPECT_EQ(byte_ptr, ustar);
}

TEST(rosidl_generator_cpp, const_byte_ptr) {
  const std::byte b{200};
  rosidl_runtime_cpp::detail::ConstBytePtr const_byte_ptr{&b};
  EXPECT_EQ(*const_byte_ptr, 200);
  EXPECT_EQ(static_cast<std::byte>(*const_byte_ptr), std::byte{200});

  std::byte bb = static_cast<std::byte>(*const_byte_ptr);
  EXPECT_EQ(bb, std::byte{200});

  unsigned char uchar = *const_byte_ptr;
  EXPECT_EQ(uchar, 200);
}

#pragma GCC diagnostic pop
