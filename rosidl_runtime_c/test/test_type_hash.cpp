// Copyright 2023 Open Source Robotics Foundation, Inc.
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

#include "gtest/gtest.h"

#include "rcutils/error_handling.h"
#include "rosidl_runtime_c/type_hash.h"

TEST(type_hash, init_zero_hash) {
  auto hash = rosidl_get_zero_initialized_type_hash();
  EXPECT_EQ(hash.version, 0);
  for (size_t i = 0; i < sizeof(hash.value); i++) {
    EXPECT_EQ(hash.value[i], 0);
  }
}

TEST(type_hash, stringify_basic) {
  const std::string expected =
    "RIHS1_00112233445566778899aabbccddeeff00112233445566778899aabbccddeeff";
  rosidl_type_hash_t hash = rosidl_get_zero_initialized_type_hash();
  hash.version = 1;
  for (size_t i = 0; i < sizeof(hash.value); i++) {
    hash.value[i] = (i % 0x10) * 0x10 + (i % 0x10);
  }
  auto allocator = rcutils_get_default_allocator();
  char * hash_string = nullptr;
  ASSERT_EQ(RCUTILS_RET_OK, rosidl_stringify_type_hash(&hash, allocator, &hash_string));
  ASSERT_TRUE(hash_string);

  std::string cpp_str(hash_string);
  EXPECT_EQ(expected, hash_string);
}

TEST(type_hash, parse_basic) {
  const std::string test_value =
    "RIHS1_00112233445566778899aabbccddeeff00112233445566778899aabbccddeeff";

  rosidl_type_hash_t hash = rosidl_get_zero_initialized_type_hash();
  ASSERT_EQ(RCUTILS_RET_OK, rosidl_parse_type_hash_string(test_value.c_str(), &hash));
  EXPECT_EQ(1, hash.version);
  for (size_t i = 0; i < sizeof(hash.value); i++) {
    size_t expected_value = (i % 0x10) * 0x10 + (i % 0x10);
    EXPECT_EQ(expected_value, hash.value[i]);
  }
}

TEST(type_hash, parse_bad_prefix) {
  const std::string test_value =
    "RRRR1_00112233445566778899aabbccddeeff00112233445566778899aabbccddeeff";
  rosidl_type_hash_t hash = rosidl_get_zero_initialized_type_hash();
  EXPECT_EQ(RCUTILS_RET_INVALID_ARGUMENT, rosidl_parse_type_hash_string(test_value.c_str(), &hash));
  rcutils_reset_error();
}

TEST(type_hash, parse_no_version) {
  const std::string test_value =
    "RIHS_00112233445566778899aabbccddeeff00112233445566778899aabbccddeeff";
  rosidl_type_hash_t hash = rosidl_get_zero_initialized_type_hash();
  EXPECT_EQ(RCUTILS_RET_INVALID_ARGUMENT, rosidl_parse_type_hash_string(test_value.c_str(), &hash));
  rcutils_reset_error();
}

TEST(type_hash, parse_too_short) {
  const std::string test_value =
    "RIHS1_00112233445566778899aabbccddeeff00112233445566778899aabbccddee";
  rosidl_type_hash_t hash = rosidl_get_zero_initialized_type_hash();
  EXPECT_EQ(RCUTILS_RET_INVALID_ARGUMENT, rosidl_parse_type_hash_string(test_value.c_str(), &hash));
  rcutils_reset_error();
}

TEST(type_hash, parse_too_long) {
  const std::string test_value =
    "RIHS1_00112233445566778899aabbccddeeff00112233445566778899aabbccddeeff00";
  rosidl_type_hash_t hash = rosidl_get_zero_initialized_type_hash();
  EXPECT_EQ(RCUTILS_RET_INVALID_ARGUMENT, rosidl_parse_type_hash_string(test_value.c_str(), &hash));
  rcutils_reset_error();
}

TEST(type_hash, parse_multidigit_version) {
  const std::string test_value =
    "RIHS213_00112233445566778899aabbccddeeff00112233445566778899aabbccddeeff";
  rosidl_type_hash_t hash = rosidl_get_zero_initialized_type_hash();
  ASSERT_EQ(RCUTILS_RET_OK, rosidl_parse_type_hash_string(test_value.c_str(), &hash));
  EXPECT_EQ(213, hash.version);
}
