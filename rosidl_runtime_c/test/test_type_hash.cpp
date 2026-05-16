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

// Helper to build a RIHS01 hash value: version=1, value bytes 0..31
rosidl_type_hash_t make_rihs01_sample()
{
  rosidl_type_hash_t hash = rosidl_get_zero_initialized_type_hash();
  hash.version = 1;
  for (uint8_t i = 0; i < ROSIDL_TYPE_HASH_SIZE; i++) {
    hash.value[i] = i;
  }
  return hash;
}

// Helper to build a RIHS02 hash value: version=2, first 16 bytes 0..15
rosidl_type_hash_t make_rihs02_sample()
{
  rosidl_type_hash_t hash = rosidl_get_zero_initialized_type_hash();
  hash.version = 2;
  for (uint8_t i = 0; i < 16; i++) {
    hash.value[i] = i;
  }
  return hash;
}

TEST(type_hash, init_zero_hash) {
  auto hash = rosidl_get_zero_initialized_type_hash();
  EXPECT_EQ(hash.version, 0);
  for (size_t i = 0; i < sizeof(hash.value); i++) {
    EXPECT_EQ(hash.value[i], 0);
  }
}

TEST(type_hash, stringify_rihs01) {
  const std::string expected =
    "RIHS01_000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f";
  auto hash = make_rihs01_sample();
  auto allocator = rcutils_get_default_allocator();
  char * hash_string = nullptr;
  ASSERT_EQ(RCUTILS_RET_OK, rosidl_stringify_type_hash(&hash, allocator, &hash_string));
  ASSERT_TRUE(hash_string);
  EXPECT_EQ(expected, std::string(hash_string));
  allocator.deallocate(hash_string, allocator.state);
}

TEST(type_hash, stringify_rihs02) {
  const std::string expected =
    "RIHS02_000102030405060708090a0b0c0d0e0f";
  auto hash = make_rihs02_sample();
  auto allocator = rcutils_get_default_allocator();
  char * hash_string = nullptr;
  ASSERT_EQ(RCUTILS_RET_OK, rosidl_stringify_type_hash(&hash, allocator, &hash_string));
  ASSERT_TRUE(hash_string);
  EXPECT_EQ(expected, std::string(hash_string));
  allocator.deallocate(hash_string, allocator.state);
}

TEST(type_hash, parse_rihs01) {
  const std::string test_value =
    "RIHS01_000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f";
  rosidl_type_hash_t hash = rosidl_get_zero_initialized_type_hash();
  ASSERT_EQ(RCUTILS_RET_OK, rosidl_parse_type_hash_string(test_value.c_str(), &hash));
  EXPECT_EQ(1, hash.version);
  for (size_t i = 0; i < ROSIDL_TYPE_HASH_SIZE; i++) {
    EXPECT_EQ(i, hash.value[i]) << "At byte " << i;
  }
  // Remaining bytes (for RIHS01, all 32 are filled) are already checked above.
}

TEST(type_hash, parse_rihs02) {
  const std::string test_value =
    "RIHS02_000102030405060708090a0b0c0d0e0f";
  rosidl_type_hash_t hash = rosidl_get_zero_initialized_type_hash();
  ASSERT_EQ(RCUTILS_RET_OK, rosidl_parse_type_hash_string(test_value.c_str(), &hash));
  EXPECT_EQ(2, hash.version);
  for (size_t i = 0; i < 16; i++) {
    EXPECT_EQ(i, hash.value[i]) << "At byte " << i;
  }
  // Upper 16 bytes must be zero
  for (size_t i = 16; i < ROSIDL_TYPE_HASH_SIZE; i++) {
    EXPECT_EQ(0, hash.value[i]) << "Upper byte " << i << " not zero";
  }
}

TEST(type_hash, parse_bad_prefix) {
  const std::string test_value =
    "RRRR01_00112233445566778899aabbccddeeff00112233445566778899aabbccddeeff";
  rosidl_type_hash_t hash = rosidl_get_zero_initialized_type_hash();
  EXPECT_EQ(RCUTILS_RET_INVALID_ARGUMENT,
            rosidl_parse_type_hash_string(test_value.c_str(), &hash));
  rcutils_reset_error();
}

TEST(type_hash, parse_no_version) {
  const std::string test_value =
    "RIHS_00112233445566778899aabbccddeeff00112233445566778899aabbccddeeff";
  rosidl_type_hash_t hash = rosidl_get_zero_initialized_type_hash();
  EXPECT_EQ(RCUTILS_RET_INVALID_ARGUMENT,
            rosidl_parse_type_hash_string(test_value.c_str(), &hash));
  rcutils_reset_error();
}

TEST(type_hash, parse_too_short_for_rihs01) {
  // A RIHS01 string with only 62 hex chars (should be 64)
  const std::string test_value =
    "RIHS01_00112233445566778899aabbccddeeff00112233445566778899aabbccddee";
  rosidl_type_hash_t hash = rosidl_get_zero_initialized_type_hash();
  EXPECT_EQ(RCUTILS_RET_INVALID_ARGUMENT,
            rosidl_parse_type_hash_string(test_value.c_str(), &hash));
  rcutils_reset_error();
}

TEST(type_hash, parse_too_long_for_rihs01) {
  const std::string test_value =
    "RIHS01_00112233445566778899aabbccddeeff00112233445566778899aabbccddeeff00";
  rosidl_type_hash_t hash = rosidl_get_zero_initialized_type_hash();
  EXPECT_EQ(RCUTILS_RET_INVALID_ARGUMENT,
            rosidl_parse_type_hash_string(test_value.c_str(), &hash));
  rcutils_reset_error();
}

TEST(type_hash, parse_too_short_for_rihs02) {
  // A RIHS02 string with 30 hex chars (should be 32)
  const std::string test_value = "RIHS02_00112233445566778899aabbccddee";
  rosidl_type_hash_t hash = rosidl_get_zero_initialized_type_hash();
  EXPECT_EQ(RCUTILS_RET_INVALID_ARGUMENT,
            rosidl_parse_type_hash_string(test_value.c_str(), &hash));
  rcutils_reset_error();
}

TEST(type_hash, parse_too_long_for_rihs02) {
  const std::string test_value = "RIHS02_00112233445566778899aabbccddeeff00";
  rosidl_type_hash_t hash = rosidl_get_zero_initialized_type_hash();
  EXPECT_EQ(RCUTILS_RET_INVALID_ARGUMENT,
            rosidl_parse_type_hash_string(test_value.c_str(), &hash));
  rcutils_reset_error();
}

TEST(type_hash, parse_bad_value) {
  const std::string test_value =
    "RIHS01_00112233445566778899aabbccddgeff00112233445566778899aabbccddeeff";
  rosidl_type_hash_t hash = rosidl_get_zero_initialized_type_hash();
  EXPECT_EQ(RCUTILS_RET_INVALID_ARGUMENT,
            rosidl_parse_type_hash_string(test_value.c_str(), &hash));
  EXPECT_EQ(1, hash.version);  // version should have been extracted before error
  rcutils_reset_error();
}

// Test the public helper that returns the hash value size
TEST(type_hash, get_value_size) {
  // RIHS01 → 32 bytes
  auto hash01 = make_rihs01_sample();
  EXPECT_EQ(32u, rosidl_type_hash_get_value_size(hash01.version));

  // RIHS02 → 16 bytes
  auto hash02 = make_rihs02_sample();
  EXPECT_EQ(16u, rosidl_type_hash_get_value_size(hash02.version));

  // Unknown version → 0
  rosidl_type_hash_t unknown = rosidl_get_zero_initialized_type_hash();
  unknown.version = 99;
  EXPECT_EQ(0u, rosidl_type_hash_get_value_size(unknown.version));
}
