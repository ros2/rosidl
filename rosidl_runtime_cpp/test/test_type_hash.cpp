// Copyright 2023, Foxglove
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

#include "rosidl_runtime_cpp/type_hash.hpp"

using namespace ::testing;  // NOLINT

TEST(TestTypeHash, unset_behavior) {
  rosidl_type_hash_t type_hash;
  type_hash.version = 0;
  std::string result = rosidl_runtime_cpp::type_hash_to_string(type_hash);
  EXPECT_EQ("", result);
}

TEST(TestTypeHash, v1_format) {
  rosidl_type_hash_t type_hash;
  type_hash.version = 1;
  for (int i = 0; i < ROSIDL_TYPE_HASH_SIZE; ++i) {
    type_hash.value[i] = i;
  }
  std::string result = rosidl_runtime_cpp::type_hash_to_string(type_hash);
  EXPECT_EQ("RIHS01_000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f", result);
}
