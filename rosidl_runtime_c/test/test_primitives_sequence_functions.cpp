// Copyright 2020 Open Source Robotics Foundation, Inc.
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
#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#define TEST_PRIMITIVE_SEQUENCE_FUNCTION_INIT_FINI(STRUCT_NAME) \
  TEST(primitives_sequence_functions, test_ ## STRUCT_NAME ## _init_fini) \
  { \
    EXPECT_FALSE(rosidl_runtime_c__ ## STRUCT_NAME ## __Sequence__init(nullptr, 0u)); \
    rosidl_runtime_c__ ## STRUCT_NAME ## __Sequence sequence; \
    EXPECT_TRUE(rosidl_runtime_c__ ## STRUCT_NAME ## __Sequence__init(&sequence, 0u)); \
    EXPECT_EQ(sequence.data, nullptr); \
    EXPECT_EQ(sequence.size, 0u); \
    EXPECT_EQ(sequence.capacity, 0u); \
 \
    /* Shouldn't do anything exciting */ \
    rosidl_runtime_c__ ## STRUCT_NAME ## __Sequence__fini(&sequence); \
 \
    constexpr size_t seq_size = 3u; \
    EXPECT_TRUE(rosidl_runtime_c__ ## STRUCT_NAME ## __Sequence__init(&sequence, seq_size)); \
    EXPECT_EQ(sequence.size, seq_size); \
    EXPECT_EQ(sequence.capacity, seq_size); \
    EXPECT_NE(sequence.data, nullptr); \
    rosidl_runtime_c__ ## STRUCT_NAME ## __Sequence__fini(&sequence); \
  }

TEST_PRIMITIVE_SEQUENCE_FUNCTION_INIT_FINI(float)
TEST_PRIMITIVE_SEQUENCE_FUNCTION_INIT_FINI(double)
TEST_PRIMITIVE_SEQUENCE_FUNCTION_INIT_FINI(long_double)
TEST_PRIMITIVE_SEQUENCE_FUNCTION_INIT_FINI(char)
TEST_PRIMITIVE_SEQUENCE_FUNCTION_INIT_FINI(wchar)
TEST_PRIMITIVE_SEQUENCE_FUNCTION_INIT_FINI(boolean)
TEST_PRIMITIVE_SEQUENCE_FUNCTION_INIT_FINI(octet)
TEST_PRIMITIVE_SEQUENCE_FUNCTION_INIT_FINI(uint8)
TEST_PRIMITIVE_SEQUENCE_FUNCTION_INIT_FINI(int8)
TEST_PRIMITIVE_SEQUENCE_FUNCTION_INIT_FINI(uint16)
TEST_PRIMITIVE_SEQUENCE_FUNCTION_INIT_FINI(int16)
TEST_PRIMITIVE_SEQUENCE_FUNCTION_INIT_FINI(uint32)
TEST_PRIMITIVE_SEQUENCE_FUNCTION_INIT_FINI(int32)
TEST_PRIMITIVE_SEQUENCE_FUNCTION_INIT_FINI(uint64)
TEST_PRIMITIVE_SEQUENCE_FUNCTION_INIT_FINI(int64)

// Testing legacy API
TEST_PRIMITIVE_SEQUENCE_FUNCTION_INIT_FINI(bool)
TEST_PRIMITIVE_SEQUENCE_FUNCTION_INIT_FINI(byte)
TEST_PRIMITIVE_SEQUENCE_FUNCTION_INIT_FINI(float32)
TEST_PRIMITIVE_SEQUENCE_FUNCTION_INIT_FINI(float64)
