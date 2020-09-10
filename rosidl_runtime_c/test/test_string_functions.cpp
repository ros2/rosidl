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
#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"

#include "rcutils/testing/fault_injection.h"

TEST(string_functions, init_fini_empty_string) {
  rosidl_runtime_c__String empty_string;
  EXPECT_TRUE(rosidl_runtime_c__String__init(&empty_string));
  EXPECT_EQ(empty_string.size, 0u);
  EXPECT_EQ(empty_string.capacity, 1u);
  EXPECT_EQ(empty_string.data[0], 0);

  // Should just return without changing anything.
  rosidl_runtime_c__String__fini(nullptr);

  rosidl_runtime_c__String__fini(&empty_string);
  EXPECT_EQ(empty_string.data, nullptr);
  EXPECT_EQ(empty_string.size, 0u);
  EXPECT_EQ(empty_string.capacity, 0u);
}

TEST(string_functions, init_fini_bad_data) {
  EXPECT_FALSE(rosidl_runtime_c__String__init(nullptr));

  rosidl_runtime_c__String bad_string;
  bad_string.size = 1u;
  bad_string.capacity = 1u;
  bad_string.data = nullptr;

  int expected_error_code = -1;
#ifndef _WIN32
  // While exit() takes an int, when a parent process waits for a child process to die, it only
  // takes the lowest 8-bits.
  // See Note: in https://pubs.opengroup.org/onlinepubs/009695399/functions/exit.html
  expected_error_code &= 255;
#endif
  EXPECT_EXIT(
    rosidl_runtime_c__String__fini(&bad_string),
    ::testing::ExitedWithCode(expected_error_code),
    "Unexpected condition: string size was non-zero for deallocated data! Exiting.");
  bad_string.size = 0u;

  EXPECT_EXIT(
    rosidl_runtime_c__String__fini(&bad_string),
    ::testing::ExitedWithCode(expected_error_code),
    "Unexpected behavior: string capacity was non-zero for deallocated data! Exiting.");

  char c = 0;
  bad_string.data = &c;
  bad_string.capacity = 0u;

  EXPECT_EXIT(
    rosidl_runtime_c__String__fini(&bad_string),
    ::testing::ExitedWithCode(expected_error_code),
    "Unexpected condition: string capacity was zero for allocated data! Exiting.");
}

TEST(string_functions, resize_assignn) {
  rosidl_runtime_c__String s, t;
  constexpr size_t s_size = 3u;
  constexpr char data[s_size + 1] = {3u, 2u, 1u, 0u};

  EXPECT_TRUE(rosidl_runtime_c__String__init(&s));
  EXPECT_TRUE(rosidl_runtime_c__String__init(&t));

  EXPECT_FALSE(rosidl_runtime_c__String__assign(nullptr, nullptr));
  EXPECT_FALSE(rosidl_runtime_c__String__assignn(nullptr, nullptr, 0));
  EXPECT_FALSE(rosidl_runtime_c__String__assignn(&s, nullptr, 0));
  EXPECT_FALSE(rosidl_runtime_c__String__assignn(nullptr, &data[0], 0));

  EXPECT_TRUE(rosidl_runtime_c__String__assign(&s, &data[0]));
  EXPECT_EQ(s.size, s_size);
  EXPECT_EQ(s.capacity, s_size + 1u);
  EXPECT_EQ(s.data[0], data[0]);
  EXPECT_EQ(s.data[1], data[1]);
  EXPECT_EQ(s.data[2], data[2]);
  EXPECT_EQ(s.data[3], 0);

  EXPECT_TRUE(rosidl_runtime_c__String__assignn(&t, &data[0], s_size));
  EXPECT_EQ(t.size, s_size);
  EXPECT_EQ(t.capacity, s_size + 1u);
  EXPECT_EQ(t.data[0], data[0]);
  EXPECT_EQ(t.data[1], data[1]);
  EXPECT_EQ(t.data[2], data[2]);
  EXPECT_EQ(t.data[3], 0);

  // Can't copy strings of size max
  EXPECT_FALSE(rosidl_runtime_c__String__assignn(&s, &data[0], SIZE_MAX));

  // Check assigning 0-length strings
  EXPECT_TRUE(rosidl_runtime_c__String__assignn(&s, &data[0], 0));
  EXPECT_EQ(s.size, 0u);
  EXPECT_EQ(s.capacity, 1u);
  EXPECT_EQ(s.data[0], 0);

  rosidl_runtime_c__String__fini(&s);
  rosidl_runtime_c__String__fini(&t);

  // Check assign after fini without init
  EXPECT_TRUE(rosidl_runtime_c__String__assign(&s, &data[0]));
  rosidl_runtime_c__String__fini(&s);
}

TEST(string_functions, init_fini_sequence) {
  rosidl_runtime_c__String__Sequence sequence;
  EXPECT_FALSE(rosidl_runtime_c__String__Sequence__init(nullptr, 0u));
  EXPECT_TRUE(rosidl_runtime_c__String__Sequence__init(&sequence, 0u));

  EXPECT_EQ(sequence.size, 0u);
  EXPECT_EQ(sequence.capacity, 0u);
  EXPECT_EQ(sequence.data, nullptr);

  // Checking these return without issue
  rosidl_runtime_c__String__Sequence__fini(nullptr);
  rosidl_runtime_c__String__Sequence__fini(&sequence);

  constexpr size_t seq_size = 3u;
  EXPECT_TRUE(rosidl_runtime_c__String__Sequence__init(&sequence, seq_size));
  EXPECT_EQ(sequence.size, seq_size);
  EXPECT_EQ(sequence.capacity, seq_size);
  rosidl_runtime_c__String__Sequence__fini(&sequence);
}

TEST(string_functions, create_destroy_sequence) {
  rosidl_runtime_c__String__Sequence * sequence =
    rosidl_runtime_c__String__Sequence__create(0u);
  EXPECT_NE(sequence, nullptr);
  EXPECT_EQ(sequence->data, nullptr);
  EXPECT_EQ(sequence->size, 0u);
  EXPECT_EQ(sequence->capacity, 0u);

  // Shouldn't do anything exciting
  rosidl_runtime_c__String__Sequence__destroy(sequence);

  constexpr size_t seq_size = 3u;
  sequence = rosidl_runtime_c__String__Sequence__create(seq_size);
  EXPECT_NE(sequence, nullptr);
  EXPECT_EQ(sequence->size, seq_size);
  EXPECT_EQ(sequence->capacity, seq_size);
  rosidl_runtime_c__String__Sequence__destroy(sequence);
}

TEST(string_functions, create_destroy_sequence_maybe_fail) {
  rosidl_runtime_c__String__Sequence * sequence = nullptr;
  constexpr size_t seq_size = 10u;

  RCUTILS_FAULT_INJECTION_TEST(
  {
    sequence = rosidl_runtime_c__String__Sequence__create(seq_size);
    if (nullptr != sequence) {
      rosidl_runtime_c__String__Sequence__destroy(sequence);
      sequence = nullptr;
    }
  });
}
