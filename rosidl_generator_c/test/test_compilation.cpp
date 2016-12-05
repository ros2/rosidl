// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#include <assert.h>
#include <gtest/gtest.h>

#include <stdio.h>
#include <string.h>

#include "rosidl_generator_c/msg/nested__struct.h"
#include "rosidl_generator_c/msg/nested__functions.h"
#include "rosidl_generator_c/msg/various__struct.h"
#include "rosidl_generator_c/msg/various__functions.h"
#include "rosidl_generator_c/primitives_array_functions.h"
#include "rosidl_generator_c/string_functions.h"


TEST(Test_c_compilation, test_constants) {
  ASSERT_TRUE(rosidl_generator_c__msg__Various__FOO);
  ASSERT_EQ(rosidl_generator_c__msg__Various__BAZ, 42);

  rosidl_generator_c__msg__Various * msg = rosidl_generator_c__msg__Various__create();
  ASSERT_TRUE(NULL != msg);
  ASSERT_EQ(msg->int8_value, -5);

  rosidl_generator_c__msg__Various__destroy(msg);
}

TEST(Test_c_compilation, test_array_init) {
  rosidl_generator_c__msg__Various * msg = rosidl_generator_c__msg__Various__create();
  ASSERT_EQ(0, msg->up_to_three_int32_values.data);
  ASSERT_EQ(0, msg->up_to_three_int32_values.capacity);

  ASSERT_TRUE(NULL != msg->up_to_three_int32_values_with_default_values.data);
  ASSERT_EQ(2, msg->up_to_three_int32_values_with_default_values.capacity);

  ASSERT_EQ(5, msg->up_to_three_int32_values_with_default_values.data[0]);
  ASSERT_EQ(23, msg->up_to_three_int32_values_with_default_values.data[1]);

  ASSERT_TRUE(rosidl_generator_c__uint64__Array__init(&msg->unbounded_uint64_values, 5));
  ASSERT_TRUE(rosidl_generator_c__msg__Nested__Array__init(&msg->unbounded_nested, 10));

  ASSERT_EQ(23, msg->unbounded_nested.data[9].two_primitives[1].uint8_value);

  ASSERT_STREQ(
    "default",
    msg->unbounded_nested.data[9].two_primitives[1].string_value_with_default.data);

  ASSERT_TRUE(
    rosidl_generator_c__String__assign(
      &msg->unbounded_nested.data[9].two_primitives[0].string_value_with_default, "foo"));
  ASSERT_STREQ(
    "foo", msg->unbounded_nested.data[9].two_primitives[0].string_value_with_default.data);

  rosidl_generator_c__msg__Various__destroy(msg);
}
