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

#include "./separate_compilation.h"  // NOLINT

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "rosidl_generator_c/msg/nested__struct.h"
#include "rosidl_generator_c/msg/nested__functions.h"
#include "rosidl_generator_c/msg/various__struct.h"
#include "rosidl_generator_c/msg/various__functions.h"
#include "rosidl_generator_c/primitives_sequence_functions.h"
#include "rosidl_generator_c/string_functions.h"

int func()
{
  rosidl_generator_c__msg__Various * msg = rosidl_generator_c__msg__Various__create();
  if (!msg) {
    fprintf(stderr, "failed to allocate message\n");
    return 1;
  }

  if (msg->int8_value != -5) {
    fprintf(stderr, "wrong default value\n");
    return 1;
  }

  if (msg->up_to_three_int32_values.data) {
    fprintf(stderr, "wrong bounded array initialization\n");
    return 1;
  }
  if (msg->up_to_three_int32_values.capacity) {
    fprintf(stderr, "wrong bounded array initialization\n");
    return 1;
  }

  if (!msg->up_to_three_int32_values_with_default_values.data) {
    fprintf(stderr, "wrong bounded array with default values initialization\n");
    return 1;
  }
  if (msg->up_to_three_int32_values_with_default_values.capacity != 2) {
    fprintf(stderr, "wrong bounded array with default values initialization\n");
    return 1;
  }
  if (msg->up_to_three_int32_values_with_default_values.data[0] != 5) {
    fprintf(stderr, "wrong default value of bounded array\n");
    return 1;
  }
  if (msg->up_to_three_int32_values_with_default_values.data[1] != 23) {
    fprintf(stderr, "wrong default value of bounded array\n");
    return 1;
  }

  bool success = rosidl_generator_c__uint64__Sequence__init(&msg->unbounded_uint64_values, 5);
  if (!success) {
    fprintf(stderr, "failed to allocate primitive array\n");
    return 1;
  }

  success = rosidl_generator_c__msg__Nested__Sequence__init(&msg->unbounded_nested, 10);
  if (!success) {
    fprintf(stderr, "failed to allocate sub message array\n");
    return 1;
  }

  if (msg->unbounded_nested.data[9].two_primitives[1].uint8_value != 23) {
    fprintf(stderr, "wrong nested default value\n");
    return 1;
  }

  int cmp = strcmp(
    msg->unbounded_nested.data[9].two_primitives[1].string_value_with_default.data, "default");
  if (cmp != 0) {
    fprintf(stderr, "wrong nested default string value\n");
    return 1;
  }

  success = rosidl_generator_c__String__assign(
    &msg->unbounded_nested.data[9].two_primitives[0].string_value_with_default, "foo");
  if (!success) {
    fprintf(stderr, "failed to assign string\n");
    return 1;
  }

  cmp = strcmp(
    msg->unbounded_nested.data[9].two_primitives[0].string_value_with_default.data, "foo");
  if (cmp != 0) {
    fprintf(stderr, "assigned string has wrong value\n");
    return 1;
  }

  rosidl_generator_c__msg__Various__destroy(msg);

  return 0;
}
