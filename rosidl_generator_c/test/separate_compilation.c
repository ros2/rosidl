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

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "rosidl_generator_c/msg/primitives__struct.h"
#include "rosidl_generator_c/msg/primitives__functions.h"
#include "rosidl_generator_c/msg/unbounded_sequence_nested__struct.h"
#include "rosidl_generator_c/msg/unbounded_sequence_nested__functions.h"
#include "rosidl_generator_c/msg/various__struct.h"
#include "rosidl_generator_c/msg/various__functions.h"
#include "rosidl_generator_c/primitives_sequence_functions.h"
#include "rosidl_generator_c/string_functions.h"

int floateq(const double lhs, const double rhs)
{
  const double EPS = 0.001;
  return fabs(lhs - rhs) < EPS;
}

int func()
{
  rosidl_generator_c__msg__Various * msg = rosidl_generator_c__msg__Various__create();
  if (!msg) {
    fprintf(stderr, "failed to allocate message\n");
    return 1;
  }

  if (!floateq(msg->float64_value_def, 2.4)) {
    fprintf(stderr, "wrong default value\n");
    return 1;
  }

  if (msg->bounded_seq.data) {
    fprintf(stderr, "wrong bounded array initialization\n");
    return 1;
  }
  if (msg->bounded_seq.capacity) {
    fprintf(stderr, "wrong bounded array initialization\n");
    return 1;
  }

  if (!msg->bounded_seq_def.data) {
    fprintf(stderr, "wrong bounded array with default values initialization\n");
    return 1;
  }
  if (msg->bounded_seq_def.capacity != 2) {
    fprintf(stderr, "wrong bounded array with default values initialization\n");
    return 1;
  }
  if (!floateq(msg->bounded_seq_def.data[0], 3.0)) {
    fprintf(stderr, "wrong default value of bounded array\n");
    return 1;
  }
  if (!floateq(msg->bounded_seq_def.data[1], 4.0)) {
    fprintf(stderr, "wrong default value of bounded array\n");
    return 1;
  }

  bool success = rosidl_generator_c__float32__Sequence__init(&msg->unbounded_seq, 5);
  if (!success) {
    fprintf(stderr, "failed to allocate primitive array\n");
    return 1;
  }

  rosidl_generator_c__msg__UnboundedSequenceNested * msg_nested =
    rosidl_generator_c__msg__UnboundedSequenceNested__create();
  if (!msg_nested) {
    fprintf(stderr, "failed to allocate nested message\n");
    return 1;
  }

  success = rosidl_generator_c__msg__Primitives__Sequence__init(
    &msg_nested->primitives_values, 10);
  if (!success) {
    fprintf(stderr, "failed to allocate sub message array\n");
    return 1;
  }

  // TODO(jacobperron): Add test for nested values with defaults
  // int cmp = strcmp(
  //   msg->unbounded_nested.data[9].two_primitives[1].string_value_with_default.data, "default");
  // if (cmp != 0) {
  //   fprintf(stderr, "wrong nested default string value\n");
  //   return 1;
  // }

  success = rosidl_generator_c__String__assign(
    &msg_nested->primitives_values.data[9].string_value, "foo");
  if (!success) {
    fprintf(stderr, "failed to assign string\n");
    return 1;
  }

  int cmp = strcmp(
    msg_nested->primitives_values.data[9].string_value.data, "foo");
  if (cmp != 0) {
    fprintf(stderr, "assigned string has wrong value\n");
    return 1;
  }

  rosidl_generator_c__msg__Various__destroy(msg);
  rosidl_generator_c__msg__UnboundedSequenceNested__destroy(msg_nested);

  return 0;
}
