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
#include <stdio.h>
#include <stdbool.h>
#include <limits.h>
#include <float.h>
#include <stdint.h>

#include "rosidl_generator_c/msg/various.h"
#include "rosidl_generator_c/msg/bool.h"
#include "rosidl_generator_c/msg/byte.h"
#include "rosidl_generator_c/msg/char.h"
#include "rosidl_generator_c/msg/float32.h"
#include "rosidl_generator_c/msg/float64.h"
#include "rosidl_generator_c/msg/int8.h"
#include "rosidl_generator_c/msg/int16.h"
#include "rosidl_generator_c/msg/int32.h"
#include "rosidl_generator_c/msg/int64.h"
#include "rosidl_generator_c/msg/uint8.h"
#include "rosidl_generator_c/msg/uint16.h"
#include "rosidl_generator_c/msg/uint32.h"
#include "rosidl_generator_c/msg/uint64.h"
#include "rosidl_generator_c/msg/primitive_values.h"


void test_primitives(void);
void test_primitives_default_value(void);

int main(void)
{
  fprintf(stderr, "Testing rosidl_generator_c message types...\n");
  fprintf(stderr, "Testing simple primitive message types...\n");
  test_primitives();
  fprintf(stderr, "Testing simple primitives with default values...\n");
  test_primitives_default_value();
  fprintf(stderr, "All tests were good!\n");
  return 0;
}

/**
 * Test message with simple primitive types.
 */
void test_primitives(void)
{
  rosidl_generator_c__msg__Bool bool_msg;
  rosidl_generator_c__msg__Byte byte_msg;
  rosidl_generator_c__msg__Char char_msg;
  rosidl_generator_c__msg__Float32 float32_msg;
  rosidl_generator_c__msg__Float64 float64_msg;
  rosidl_generator_c__msg__Int8 int8_msg;
  rosidl_generator_c__msg__Int16 int16_msg;
  rosidl_generator_c__msg__Int32 int32_msg;
  rosidl_generator_c__msg__Int64 int64_msg;
  rosidl_generator_c__msg__Uint8 uint8_msg;
  rosidl_generator_c__msg__Uint16 uint16_msg;
  rosidl_generator_c__msg__Uint32 uint32_msg;
  rosidl_generator_c__msg__Uint64 uint64_msg;

  bool_msg.empty_bool = true;
  assert(bool_msg.empty_bool == true);

  bool_msg.empty_bool = false;
  assert(bool_msg.empty_bool == false);

  byte_msg.empty_byte = 0;
  assert(byte_msg.empty_byte == 0);

  byte_msg.empty_byte = 255;
  assert(byte_msg.empty_byte == 255);

  char_msg.empty_char = CHAR_MIN;
  assert(char_msg.empty_char == CHAR_MIN);

  char_msg.empty_char = CHAR_MAX;
  assert(char_msg.empty_char == CHAR_MAX);

  float32_msg.empty_float32 = FLT_MIN;
  assert(float32_msg.empty_float32 == FLT_MIN);

  float32_msg.empty_float32 = FLT_MAX;
  assert(float32_msg.empty_float32 == FLT_MAX);

  float64_msg.empty_float64 = DBL_MIN;
  assert(float64_msg.empty_float64 == DBL_MIN);

  float64_msg.empty_float64 = DBL_MAX;
  assert(float64_msg.empty_float64 == DBL_MAX);

  int8_msg.empty_int8 = INT8_MIN;
  assert(int8_msg.empty_int8 == INT8_MIN);

  int8_msg.empty_int8 = INT8_MAX;
  assert(int8_msg.empty_int8 == INT8_MAX);

  int16_msg.empty_int16 = INT16_MIN;
  assert(int16_msg.empty_int16 == INT16_MIN);

  int16_msg.empty_int16 = INT16_MAX;
  assert(int16_msg.empty_int16 == INT16_MAX);

  int32_msg.empty_int32 = INT32_MIN;
  assert(int32_msg.empty_int32 == INT32_MIN);

  int32_msg.empty_int32 = INT32_MAX;
  assert(int32_msg.empty_int32 == INT32_MAX);

  int64_msg.empty_int64 = INT64_MIN;
  assert(int64_msg.empty_int64 == INT64_MIN);

  int64_msg.empty_int64 = INT64_MAX;
  assert(int64_msg.empty_int64 == INT64_MAX);

  uint8_msg.empty_uint8 = 0;
  assert(uint8_msg.empty_uint8 == 0);

  uint8_msg.empty_uint8 = UINT8_MAX;
  assert(uint8_msg.empty_uint8 == UINT8_MAX);

  uint16_msg.empty_uint16 = 0;
  assert(uint16_msg.empty_uint16 == 0);

  uint16_msg.empty_uint16 = UINT16_MAX;
  assert(uint16_msg.empty_uint16 == UINT16_MAX);

  uint32_msg.empty_uint32 = 0;
  assert(uint32_msg.empty_uint32 == 0);

  uint32_msg.empty_uint32 = UINT32_MAX;
  assert(uint32_msg.empty_uint32 == UINT32_MAX);

  uint64_msg.empty_uint64 = 0;
  assert(uint64_msg.empty_uint64 == 0);

  uint64_msg.empty_uint64 = UINT64_MAX;
  assert(uint64_msg.empty_uint64 == UINT64_MAX);
}

/**
 * Test message with simple primitive types using a default value initializer.
 */
void test_primitives_default_value(void)
{
  rosidl_generator_c__msg__PrimitiveValues * primitive_values = NULL;

  primitive_values = rosidl_generator_c__msg__PrimitiveValues__create();

  assert(primitive_values != NULL);

  assert(true == primitive_values->def_bool_1);
  assert(false == primitive_values->def_bool_2);
  assert(66 == primitive_values->def_byte);
  assert(-66 == primitive_values->def_char);
  assert(3.1416f == primitive_values->def_float32);
  assert(3.14159265 == primitive_values->def_float64);
  assert(3 == primitive_values->def_int8);
  assert(6 == primitive_values->def_int16);
  assert(10 == primitive_values->def_int32);
  assert(15 == primitive_values->def_int64);
  assert(33 == primitive_values->def_uint8);
  assert(36 == primitive_values->def_uint16);
  assert(310 == primitive_values->def_uint32);
  assert(315 == primitive_values->def_uint64);

  rosidl_generator_c__msg__PrimitiveValues__destroy(primitive_values);
}
