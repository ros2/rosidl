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
#include <string.h>

#include "rosidl_generator_c/primitives_sequence_functions.h"
#include "rosidl_generator_c/string_functions.h"

#include "rosidl_generator_c/msg/array_nested.h"
#include "rosidl_generator_c/msg/array_primitives.h"
#include "rosidl_generator_c/msg/array_primitives_nested.h"
#include "rosidl_generator_c/msg/bounded_sequence_nested.h"
#include "rosidl_generator_c/msg/bounded_sequence_primitives.h"
#include "rosidl_generator_c/msg/bounded_sequence_primitives_nested.h"
#include "rosidl_generator_c/msg/bounded_string.h"
#include "rosidl_generator_c/msg/empty.h"
#include "rosidl_generator_c/msg/nested.h"
#include "rosidl_generator_c/msg/primitives.h"
#include "rosidl_generator_c/msg/primitives_constants.h"
#include "rosidl_generator_c/msg/primitives_default.h"
#include "rosidl_generator_c/msg/unbounded_sequence_nested.h"
#include "rosidl_generator_c/msg/unbounded_sequence_primitives.h"
#include "rosidl_generator_c/msg/unbounded_sequence_primitives_nested.h"

#define TEST_STRING "foo"
#define TEST_STRING2 \
  "The quick brown fox jumps over the lazy dog."
#define TEST_STRING3 \
  "PACK MY BOX WITH FIVE DOZEN LIQUOR JUGS."
#define NESTED_SIZE 7

#define STRINGIFY(x) _STRINGIFY(x)
#define _STRINGIFY(x) #x

#define EXPECT_EQ(arg1, arg2) if ((arg1) != (arg2)) { \
    fputs(STRINGIFY(arg1) " != " STRINGIFY(arg2) "\n", stderr); \
    return 1; \
}
#define EXPECT_NE(arg1, arg2) if ((arg1) == (arg2)) return 1

int test_primitives(void);
int test_primitives_default_value(void);
int test_bounded_string(void);
int test_unbounded_sequence_primitives(void);
int test_bounded_sequence_primitives(void);
int test_array_primitives(void);
int test_bounded_sequence_nested(void);
int test_unbounded_sequence_nested(void);
int test_unbounded_sequence_primitives_nested(void);
int test_array_nested(void);

int main(void)
{
  int rc = 0;
  printf("Testing rosidl_generator_c message types...\n");
  printf("Testing simple primitive message types...\n");
  if (test_primitives()) {
    fprintf(stderr, "test_primitives() FAILED\n");
    rc++;
  }
  printf("Testing simple primitives with default values...\n");
  if (test_primitives_default_value()) {
    fprintf(stderr, "test_primitives_default_value() FAILED\n");
    rc++;
  }
  printf("Testing bounded string types...\n");
  if (test_bounded_string()) {
    fprintf(stderr, "test_bounded_string() FAILED\n");
    rc++;
  }
  printf("Testing primitives unbounded sequences types...\n");
  if (test_unbounded_sequence_primitives()) {
    fprintf(stderr, "test_unbounded_sequence_primitives() FAILED\n");
    rc++;
  }
  printf("Testing primitives bounded sequences types...\n");
  if (test_bounded_sequence_primitives()) {
    fprintf(stderr, "test_bounded_sequence_primitives() FAILED\n");
    rc++;
  }
  printf("Testing primitives arrays types...\n");
  if (test_array_primitives()) {
    fprintf(stderr, "test_array_primitives() FAILED\n");
    rc++;
  }
  printf("Testing array_nested messages...\n");
  if (test_array_nested()) {
    fprintf(stderr, "test_array_nested() FAILED\n");
    rc++;
  }
  printf("Testing bounded_sequence_nested messages...\n");
  if (test_bounded_sequence_nested()) {
    fprintf(stderr, "test_bounded_sequence_nested() FAILED\n");
    rc++;
  }
  printf("Testing unbounded_sequence_nested messages...\n");
  if (test_unbounded_sequence_nested()) {
    fprintf(stderr, "test_unbounded_sequence_nested() FAILED\n");
    rc++;
  }
  printf("Testing unbounded_sequence_primitives_nested messages...\n");
  if (test_unbounded_sequence_primitives_nested()) {
    fprintf(stderr, "test_unbounded_sequence_primitives_nested() FAILED\n");
  }
  if (rc != 0) {
    fprintf(stderr, "Some tests failed!\n");
  } else {
    printf("All tests were good!\n");
  }
  return rc != 0;
}

/**
 * Test message with simple primitive types.
 */
int test_primitives(void)
{
  rosidl_generator_c__msg__Primitives msg;

  msg.bool_value = true;
  EXPECT_EQ(true, msg.bool_value);

  msg.bool_value = false;
  EXPECT_EQ(false, msg.bool_value);

  msg.byte_value = 0;
  EXPECT_EQ(0, msg.byte_value);

  msg.byte_value = 255;
  EXPECT_EQ(255, msg.byte_value);

  EXPECT_EQ(200u, rosidl_generator_c__msg__PrimitivesConstants__UINT8_CONST);
  EXPECT_EQ(-1000, rosidl_generator_c__msg__PrimitivesConstants__INT16_CONST);
  EXPECT_EQ(0, strncmp(rosidl_generator_c__msg__PrimitivesConstants__STRING_CONST, "foo", 3));
  EXPECT_EQ(100, rosidl_generator_c__msg__PrimitivesConstants__CHAR_CONST);
  EXPECT_EQ(50, rosidl_generator_c__msg__PrimitivesConstants__BYTE_CONST);

  msg.char_value = 0;
  EXPECT_EQ(0, msg.char_value);

  msg.char_value = UINT8_MAX;
  EXPECT_EQ(UINT8_MAX, msg.char_value);

  msg.float32_value = FLT_MIN;
  EXPECT_EQ(FLT_MIN, msg.float32_value);

  msg.float32_value = FLT_MAX;
  EXPECT_EQ(FLT_MAX, msg.float32_value);

  msg.float64_value = DBL_MIN;
  EXPECT_EQ(DBL_MIN, msg.float64_value);

  msg.float64_value = DBL_MAX;
  EXPECT_EQ(DBL_MAX, msg.float64_value);

  msg.int8_value = INT8_MIN;
  EXPECT_EQ(INT8_MIN, msg.int8_value);

  msg.int8_value = INT8_MAX;
  EXPECT_EQ(INT8_MAX, msg.int8_value);

  msg.int16_value = INT16_MIN;
  EXPECT_EQ(INT16_MIN, msg.int16_value);

  msg.int16_value = INT16_MAX;
  EXPECT_EQ(INT16_MAX, msg.int16_value);

  msg.int32_value = INT32_MIN;
  EXPECT_EQ(INT32_MIN, msg.int32_value);

  msg.int32_value = INT32_MAX;
  EXPECT_EQ(INT32_MAX, msg.int32_value);

  msg.int64_value = INT64_MIN;
  EXPECT_EQ(INT64_MIN, msg.int64_value);

  msg.int64_value = INT64_MAX;
  EXPECT_EQ(INT64_MAX, msg.int64_value);

  msg.uint8_value = 0;
  EXPECT_EQ(0, msg.uint8_value);

  msg.uint8_value = UINT8_MAX;
  EXPECT_EQ(UINT8_MAX, msg.uint8_value);

  msg.uint16_value = 0;
  EXPECT_EQ(0, msg.uint16_value);

  msg.uint16_value = UINT16_MAX;
  EXPECT_EQ(UINT16_MAX, msg.uint16_value);

  msg.uint32_value = 0;
  EXPECT_EQ(0, msg.uint32_value);

  msg.uint32_value = UINT32_MAX;
  EXPECT_EQ(UINT32_MAX, msg.uint32_value);

  msg.uint64_value = 0;
  EXPECT_EQ(0, msg.uint64_value);

  msg.uint64_value = UINT64_MAX;
  EXPECT_EQ(UINT64_MAX, msg.uint64_value);

  return 0;
}

/**
 * Test message with simple primitive types using a default value initializer.
 */
int test_primitives_default_value(void)
{
  rosidl_generator_c__msg__PrimitivesDefault * primitive_default = NULL;

  primitive_default = rosidl_generator_c__msg__PrimitivesDefault__create();

  EXPECT_NE(primitive_default, NULL);

  EXPECT_EQ(true, primitive_default->bool_value);
  EXPECT_EQ(50, primitive_default->byte_value);
  EXPECT_EQ(100, primitive_default->char_value);
  EXPECT_EQ(1.125f, primitive_default->float32_value);
  EXPECT_EQ(1.125, primitive_default->float64_value);
  EXPECT_EQ(-50, primitive_default->int8_value);
  EXPECT_EQ(200, primitive_default->uint8_value);
  EXPECT_EQ(-1000, primitive_default->int16_value);
  EXPECT_EQ(2000, primitive_default->uint16_value);
  EXPECT_EQ(-30000, primitive_default->int32_value);
  EXPECT_EQ(60000, primitive_default->uint32_value);
  EXPECT_EQ(-40000000, primitive_default->int64_value);
  EXPECT_EQ(50000000, primitive_default->uint64_value);
  EXPECT_EQ(0, strcmp("bar", primitive_default->string_value.data));

  rosidl_generator_c__msg__PrimitivesDefault__destroy(primitive_default);

  return 0;
}

/**
 * Test message with bounded strings.
 */
int test_bounded_string(void)
{
  bool res = false;
  rosidl_generator_c__msg__BoundedString * strings = NULL;

  strings = rosidl_generator_c__msg__BoundedString__create();
  EXPECT_NE(strings, NULL);

  res = rosidl_generator_c__String__assign(&strings->bounded_string_value, TEST_STRING);
  EXPECT_EQ(true, res);
  EXPECT_EQ(0, strcmp(strings->bounded_string_value.data, TEST_STRING));
  EXPECT_EQ(0, strcmp(strings->bounded_string_with_default_value.data, "Hello world!"));
  // since upper-bound checking is not implemented yet, we restrict the string copying
  // TODO(mikaelarguedas) Test string length properly instead of cheating copy
  // res = rosidl_generator_c__String__assign(&strings->bounded_string_value, TEST_STRING2);
  res = rosidl_generator_c__String__assignn(&strings->bounded_string_value, TEST_STRING2, 3);
  EXPECT_EQ(true, res);
  EXPECT_EQ(0, strcmp(strings->bounded_string_value.data, "The"));

  rosidl_generator_c__msg__BoundedString__destroy(strings);

  return 0;
}

/**
 * Test message with different string array types.
 */
/*int test_string_arrays(void)
{
  bool res = false;
  rosidl_generator_c__msg__StringArrays * strings = rosidl_generator_c__msg__StringArrays__create();

  res = rosidl_generator_c__msg__StringArrays__init(strings);
  EXPECT_EQ(res, true);

  rosidl_generator_c__String__Sequence__init(&strings->string_dynamic_array_value, 3);
  res = rosidl_generator_c__String__assign(
    &strings->string_dynamic_array_value.data[0], TEST_STRING);
  EXPECT_EQ(true, res);
  EXPECT_EQ(0, strcmp(strings->string_dynamic_array_value.data[0].data, TEST_STRING));

  res = rosidl_generator_c__String__assign(
    &strings->string_dynamic_array_value.data[1], TEST_STRING2);
  EXPECT_EQ(true, res);
  EXPECT_EQ(0, strcmp(strings->string_dynamic_array_value.data[1].data, TEST_STRING2));

  res = rosidl_generator_c__String__assign(
    &strings->string_dynamic_array_value.data[2], TEST_STRING3);
  EXPECT_EQ(true, res);
  EXPECT_EQ(0, strcmp(strings->string_dynamic_array_value.data[2].data, TEST_STRING3));

  res = rosidl_generator_c__String__assign(
    &strings->string_static_array_value[0], TEST_STRING);
  EXPECT_EQ(true, res);
  EXPECT_EQ(0, strcmp(strings->string_static_array_value[0].data, TEST_STRING));

  res = rosidl_generator_c__String__assign(
    &strings->string_static_array_value[1], TEST_STRING2);
  EXPECT_EQ(true, res);
  EXPECT_EQ(0, strcmp(strings->string_static_array_value[1].data, TEST_STRING2));

  res = rosidl_generator_c__String__assign(
    &strings->string_static_array_value[2], TEST_STRING3);
  EXPECT_EQ(true, res);
  EXPECT_EQ(0, strcmp(strings->string_static_array_value[2].data, TEST_STRING3));

  rosidl_generator_c__String__Sequence__init(&strings->string_bounded_array_value, 4);
  res = rosidl_generator_c__String__assign(
    &strings->string_bounded_array_value.data[0], TEST_STRING);
  EXPECT_EQ(true, res);
  EXPECT_EQ(0, strcmp(strings->string_bounded_array_value.data[0].data, TEST_STRING));

  res = rosidl_generator_c__String__assign(
    &strings->string_bounded_array_value.data[1], TEST_STRING2);
  EXPECT_EQ(true, res);
  EXPECT_EQ(0, strcmp(strings->string_bounded_array_value.data[1].data, TEST_STRING2));

  res = rosidl_generator_c__String__assign(
    &strings->string_bounded_array_value.data[2], TEST_STRING3);
  EXPECT_EQ(true, res);
  EXPECT_EQ(0, strcmp(strings->string_bounded_array_value.data[2].data, TEST_STRING3));

  res = rosidl_generator_c__String__assign(
    &strings->string_bounded_array_value.data[3], "Hello World!");
  EXPECT_EQ(true, res);
  EXPECT_EQ(0, strcmp(strings->string_bounded_array_value.data[3].data, "Hello World!"));

  rosidl_generator_c__msg__StringArrays__destroy(strings);

  return 0;
}*/

/**
 * Test different string array types using a default value initializer.
 */
/*int test_string_arrays_default_value(void)
{
  rosidl_generator_c__msg__StringArrays * string_arrays = NULL;

  string_arrays = rosidl_generator_c__msg__StringArrays__create();

  EXPECT_NE(string_arrays, NULL);

  bool res = rosidl_generator_c__msg__StringArrays__init(string_arrays);
  EXPECT_EQ(true, res);

  EXPECT_EQ(0, strcmp(string_arrays->def_string_static_array_value[0].data, "Hello"));
  EXPECT_EQ(0, strcmp(string_arrays->def_string_static_array_value[1].data, "World"));
  EXPECT_EQ(0, strcmp(string_arrays->def_string_static_array_value[2].data, "!"));

  EXPECT_EQ(0, strcmp(string_arrays->def_string_dynamic_array_value.data[0].data, "What"));
  EXPECT_EQ(0, strcmp(string_arrays->def_string_dynamic_array_value.data[1].data, "a"));
  EXPECT_EQ(0, strcmp(string_arrays->def_string_dynamic_array_value.data[2].data, "wonderful"));
  EXPECT_EQ(0, strcmp(string_arrays->def_string_dynamic_array_value.data[3].data, "world"));
  EXPECT_EQ(0, strcmp(string_arrays->def_string_dynamic_array_value.data[4].data, "!"));
  EXPECT_EQ(5, string_arrays->def_string_dynamic_array_value.size);

  EXPECT_EQ(0, strcmp(string_arrays->def_string_bounded_array_value.data[0].data, "Hello"));
  EXPECT_EQ(0, strcmp(string_arrays->def_string_bounded_array_value.data[1].data, "World"));
  EXPECT_EQ(0, strcmp(string_arrays->def_string_bounded_array_value.data[2].data, "!"));
  EXPECT_EQ(3, string_arrays->def_string_bounded_array_value.size);

  EXPECT_EQ(0, strcmp(string_arrays->def_various_quotes.data[0].data, "H\"el'lo"));
  EXPECT_EQ(0, strcmp(string_arrays->def_various_quotes.data[1].data, "Wo'r\"ld"));
  EXPECT_EQ(2, string_arrays->def_various_quotes.size);

  EXPECT_EQ(0, strcmp(string_arrays->def_various_commas.data[0].data, "Hel,lo"));
  EXPECT_EQ(0, strcmp(string_arrays->def_various_commas.data[1].data, ",World"));
  EXPECT_EQ(0, strcmp(string_arrays->def_various_commas.data[2].data, "abcd"));
  EXPECT_EQ(0, strcmp(string_arrays->def_various_commas.data[3].data, "!,"));
  EXPECT_EQ(4, string_arrays->def_various_commas.size);

  rosidl_generator_c__msg__StringArrays__destroy(string_arrays);

  return 0;
}*/


/**
 * Test unbounded sequences
 */
int test_unbounded_sequence_primitives(void)
{
  bool res = false;
  int i;
  rosidl_generator_c__msg__UnboundedSequencePrimitives * sequences = NULL;

  sequences = rosidl_generator_c__msg__UnboundedSequencePrimitives__create();
  EXPECT_NE(NULL, sequences);

  // bool_values
  res = rosidl_generator_c__boolean__Sequence__init(&sequences->bool_values, NESTED_SIZE);
  EXPECT_EQ(true, res);
  // load values
  for (i = 0; i < NESTED_SIZE; i++) {
    if (0 == (i % 2)) {
      sequences->bool_values.data[i] = true;
    } else {
      sequences->bool_values.data[i] = false;
    }
  }
  // test values
  for (i = 0; i < NESTED_SIZE; i++) {
    if (0 == (i % 2)) {
      EXPECT_EQ(true, sequences->bool_values.data[i]);
    } else {
      EXPECT_EQ(false, sequences->bool_values.data[i]);
    }
  }

  // byte_values
  res = rosidl_generator_c__octet__Sequence__init(&sequences->byte_values, NESTED_SIZE);
  EXPECT_EQ(true, res);
  uint8_t test_values_byte[7] = {0, 57, 110, 177, 201, 240, 255};
  for (i = 0; i < NESTED_SIZE; i++) {
    sequences->byte_values.data[i] = test_values_byte[i];
  }
  // test values
  for (i = 0; i < NESTED_SIZE; i++) {
    EXPECT_EQ(test_values_byte[i], sequences->byte_values.data[i]);
  }


  // char values
  res = rosidl_generator_c__uint8__Sequence__init(&sequences->char_values, NESTED_SIZE);
  EXPECT_EQ(true, res);
  char test_values_char[7] = {'a', '5', '#', 'Z', '@', '-', ' '};
  for (i = 0; i < NESTED_SIZE; i++) {
    sequences->char_values.data[i] = test_values_char[i];
  }
  // test values
  for (i = 0; i < NESTED_SIZE; i++) {
    EXPECT_EQ(test_values_char[i], sequences->char_values.data[i]);
  }

  // float32 values
  res = rosidl_generator_c__float__Sequence__init(&sequences->float32_values, NESTED_SIZE);
  EXPECT_EQ(true, res);
  float test_values_float32[7] = {
    -3.000001f, 22143.541325f, 6331.00432f, -214.66241f, 0.000001f, 1415555.12345f, -1.11154f
  };
  for (i = 0; i < NESTED_SIZE; i++) {
    sequences->float32_values.data[i] = test_values_float32[i];
  }
  // test values
  for (i = 0; i < NESTED_SIZE; i++) {
    EXPECT_EQ(test_values_float32[i], sequences->float32_values.data[i]);
  }

  // float64 values
  res = rosidl_generator_c__double__Sequence__init(&sequences->float64_values, NESTED_SIZE);
  EXPECT_EQ(true, res);
  double test_values_float64[7] = {
    -120310.00843902140001, 22143.54483920141325, 6331.0048392104432, -214.62850432596241,
    0.0000000000001, 1415555.128294031432345, -1.111184329208454
  };
  for (i = 0; i < NESTED_SIZE; i++) {
    sequences->float64_values.data[i] = test_values_float64[i];
  }
  // test values
  for (i = 0; i < NESTED_SIZE; i++) {
    EXPECT_EQ(test_values_float64[i], sequences->float64_values.data[i]);
  }

  // int8 values
  res = rosidl_generator_c__int8__Sequence__init(&sequences->int8_values, NESTED_SIZE);
  EXPECT_EQ(true, res);
  int8_t test_values_int8[7] = {-127, -55, -30, 0, 58, 100, 127};
  for (i = 0; i < NESTED_SIZE; i++) {
    sequences->int8_values.data[i] = test_values_int8[i];
  }
  // test values
  for (i = 0; i < NESTED_SIZE; i++) {
    EXPECT_EQ(test_values_int8[i], sequences->int8_values.data[i]);
  }

  // int16 values
  res = rosidl_generator_c__int16__Sequence__init(&sequences->int16_values, NESTED_SIZE);
  EXPECT_EQ(true, res);
  int16_t test_values_int16[7] =
  {-32767, -22222, -11111, 0, 11111, 22222, 32767};
  for (i = 0; i < NESTED_SIZE; i++) {
    sequences->int16_values.data[i] = test_values_int16[i];
  }
  // test values
  for (i = 0; i < NESTED_SIZE; i++) {
    EXPECT_EQ(test_values_int16[i], sequences->int16_values.data[i]);
  }

  // int32 values
  res = rosidl_generator_c__int32__Sequence__init(&sequences->int32_values, NESTED_SIZE);
  EXPECT_EQ(true, res);
  int32_t test_values_int32[7] =
  {INT32_MIN, INT32_MIN / 2, INT32_MIN / 4, 0L, INT32_MAX / 4, INT32_MAX / 2, INT32_MAX};
  for (i = 0; i < NESTED_SIZE; i++) {
    sequences->int32_values.data[i] = test_values_int32[i];
  }
  // test values
  for (i = 0; i < NESTED_SIZE; i++) {
    EXPECT_EQ(test_values_int32[i], sequences->int32_values.data[i]);
  }

  // int64 values
  res = rosidl_generator_c__int64__Sequence__init(&sequences->int64_values, NESTED_SIZE);
  EXPECT_EQ(true, res);
  int64_t test_values_int64[7] = {
    -9223372036854775807LL, -5000000000000000000LL, -1111111111111111111LL, 0,
    1111111111111111111LL, 5000000000000000000LL, 9223372036854775807LL
  };
  for (i = 0; i < NESTED_SIZE; i++) {
    sequences->int64_values.data[i] = test_values_int64[i];
  }
  // test values
  for (i = 0; i < NESTED_SIZE; i++) {
    EXPECT_EQ(test_values_int64[i], sequences->int64_values.data[i]);
  }

  // uint8 values
  res = rosidl_generator_c__uint8__Sequence__init(&sequences->uint8_values, NESTED_SIZE);
  EXPECT_EQ(true, res);
  uint8_t test_values_uint8[7] = {0, 5, 70, 128, 180, 220, 255};
  for (i = 0; i < NESTED_SIZE; i++) {
    sequences->uint8_values.data[i] = test_values_uint8[i];
  }
  // test values
  for (i = 0; i < NESTED_SIZE; i++) {
    EXPECT_EQ(test_values_uint8[i], sequences->uint8_values.data[i]);
  }

  // uint16 values
  res = rosidl_generator_c__uint16__Sequence__init(&sequences->uint16_values, NESTED_SIZE);
  EXPECT_EQ(true, res);
  uint16_t test_values_uint16[7] = {0U, 11111U, 22222U, 33333U, 44444U, 55555U, 65535U};
  for (i = 0; i < NESTED_SIZE; i++) {
    sequences->uint16_values.data[i] = test_values_uint16[i];
  }
  // test values
  for (i = 0; i < NESTED_SIZE; i++) {
    EXPECT_EQ(test_values_uint16[i], sequences->uint16_values.data[i]);
  }

  // uint32 values
  res = rosidl_generator_c__uint32__Sequence__init(&sequences->uint32_values, NESTED_SIZE);
  EXPECT_EQ(true, res);
  uint32_t test_values_uint32[7] = {
    0UL, 100UL, 2000UL, 30000UL, 444444UL, 567890123UL, 4294967295UL
  };
  for (i = 0; i < NESTED_SIZE; i++) {
    sequences->uint32_values.data[i] = test_values_uint32[i];
  }
  // test values
  for (i = 0; i < NESTED_SIZE; i++) {
    EXPECT_EQ(test_values_uint32[i], sequences->uint32_values.data[i]);
  }

  // uint64 values
  res = rosidl_generator_c__uint64__Sequence__init(&sequences->uint64_values, NESTED_SIZE);
  EXPECT_EQ(true, res);
  uint64_t test_values_uint64[7] = {
    0ULL, 10000ULL, 30000000ULL, 444444444ULL, 567890123456789ULL,
    429496729578901234ULL, 18446744073709551615ULL
  };
  for (i = 0; i < NESTED_SIZE; i++) {
    sequences->uint64_values.data[i] = test_values_uint64[i];
  }
  // test values
  for (i = 0; i < NESTED_SIZE; i++) {
    EXPECT_EQ(test_values_uint64[i], sequences->uint64_values.data[i]);
  }

  rosidl_generator_c__msg__UnboundedSequencePrimitives__destroy(sequences);

  return 0;
}

/**
 * Test bounded sequences
 */
int test_bounded_sequence_primitives(void)
{
  bool res = false;
  int i;
  rosidl_generator_c__msg__BoundedSequencePrimitives * sequences = NULL;

  sequences = rosidl_generator_c__msg__BoundedSequencePrimitives__create();
  EXPECT_NE(NULL, sequences);

  // bool_values
  res = rosidl_generator_c__boolean__Sequence__init(&sequences->bool_values, NESTED_SIZE);
  EXPECT_EQ(true, res);
  // load values
  for (i = 0; i < NESTED_SIZE; i++) {
    if (0 == (i % 2)) {
      sequences->bool_values.data[i] = true;
    } else {
      sequences->bool_values.data[i] = false;
    }
  }
  // test values
  for (i = 0; i < NESTED_SIZE; i++) {
    if (0 == (i % 2)) {
      EXPECT_EQ(true, sequences->bool_values.data[i]);
    } else {
      EXPECT_EQ(false, sequences->bool_values.data[i]);
    }
  }

  // byte_values
  res = rosidl_generator_c__octet__Sequence__init(&sequences->byte_values, NESTED_SIZE);
  EXPECT_EQ(true, res);
  uint8_t test_values_byte[8] = {0, 57, 110, 177, 201, 240, 255, 111};
  for (i = 0; i < NESTED_SIZE; i++) {
    sequences->byte_values.data[i] = test_values_byte[i];
  }
  // test values
  for (i = 0; i < NESTED_SIZE; i++) {
    EXPECT_EQ(test_values_byte[i], sequences->byte_values.data[i]);
  }


  // char values
  res = rosidl_generator_c__uint8__Sequence__init(&sequences->char_values, NESTED_SIZE);
  EXPECT_EQ(true, res);
  char test_values_char[7] = {'a', '5', '#', 'Z', '@', '-', ' '};
  for (i = 0; i < NESTED_SIZE; i++) {
    sequences->char_values.data[i] = test_values_char[i];
  }

  // test values
  for (i = 0; i < NESTED_SIZE; i++) {
    EXPECT_EQ(test_values_char[i], sequences->char_values.data[i]);
  }

  // float32 values
  res = rosidl_generator_c__float__Sequence__init(&sequences->float32_values, NESTED_SIZE);
  EXPECT_EQ(true, res);
  float test_values_float32[7] = {
    -3.000001f, 22143.541325f, 6331.00432f, -214.66241f, 0.000001f, 1415555.12345f, -1.11154f
  };
  for (i = 0; i < NESTED_SIZE; i++) {
    sequences->float32_values.data[i] = test_values_float32[i];
  }
  // test values
  for (i = 0; i < NESTED_SIZE; i++) {
    EXPECT_EQ(test_values_float32[i], sequences->float32_values.data[i]);
  }

  // float64 values
  res = rosidl_generator_c__double__Sequence__init(&sequences->float64_values, NESTED_SIZE);
  EXPECT_EQ(true, res);
  double test_values_float64[7] = {
    -120310.00843902140001, 22143.54483920141325, 6331.0048392104432, -214.62850432596241,
    0.0000000000001, 1415555.128294031432345, -1.111184329208454
  };
  for (i = 0; i < NESTED_SIZE; i++) {
    sequences->float64_values.data[i] = test_values_float64[i];
  }
  // test values
  for (i = 0; i < NESTED_SIZE; i++) {
    EXPECT_EQ(test_values_float64[i], sequences->float64_values.data[i]);
  }

  // int8 values
  res = rosidl_generator_c__int8__Sequence__init(&sequences->int8_values, NESTED_SIZE);
  EXPECT_EQ(true, res);
  int8_t test_values_int8[7] = {-127, -55, -30, 0, 58, 100, 127};
  for (i = 0; i < NESTED_SIZE; i++) {
    sequences->int8_values.data[i] = test_values_int8[i];
  }
  // test values
  for (i = 0; i < NESTED_SIZE; i++) {
    EXPECT_EQ(test_values_int8[i], sequences->int8_values.data[i]);
  }

  // int16 values
  res = rosidl_generator_c__int16__Sequence__init(&sequences->int16_values, NESTED_SIZE);
  EXPECT_EQ(true, res);
  int16_t test_values_int16[7] =
  {-32767, -22222, -11111, 0, 11111, 22222, 32767};
  for (i = 0; i < NESTED_SIZE; i++) {
    sequences->int16_values.data[i] = test_values_int16[i];
  }
  // test values
  for (i = 0; i < NESTED_SIZE; i++) {
    EXPECT_EQ(test_values_int16[i], sequences->int16_values.data[i]);
  }

  // int32 values
  res = rosidl_generator_c__int32__Sequence__init(&sequences->int32_values, NESTED_SIZE);
  EXPECT_EQ(true, res);
  int32_t test_values_int32[7] =
  {INT32_MIN, INT32_MIN / 2, INT32_MIN / 4, 0L, INT32_MAX / 4, INT32_MAX / 2, INT32_MAX};
  for (i = 0; i < NESTED_SIZE; i++) {
    sequences->int32_values.data[i] = test_values_int32[i];
  }
  // test values
  for (i = 0; i < NESTED_SIZE; i++) {
    EXPECT_EQ(test_values_int32[i], sequences->int32_values.data[i]);
  }

  // int64 values
  res = rosidl_generator_c__int64__Sequence__init(&sequences->int64_values, NESTED_SIZE);
  EXPECT_EQ(true, res);
  int64_t test_values_int64[7] = {
    -9223372036854775807LL, -5000000000000000000LL, -1111111111111111111, 0,
    1111111111111111111LL, 5000000000000000000LL, 9223372036854775807LL
  };
  for (i = 0; i < NESTED_SIZE; i++) {
    sequences->int64_values.data[i] = test_values_int64[i];
  }
  // test values
  for (i = 0; i < NESTED_SIZE; i++) {
    EXPECT_EQ(test_values_int64[i], sequences->int64_values.data[i]);
  }

  // uint8 values
  res = rosidl_generator_c__uint8__Sequence__init(&sequences->uint8_values, NESTED_SIZE);
  EXPECT_EQ(true, res);
  uint8_t test_values_uint8[7] = {0, 5, 70, 128, 180, 220, 255};
  for (i = 0; i < NESTED_SIZE; i++) {
    sequences->uint8_values.data[i] = test_values_uint8[i];
  }
  // test values
  for (i = 0; i < NESTED_SIZE; i++) {
    EXPECT_EQ(test_values_uint8[i], sequences->uint8_values.data[i]);
  }

  // uint16 values
  res = rosidl_generator_c__uint16__Sequence__init(&sequences->uint16_values, NESTED_SIZE);
  EXPECT_EQ(true, res);
  uint16_t test_values_uint16[7] = {0U, 11111U, 22222U, 33333U, 44444U, 55555U, 65535U};
  for (i = 0; i < NESTED_SIZE; i++) {
    sequences->uint16_values.data[i] = test_values_uint16[i];
  }
  // test values
  for (i = 0; i < NESTED_SIZE; i++) {
    EXPECT_EQ(test_values_uint16[i], sequences->uint16_values.data[i]);
  }

  // uint32 values
  res = rosidl_generator_c__uint32__Sequence__init(&sequences->uint32_values, NESTED_SIZE);
  EXPECT_EQ(true, res);
  uint32_t test_values_uint32[7] =
  {0UL, 100UL, 2000UL, 30000UL, 444444UL, 567890123UL, 4294967295UL};
  for (i = 0; i < NESTED_SIZE; i++) {
    sequences->uint32_values.data[i] = test_values_uint32[i];
  }
  // test values
  for (i = 0; i < NESTED_SIZE; i++) {
    EXPECT_EQ(test_values_uint32[i], sequences->uint32_values.data[i]);
  }

  // uint64 values
  res = rosidl_generator_c__uint64__Sequence__init(&sequences->uint64_values, NESTED_SIZE);
  EXPECT_EQ(true, res);
  uint64_t test_values_uint64[7] = {
    0ULL, 10000ULL, 30000000ULL, 444444444ULL, 567890123456789ULL,
    429496729578901234ULL, 18446744073709551615ULL
  };
  for (i = 0; i < NESTED_SIZE; i++) {
    sequences->uint64_values.data[i] = test_values_uint64[i];
  }
  // test values
  for (i = 0; i < NESTED_SIZE; i++) {
    EXPECT_EQ(test_values_uint64[i], sequences->uint64_values.data[i]);
  }

  rosidl_generator_c__msg__BoundedSequencePrimitives__destroy(sequences);

  return 0;
}

/**
 * Test arrays
 */
int test_array_primitives(void)
{
  int i;
  rosidl_generator_c__msg__ArrayPrimitives * arrays = NULL;

  arrays = rosidl_generator_c__msg__ArrayPrimitives__create();
  EXPECT_NE(NULL, arrays);

  // bool_values
  // load values
  for (i = 0; i < NESTED_SIZE; i++) {
    if (0 == (i % 2)) {
      arrays->bool_values[i] = true;
    } else {
      arrays->bool_values[i] = false;
    }
  }

  // test values
  for (i = 0; i < NESTED_SIZE; i++) {
    if (0 == (i % 2)) {
      EXPECT_EQ(true, arrays->bool_values[i]);
    } else {
      EXPECT_EQ(false, arrays->bool_values[i]);
    }
  }

  // byte_values
  uint8_t test_values_byte[NESTED_SIZE] = {0, 57, 110, 177, 201, 240, 255};
  for (i = 0; i < NESTED_SIZE; i++) {
    arrays->byte_values[i] = test_values_byte[i];
  }
  // test values
  for (i = 0; i < NESTED_SIZE; i++) {
    EXPECT_EQ(test_values_byte[i], arrays->byte_values[i]);
  }

  // char array
  char test_values_char[NESTED_SIZE] = {'a', '5', '#', 'Z', '@', '-', ' '};
  for (i = 0; i < NESTED_SIZE; i++) {
    arrays->char_values[i] = test_values_char[i];
  }

  // test values
  for (i = 0; i < NESTED_SIZE; i++) {
    EXPECT_EQ(test_values_char[i], arrays->char_values[i]);
  }

  // float32 array
  float test_values_float32[NESTED_SIZE] = {
    -3.000001f, 22143.541325f, 6331.00432f, -214.66241f, 0.000001f, 1415555.12345f, -1.11154f
  };
  for (i = 0; i < NESTED_SIZE; i++) {
    arrays->float32_values[i] = test_values_float32[i];
  }
  // test values
  for (i = 0; i < NESTED_SIZE; i++) {
    EXPECT_EQ(test_values_float32[i], arrays->float32_values[i]);
  }

  // float64 array
  double test_values_float64[NESTED_SIZE] = {
    -120310.00843902140001, 22143.54483920141325, 6331.0048392104432, -214.62850432596241,
    0.0000000000001, 1415555.128294031432345, -1.111184329208454
  };
  for (i = 0; i < NESTED_SIZE; i++) {
    arrays->float64_values[i] = test_values_float64[i];
  }
  // test values
  for (i = 0; i < NESTED_SIZE; i++) {
    EXPECT_EQ(test_values_float64[i], arrays->float64_values[i]);
  }

  // int8 array
  int8_t test_values_int8[NESTED_SIZE] = {-127, -55, -30, 0, 58, 100, 127};
  for (i = 0; i < NESTED_SIZE; i++) {
    arrays->int8_values[i] = test_values_int8[i];
  }
  // test values
  for (i = 0; i < NESTED_SIZE; i++) {
    EXPECT_EQ(test_values_int8[i], arrays->int8_values[i]);
  }

  // int16 array
  int16_t test_values_int16[NESTED_SIZE] = {
    -32767, -22222, -11111, 0, 11111, 22222, 32767
  };
  for (i = 0; i < NESTED_SIZE; i++) {
    arrays->int16_values[i] = test_values_int16[i];
  }
  // test values
  for (i = 0; i < NESTED_SIZE; i++) {
    EXPECT_EQ(test_values_int16[i], arrays->int16_values[i]);
  }

  // int32 array
  int32_t test_values_int32[NESTED_SIZE] =
  {INT32_MIN, INT32_MIN / 2, INT32_MIN / 4, 0L, INT32_MAX / 4, INT32_MAX / 2, INT32_MAX};
  for (i = 0; i < NESTED_SIZE; i++) {
    arrays->int32_values[i] = test_values_int32[i];
  }
  // test values
  for (i = 0; i < NESTED_SIZE; i++) {
    EXPECT_EQ(test_values_int32[i], arrays->int32_values[i]);
  }

  // int64 array
  int64_t test_values_int64[NESTED_SIZE] = {
    -9223372036854775807LL, -5000000000000000000LL, -1111111111111111111LL, 0,
    1111111111111111111ULL, 5000000000000000000ULL, 9223372036854775807ULL
  };
  for (i = 0; i < NESTED_SIZE; i++) {
    arrays->int64_values[i] = test_values_int64[i];
  }
  // test values
  for (i = 0; i < NESTED_SIZE; i++) {
    EXPECT_EQ(test_values_int64[i], arrays->int64_values[i]);
  }

  // uint8 array
  uint8_t test_values_uint8[NESTED_SIZE] = {0, 5, 70, 128, 180, 220, 255};
  for (i = 0; i < NESTED_SIZE; i++) {
    arrays->uint8_values[i] = test_values_uint8[i];
  }
  // test values
  for (i = 0; i < NESTED_SIZE; i++) {
    EXPECT_EQ(test_values_uint8[i], arrays->uint8_values[i]);
  }

  // uint16 array
  uint16_t test_values_uint16[NESTED_SIZE] = {0U, 11111U, 22222U, 33333U, 44444U, 55555U, 65535U};
  for (i = 0; i < NESTED_SIZE; i++) {
    arrays->uint16_values[i] = test_values_uint16[i];
  }
  // test values
  for (i = 0; i < NESTED_SIZE; i++) {
    EXPECT_EQ(test_values_uint16[i], arrays->uint16_values[i]);
  }

  // uint32 array
  uint32_t test_values_uint32[NESTED_SIZE] = {
    0UL, 100UL, 2000UL, 30000UL, 444444UL, 567890123UL, 4294967295UL
  };
  for (i = 0; i < NESTED_SIZE; i++) {
    arrays->uint32_values[i] = test_values_uint32[i];
  }
  // test values
  for (i = 0; i < NESTED_SIZE; i++) {
    EXPECT_EQ(test_values_uint32[i], arrays->uint32_values[i]);
  }

  // uint64 array
  uint64_t test_values_uint64[NESTED_SIZE] = {
    0ULL, 10000ULL, 30000000ULL, 444444444ULL, 567890123456789ULL,
    429496729578901234ULL, 18446744073709551615ULL
  };
  for (i = 0; i < NESTED_SIZE; i++) {
    arrays->uint64_values[i] = test_values_uint64[i];
  }
  // test values
  for (i = 0; i < NESTED_SIZE; i++) {
    EXPECT_EQ(test_values_uint64[i], arrays->uint64_values[i]);
  }

  rosidl_generator_c__msg__ArrayPrimitives__destroy(arrays);

  return 0;
}

/**
 * Test bounded sequence of nested type
 */
int test_bounded_sequence_nested(void)
{
  bool res;
  // We don't populate every sequence element to test the destruction of uninitialized sequences
  size_t size = 4;
  rosidl_generator_c__msg__BoundedSequenceNested * msg =
    rosidl_generator_c__msg__BoundedSequenceNested__create();

  rosidl_generator_c__msg__Primitives__Sequence__init(&msg->primitives_values, size);
  msg->primitives_values.data[0].bool_value = false;
  msg->primitives_values.data[1].bool_value = true;
  msg->primitives_values.data[0].byte_value = 0;
  msg->primitives_values.data[1].byte_value = UINT8_MAX;
  msg->primitives_values.data[0].char_value = 0;
  msg->primitives_values.data[1].char_value = UINT8_MAX;
  msg->primitives_values.data[0].float32_value = FLT_MIN;
  msg->primitives_values.data[1].float32_value = FLT_MAX;
  msg->primitives_values.data[0].float64_value = DBL_MIN;
  msg->primitives_values.data[1].float64_value = DBL_MAX;
  msg->primitives_values.data[0].int8_value = INT8_MIN;
  msg->primitives_values.data[1].int8_value = INT8_MAX;
  msg->primitives_values.data[0].uint8_value = 0;
  msg->primitives_values.data[1].uint8_value = UINT8_MAX;
  msg->primitives_values.data[0].int16_value = INT16_MIN;
  msg->primitives_values.data[1].int16_value = INT16_MAX;
  msg->primitives_values.data[0].uint16_value = 0;
  msg->primitives_values.data[1].uint16_value = UINT16_MAX;
  msg->primitives_values.data[0].int32_value = INT32_MIN;
  msg->primitives_values.data[1].int32_value = INT32_MAX;
  msg->primitives_values.data[0].uint32_value = 0;
  msg->primitives_values.data[1].uint32_value = UINT32_MAX;
  msg->primitives_values.data[0].int64_value = INT64_MIN;
  msg->primitives_values.data[1].int64_value = INT64_MAX;
  msg->primitives_values.data[0].uint64_value = 0;
  msg->primitives_values.data[1].uint64_value = UINT64_MAX;
  res = rosidl_generator_c__String__assign(
    &msg->primitives_values.data[0].string_value, TEST_STRING);
  EXPECT_EQ(true, res);
  res = rosidl_generator_c__String__assign(
    &msg->primitives_values.data[1].string_value, TEST_STRING);
  EXPECT_EQ(true, res);

  // now lets verify if it's actually the expected values
  EXPECT_EQ(false, msg->primitives_values.data[0].bool_value);
  EXPECT_EQ(true, msg->primitives_values.data[1].bool_value);
  EXPECT_EQ(0, msg->primitives_values.data[0].byte_value);
  EXPECT_EQ(UINT8_MAX, msg->primitives_values.data[1].byte_value);
  EXPECT_EQ(0, msg->primitives_values.data[0].char_value);
  EXPECT_EQ(UINT8_MAX, msg->primitives_values.data[1].char_value);
  EXPECT_EQ(FLT_MIN, msg->primitives_values.data[0].float32_value);
  EXPECT_EQ(FLT_MAX, msg->primitives_values.data[1].float32_value);
  EXPECT_EQ(DBL_MIN, msg->primitives_values.data[0].float64_value);
  EXPECT_EQ(DBL_MAX, msg->primitives_values.data[1].float64_value);
  EXPECT_EQ(INT8_MIN, msg->primitives_values.data[0].int8_value);
  EXPECT_EQ(INT8_MAX, msg->primitives_values.data[1].int8_value);
  EXPECT_EQ(0, msg->primitives_values.data[0].uint8_value);
  EXPECT_EQ(UINT8_MAX, msg->primitives_values.data[1].uint8_value);
  EXPECT_EQ(INT16_MIN, msg->primitives_values.data[0].int16_value);
  EXPECT_EQ(INT16_MAX, msg->primitives_values.data[1].int16_value);
  EXPECT_EQ(0, msg->primitives_values.data[0].uint16_value);
  EXPECT_EQ(UINT16_MAX, msg->primitives_values.data[1].uint16_value);
  EXPECT_EQ(INT32_MIN, msg->primitives_values.data[0].int32_value);
  EXPECT_EQ(INT32_MAX, msg->primitives_values.data[1].int32_value);
  EXPECT_EQ(0, msg->primitives_values.data[0].uint32_value);
  EXPECT_EQ(UINT32_MAX, msg->primitives_values.data[1].uint32_value);
  EXPECT_EQ(INT64_MIN, msg->primitives_values.data[0].int64_value);
  EXPECT_EQ(INT64_MAX, msg->primitives_values.data[1].int64_value);
  EXPECT_EQ(0, msg->primitives_values.data[0].uint64_value);
  EXPECT_EQ(UINT64_MAX, msg->primitives_values.data[1].uint64_value);

  EXPECT_EQ(0, strcmp(msg->primitives_values.data[0].string_value.data, TEST_STRING));
  EXPECT_EQ(0, strcmp(msg->primitives_values.data[1].string_value.data, TEST_STRING));

  rosidl_generator_c__msg__BoundedSequenceNested__destroy(msg);

  return 0;
}

/**
 * Test unbounded sequence of nested type
 */
int test_unbounded_sequence_nested(void)
{
  bool res;
  // We don't populate every sequence element to test the destruction of uninitialized sequences
  size_t size = 4;

  rosidl_generator_c__msg__UnboundedSequenceNested * msg =
    rosidl_generator_c__msg__UnboundedSequenceNested__create();

  rosidl_generator_c__msg__Primitives__Sequence__init(&msg->primitives_values, size);
  msg->primitives_values.data[0].bool_value = false;
  msg->primitives_values.data[1].bool_value = true;
  msg->primitives_values.data[0].byte_value = 0;
  msg->primitives_values.data[1].byte_value = UINT8_MAX;
  msg->primitives_values.data[0].char_value = 0;
  msg->primitives_values.data[1].char_value = UINT8_MAX;
  msg->primitives_values.data[0].float32_value = FLT_MIN;
  msg->primitives_values.data[1].float32_value = FLT_MAX;
  msg->primitives_values.data[0].float64_value = DBL_MIN;
  msg->primitives_values.data[1].float64_value = DBL_MAX;
  msg->primitives_values.data[0].int8_value = INT8_MIN;
  msg->primitives_values.data[1].int8_value = INT8_MAX;
  msg->primitives_values.data[0].uint8_value = 0;
  msg->primitives_values.data[1].uint8_value = UINT8_MAX;
  msg->primitives_values.data[0].int16_value = INT16_MIN;
  msg->primitives_values.data[1].int16_value = INT16_MAX;
  msg->primitives_values.data[0].uint16_value = 0;
  msg->primitives_values.data[1].uint16_value = UINT16_MAX;
  msg->primitives_values.data[0].int32_value = INT32_MIN;
  msg->primitives_values.data[1].int32_value = INT32_MAX;
  msg->primitives_values.data[0].uint32_value = 0;
  msg->primitives_values.data[1].uint32_value = UINT32_MAX;
  msg->primitives_values.data[0].int64_value = INT64_MIN;
  msg->primitives_values.data[1].int64_value = INT64_MAX;
  msg->primitives_values.data[0].uint64_value = 0;
  msg->primitives_values.data[1].uint64_value = UINT64_MAX;
  res = rosidl_generator_c__String__assign(
    &msg->primitives_values.data[0].string_value, TEST_STRING);
  EXPECT_EQ(true, res);
  res = rosidl_generator_c__String__assign(
    &msg->primitives_values.data[1].string_value, TEST_STRING);
  EXPECT_EQ(true, res);

  // now lets verify if it's actually the expected values
  EXPECT_EQ(false, msg->primitives_values.data[0].bool_value);
  EXPECT_EQ(true, msg->primitives_values.data[1].bool_value);
  EXPECT_EQ(0, msg->primitives_values.data[0].byte_value);
  EXPECT_EQ(UINT8_MAX, msg->primitives_values.data[1].byte_value);
  EXPECT_EQ(0, msg->primitives_values.data[0].char_value);
  EXPECT_EQ(UINT8_MAX, msg->primitives_values.data[1].char_value);
  EXPECT_EQ(FLT_MIN, msg->primitives_values.data[0].float32_value);
  EXPECT_EQ(FLT_MAX, msg->primitives_values.data[1].float32_value);
  EXPECT_EQ(DBL_MIN, msg->primitives_values.data[0].float64_value);
  EXPECT_EQ(DBL_MAX, msg->primitives_values.data[1].float64_value);
  EXPECT_EQ(INT8_MIN, msg->primitives_values.data[0].int8_value);
  EXPECT_EQ(INT8_MAX, msg->primitives_values.data[1].int8_value);
  EXPECT_EQ(0, msg->primitives_values.data[0].uint8_value);
  EXPECT_EQ(UINT8_MAX, msg->primitives_values.data[1].uint8_value);
  EXPECT_EQ(INT16_MIN, msg->primitives_values.data[0].int16_value);
  EXPECT_EQ(INT16_MAX, msg->primitives_values.data[1].int16_value);
  EXPECT_EQ(0, msg->primitives_values.data[0].uint16_value);
  EXPECT_EQ(UINT16_MAX, msg->primitives_values.data[1].uint16_value);
  EXPECT_EQ(INT32_MIN, msg->primitives_values.data[0].int32_value);
  EXPECT_EQ(INT32_MAX, msg->primitives_values.data[1].int32_value);
  EXPECT_EQ(0, msg->primitives_values.data[0].uint32_value);
  EXPECT_EQ(UINT32_MAX, msg->primitives_values.data[1].uint32_value);
  EXPECT_EQ(INT64_MIN, msg->primitives_values.data[0].int64_value);
  EXPECT_EQ(INT64_MAX, msg->primitives_values.data[1].int64_value);
  EXPECT_EQ(0, msg->primitives_values.data[0].uint64_value);
  EXPECT_EQ(UINT64_MAX, msg->primitives_values.data[1].uint64_value);

  EXPECT_EQ(0, strcmp(msg->primitives_values.data[0].string_value.data, TEST_STRING));
  EXPECT_EQ(0, strcmp(msg->primitives_values.data[1].string_value.data, TEST_STRING));

  rosidl_generator_c__msg__UnboundedSequenceNested__destroy(msg);

  return 0;
}

/**
 * Test array with nested type
 */
int test_array_nested(void)
{
  size_t i;
  bool res;
  // We don't populate every array element to test the destruction of uninitialized arrays
  size_t size = 4;

  rosidl_generator_c__msg__ArrayNested * msg =
    rosidl_generator_c__msg__ArrayNested__create();

  for (i = 0; i < size; i++) {
    msg->primitives_values[i].bool_value = (i % 2 == 0) ? false : true;
    msg->primitives_values[i].byte_value = (i % 2 == 0) ? 0 : UINT8_MAX;
    msg->primitives_values[i].char_value = (i % 2 == 0) ? 0 : UINT8_MAX;
    msg->primitives_values[i].float32_value = (i % 2 == 0) ? FLT_MIN : FLT_MAX;
    msg->primitives_values[i].float64_value = (i % 2 == 0) ? DBL_MIN : DBL_MAX;
    msg->primitives_values[i].int8_value = (i % 2 == 0) ? INT8_MIN : INT8_MAX;
    msg->primitives_values[i].uint8_value = (i % 2 == 0) ? 0 : UINT8_MAX;
    msg->primitives_values[i].int16_value = (i % 2 == 0) ? INT16_MIN : INT16_MAX;
    msg->primitives_values[i].uint16_value = (i % 2 == 0) ? 0 : UINT16_MAX;
    msg->primitives_values[i].int32_value = (i % 2 == 0) ? INT32_MIN : INT32_MAX;
    msg->primitives_values[i].uint32_value = (i % 2 == 0) ? 0 : UINT32_MAX;
    msg->primitives_values[i].int64_value = (i % 2 == 0) ? INT64_MIN : INT64_MAX;
    msg->primitives_values[i].uint64_value = (i % 2 == 0) ? 0 : UINT64_MAX;
    res = rosidl_generator_c__String__assign(&msg->primitives_values[i].string_value, "Test 1");
    EXPECT_EQ(true, res);
  }

  for (i = 0; i < 4; i++) {
    EXPECT_EQ(((i % 2 == 0) ? false : true), msg->primitives_values[i].bool_value);
    EXPECT_EQ(((i % 2 == 0) ? 0 : UINT8_MAX), msg->primitives_values[i].byte_value);
    EXPECT_EQ(((i % 2 == 0) ? 0 : UINT8_MAX), msg->primitives_values[i].char_value);
    EXPECT_EQ(((i % 2 == 0) ? FLT_MIN : FLT_MAX), msg->primitives_values[i].float32_value);
    EXPECT_EQ(((i % 2 == 0) ? DBL_MIN : DBL_MAX), msg->primitives_values[i].float64_value);
    EXPECT_EQ(((i % 2 == 0) ? INT8_MIN : INT8_MAX), msg->primitives_values[i].int8_value);
    EXPECT_EQ(((i % 2 == 0) ? 0 : UINT8_MAX), msg->primitives_values[i].uint8_value);
    EXPECT_EQ(((i % 2 == 0) ? INT16_MIN : INT16_MAX), msg->primitives_values[i].int16_value);
    EXPECT_EQ(((i % 2 == 0) ? 0 : UINT16_MAX), msg->primitives_values[i].uint16_value);
    EXPECT_EQ(((i % 2 == 0) ? INT32_MIN : INT32_MAX), msg->primitives_values[i].int32_value);
    EXPECT_EQ(((i % 2 == 0) ? 0 : UINT32_MAX), msg->primitives_values[i].uint32_value);
    EXPECT_EQ(((i % 2 == 0) ? INT64_MIN : INT64_MAX), msg->primitives_values[i].int64_value);
    EXPECT_EQ(((i % 2 == 0) ? 0 : UINT64_MAX), msg->primitives_values[i].uint64_value);
    EXPECT_EQ(0, strcmp(msg->primitives_values[i].string_value.data, "Test 1"));
  }

  rosidl_generator_c__msg__ArrayNested__destroy(msg);

  return 0;
}

/**
 * Test unbounded sequence of unbounded sequences of primitives
 */
int test_unbounded_sequence_primitives_nested(void)
{
  size_t i;
  bool res;
  // We don't populate every sequence element to test the destruction of uninitialized sequences
  size_t size = 4;
  rosidl_generator_c__msg__UnboundedSequencePrimitivesNested * msg =
    rosidl_generator_c__msg__UnboundedSequencePrimitivesNested__create();
  rosidl_generator_c__msg__UnboundedSequencePrimitives__Sequence__init(
    &msg->unbounded_sequence_primitives_values, size);

  for (i = 0; i < size; i++) {
    res = rosidl_generator_c__boolean__Sequence__init(
      &msg->unbounded_sequence_primitives_values.data[i].bool_values, size);
    EXPECT_EQ(true, res);
    msg->unbounded_sequence_primitives_values.data[i].bool_values.data[0] = false;
    msg->unbounded_sequence_primitives_values.data[i].bool_values.data[1] = true;
    res = rosidl_generator_c__octet__Sequence__init(
      &msg->unbounded_sequence_primitives_values.data[i].byte_values, size);
    EXPECT_EQ(true, res);
    msg->unbounded_sequence_primitives_values.data[i].byte_values.data[0] = 0;
    msg->unbounded_sequence_primitives_values.data[i].byte_values.data[1] = UINT8_MAX;
    res = rosidl_generator_c__uint8__Sequence__init(
      &msg->unbounded_sequence_primitives_values.data[i].char_values, size);
    EXPECT_EQ(true, res);
    msg->unbounded_sequence_primitives_values.data[i].char_values.data[0] = 0;
    msg->unbounded_sequence_primitives_values.data[i].char_values.data[1] = UINT8_MAX;
    res = rosidl_generator_c__float__Sequence__init(
      &msg->unbounded_sequence_primitives_values.data[i].float32_values, size);
    EXPECT_EQ(true, res);
    msg->unbounded_sequence_primitives_values.data[i].float32_values.data[0] = FLT_MIN;
    msg->unbounded_sequence_primitives_values.data[i].float32_values.data[1] = FLT_MAX;
    res = rosidl_generator_c__double__Sequence__init(
      &msg->unbounded_sequence_primitives_values.data[i].float64_values, size);
    EXPECT_EQ(true, res);
    msg->unbounded_sequence_primitives_values.data[i].float64_values.data[0] = DBL_MIN;
    msg->unbounded_sequence_primitives_values.data[i].float64_values.data[1] = DBL_MAX;
    res = rosidl_generator_c__int8__Sequence__init(
      &msg->unbounded_sequence_primitives_values.data[i].int8_values, size);
    EXPECT_EQ(true, res);
    msg->unbounded_sequence_primitives_values.data[i].int8_values.data[0] = INT8_MIN;
    msg->unbounded_sequence_primitives_values.data[i].int8_values.data[1] = INT8_MAX;
    res = rosidl_generator_c__uint8__Sequence__init(
      &msg->unbounded_sequence_primitives_values.data[i].uint8_values, size);
    EXPECT_EQ(true, res);
    msg->unbounded_sequence_primitives_values.data[i].uint8_values.data[0] = 0;
    msg->unbounded_sequence_primitives_values.data[i].uint8_values.data[1] = UINT8_MAX;
    res = rosidl_generator_c__int16__Sequence__init(
      &msg->unbounded_sequence_primitives_values.data[i].int16_values, size);
    EXPECT_EQ(true, res);
    msg->unbounded_sequence_primitives_values.data[i].int16_values.data[0] = INT16_MIN;
    msg->unbounded_sequence_primitives_values.data[i].int16_values.data[1] = INT16_MAX;
    res = rosidl_generator_c__uint16__Sequence__init(
      &msg->unbounded_sequence_primitives_values.data[i].uint16_values, size);
    EXPECT_EQ(true, res);
    msg->unbounded_sequence_primitives_values.data[i].uint16_values.data[0] = 0;
    msg->unbounded_sequence_primitives_values.data[i].uint16_values.data[1] = UINT16_MAX;
    res = rosidl_generator_c__int32__Sequence__init(
      &msg->unbounded_sequence_primitives_values.data[i].int32_values, size);
    EXPECT_EQ(true, res);
    msg->unbounded_sequence_primitives_values.data[i].int32_values.data[0] = INT32_MIN;
    msg->unbounded_sequence_primitives_values.data[i].int32_values.data[1] = INT32_MAX;
    res = rosidl_generator_c__uint32__Sequence__init(
      &msg->unbounded_sequence_primitives_values.data[i].uint32_values, size);
    EXPECT_EQ(true, res);
    msg->unbounded_sequence_primitives_values.data[i].uint32_values.data[0] = 0;
    msg->unbounded_sequence_primitives_values.data[i].uint32_values.data[1] = UINT32_MAX;
    res = rosidl_generator_c__int64__Sequence__init(
      &msg->unbounded_sequence_primitives_values.data[i].int64_values, size);
    EXPECT_EQ(true, res);
    msg->unbounded_sequence_primitives_values.data[i].int64_values.data[0] = INT64_MIN;
    msg->unbounded_sequence_primitives_values.data[i].int64_values.data[1] = INT64_MAX;
    res = rosidl_generator_c__uint64__Sequence__init(
      &msg->unbounded_sequence_primitives_values.data[i].uint64_values, size);
    EXPECT_EQ(true, res);
    msg->unbounded_sequence_primitives_values.data[i].uint64_values.data[0] = 0;
    msg->unbounded_sequence_primitives_values.data[i].uint64_values.data[1] = UINT64_MAX;
    res = rosidl_generator_c__String__Sequence__init(
      &msg->unbounded_sequence_primitives_values.data[i].string_values, size);
    EXPECT_EQ(true, res);
    res = rosidl_generator_c__String__assign(
      &msg->unbounded_sequence_primitives_values.data[i].string_values.data[0], TEST_STRING);
    EXPECT_EQ(true, res);
    res = rosidl_generator_c__String__assign(
      &msg->unbounded_sequence_primitives_values.data[i].string_values.data[1], TEST_STRING);
    EXPECT_EQ(true, res);
  }

  for (i = 0; i < size; i++) {
    EXPECT_EQ(false, msg->unbounded_sequence_primitives_values.data[i].bool_values.data[0]);
    EXPECT_EQ(true, msg->unbounded_sequence_primitives_values.data[i].bool_values.data[1]);
    EXPECT_EQ(0, msg->unbounded_sequence_primitives_values.data[i].byte_values.data[0]);
    EXPECT_EQ(UINT8_MAX, msg->unbounded_sequence_primitives_values.data[i].byte_values.data[1]);
    EXPECT_EQ(0, msg->unbounded_sequence_primitives_values.data[i].char_values.data[0]);
    EXPECT_EQ(UINT8_MAX, msg->unbounded_sequence_primitives_values.data[i].char_values.data[1]);
    EXPECT_EQ(FLT_MIN, msg->unbounded_sequence_primitives_values.data[i].float32_values.data[0]);
    EXPECT_EQ(FLT_MAX, msg->unbounded_sequence_primitives_values.data[i].float32_values.data[1]);
    EXPECT_EQ(DBL_MIN, msg->unbounded_sequence_primitives_values.data[i].float64_values.data[0]);
    EXPECT_EQ(DBL_MAX, msg->unbounded_sequence_primitives_values.data[i].float64_values.data[1]);
    EXPECT_EQ(INT8_MIN, msg->unbounded_sequence_primitives_values.data[i].int8_values.data[0]);
    EXPECT_EQ(INT8_MAX, msg->unbounded_sequence_primitives_values.data[i].int8_values.data[1]);
    EXPECT_EQ(0, msg->unbounded_sequence_primitives_values.data[i].uint8_values.data[0]);
    EXPECT_EQ(UINT8_MAX, msg->unbounded_sequence_primitives_values.data[i].uint8_values.data[1]);
    EXPECT_EQ(INT16_MIN, msg->unbounded_sequence_primitives_values.data[i].int16_values.data[0]);
    EXPECT_EQ(INT16_MAX, msg->unbounded_sequence_primitives_values.data[i].int16_values.data[1]);
    EXPECT_EQ(0, msg->unbounded_sequence_primitives_values.data[i].uint16_values.data[0]);
    EXPECT_EQ(UINT16_MAX, msg->unbounded_sequence_primitives_values.data[i].uint16_values.data[1]);
    EXPECT_EQ(INT32_MIN, msg->unbounded_sequence_primitives_values.data[i].int32_values.data[0]);
    EXPECT_EQ(INT32_MAX, msg->unbounded_sequence_primitives_values.data[i].int32_values.data[1]);
    EXPECT_EQ(0, msg->unbounded_sequence_primitives_values.data[i].uint32_values.data[0]);
    EXPECT_EQ(UINT32_MAX, msg->unbounded_sequence_primitives_values.data[i].uint32_values.data[1]);
    EXPECT_EQ(INT64_MIN, msg->unbounded_sequence_primitives_values.data[i].int64_values.data[0]);
    EXPECT_EQ(INT64_MAX, msg->unbounded_sequence_primitives_values.data[i].int64_values.data[1]);
    EXPECT_EQ(0, msg->unbounded_sequence_primitives_values.data[i].uint64_values.data[0]);
    EXPECT_EQ(UINT64_MAX, msg->unbounded_sequence_primitives_values.data[i].uint64_values.data[1]);
    EXPECT_EQ(0, strcmp(
        msg->unbounded_sequence_primitives_values.data[i].string_values.data[0].data, TEST_STRING));
    EXPECT_EQ(0, strcmp(
        msg->unbounded_sequence_primitives_values.data[i].string_values.data[1].data, TEST_STRING));
  }

  rosidl_generator_c__msg__UnboundedSequencePrimitivesNested__destroy(msg);

  return 0;
}
