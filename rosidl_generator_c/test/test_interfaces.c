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
#include "rosidl_generator_c/u16string_functions.h"

#include "rosidl_generator_c/msg/bool.h"
#include "rosidl_generator_c/msg/bounded_array_nested.h"
#include "rosidl_generator_c/msg/byte.h"
#include "rosidl_generator_c/msg/char.h"
#include "rosidl_generator_c/msg/constants.h"
#include "rosidl_generator_c/msg/dynamic_array_nested.h"
#include "rosidl_generator_c/msg/dynamic_array_primitives_nested.h"
#include "rosidl_generator_c/msg/dynamic_array_primitives.h"
#include "rosidl_generator_c/msg/empty.h"
#include "rosidl_generator_c/msg/float32.h"
#include "rosidl_generator_c/msg/float64.h"
#include "rosidl_generator_c/msg/int16.h"
#include "rosidl_generator_c/msg/int32.h"
#include "rosidl_generator_c/msg/int64.h"
#include "rosidl_generator_c/msg/int8.h"
#include "rosidl_generator_c/msg/nested.h"
#include "rosidl_generator_c/msg/primitives_bounded_arrays.h"
#include "rosidl_generator_c/msg/primitives_static_arrays.h"
#include "rosidl_generator_c/msg/primitives.h"
#include "rosidl_generator_c/msg/primitives_unbounded_arrays.h"
#include "rosidl_generator_c/msg/primitive_values.h"
#include "rosidl_generator_c/msg/static_array_nested.h"
#include "rosidl_generator_c/msg/string_arrays.h"
#include "rosidl_generator_c/msg/strings.h"
#include "rosidl_generator_c/msg/telegram1.h"
#include "rosidl_generator_c/msg/telegram2.h"
#include "rosidl_generator_c/msg/uint16.h"
#include "rosidl_generator_c/msg/uint32.h"
#include "rosidl_generator_c/msg/uint64.h"
#include "rosidl_generator_c/msg/uint8.h"
#include "rosidl_generator_c/msg/various.h"
#include "rosidl_generator_c/msg/wire.h"
#include "rosidl_generator_c/msg/w_strings.h"

#define TEST_STRING \
  "Deep into that darkness peering, long I stood there wondering, fearing"
#define TEST_STRING2 \
  "The quick brown fox jumps over the lazy dog."
#define TEST_STRING3 \
  "PACK MY BOX WITH FIVE DOZEN LIQUOR JUGS."
#define TEST_WSTRING \
  u"Deep into that darkness peering, long I stood there wondering, fearing \u2122"
#define ARRAY_SIZE 7

#define STRINGIFY(x) _STRINGIFY(x)
#define _STRINGIFY(x) #x

#define EXPECT_EQ(arg1, arg2) if ((arg1) != (arg2)) { \
    fputs(STRINGIFY(arg1) " != " STRINGIFY(arg2) "\n", stderr); \
    return 1; \
}
#define EXPECT_NE(arg1, arg2) if ((arg1) == (arg2)) return 1

int test_primitives(void);
int test_primitives_default_value(void);
int test_strings(void);
int test_string_arrays(void);
int test_string_arrays_default_value(void);
int test_primitives_unbounded_arrays(void);
int test_primitives_bounded_arrays(void);
int test_primitives_static_arrays(void);
int test_submessages(void);
int test_bounded_array_nested(void);
int test_dynamic_array_nested(void);
int test_dynamic_array_primitives_nested(void);
int test_static_array_nested(void);

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
  printf("Testing string types...\n");
  if (test_strings()) {
    fprintf(stderr, "test_strings() FAILED\n");
    rc++;
  }
  printf("Testing primitives unbounded arrays types...\n");
  if (test_primitives_unbounded_arrays()) {
    fprintf(stderr, "test_primitives_unbounded_arrays() FAILED\n");
    rc++;
  }
  printf("Testing primitives bounded arrays types...\n");
  if (test_primitives_bounded_arrays()) {
    fprintf(stderr, "test_primitives_bounded_arrays() FAILED\n");
    rc++;
  }
  printf("Testing primitives static arrays types...\n");
  if (test_primitives_static_arrays()) {
    fprintf(stderr, "test_primitives_static_arrays() FAILED\n");
    rc++;
  }
  printf("Testing nested sub-messages...\n");
  if (test_submessages()) {
    fprintf(stderr, "test_submessages() FAILED\n");
    rc++;
  }
  printf("Testing static_array_nested messages...\n");
  if (test_static_array_nested()) {
    fprintf(stderr, "test_static_array_nested() FAILED\n");
    rc++;
  }
  printf("Testing bounded_array_nested messages...\n");
  if (test_bounded_array_nested()) {
    fprintf(stderr, "test_bounded_array_nested() FAILED\n");
    rc++;
  }
  printf("Testing dynamic_array_nested messages...\n");
  if (test_dynamic_array_nested()) {
    fprintf(stderr, "test_dynamic_array_nested() FAILED\n");
    rc++;
  }
  printf("Testing dynamic_array_primitives_nested messages...\n");
  if (test_dynamic_array_primitives_nested()) {
    fprintf(stderr, "test_dynamic_array_primitives_nested() FAILED\n");
  }
  printf("Testing string arrays...\n");
  if (test_string_arrays()) {
    fprintf(stderr, "test_string_arrays() FAILED\n");
    rc++;
  }
  printf("Testing string arrays with default values...\n");
  if (test_string_arrays_default_value()) {
    fprintf(stderr, "test_string_arrays_default_value() FAILED\n");
    rc++;
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
  EXPECT_EQ(true, bool_msg.empty_bool);

  bool_msg.empty_bool = false;
  EXPECT_EQ(false, bool_msg.empty_bool);

  byte_msg.empty_byte = 0;
  EXPECT_EQ(0, byte_msg.empty_byte);

  byte_msg.empty_byte = 255;
  EXPECT_EQ(255, byte_msg.empty_byte);

  EXPECT_EQ(123, rosidl_generator_c__msg__Constants__X);
  EXPECT_EQ(-123, rosidl_generator_c__msg__Constants__Y);
  EXPECT_EQ(0, strncmp(rosidl_generator_c__msg__Constants__FOO, "foo", 3));
  EXPECT_EQ(127, rosidl_generator_c__msg__Constants__TOTO);
  EXPECT_EQ(48, rosidl_generator_c__msg__Constants__TATA);

  char_msg.empty_char = 0;
  EXPECT_EQ(0, char_msg.empty_char);

  char_msg.empty_char = UINT8_MAX;
  EXPECT_EQ(UINT8_MAX, char_msg.empty_char);

  float32_msg.empty_float32 = FLT_MIN;
  EXPECT_EQ(FLT_MIN, float32_msg.empty_float32);

  float32_msg.empty_float32 = FLT_MAX;
  EXPECT_EQ(FLT_MAX, float32_msg.empty_float32);

  float64_msg.empty_float64 = DBL_MIN;
  EXPECT_EQ(DBL_MIN, float64_msg.empty_float64);

  float64_msg.empty_float64 = DBL_MAX;
  EXPECT_EQ(DBL_MAX, float64_msg.empty_float64);

  int8_msg.empty_int8 = INT8_MIN;
  EXPECT_EQ(INT8_MIN, int8_msg.empty_int8);

  int8_msg.empty_int8 = INT8_MAX;
  EXPECT_EQ(INT8_MAX, int8_msg.empty_int8);

  int16_msg.empty_int16 = INT16_MIN;
  EXPECT_EQ(INT16_MIN, int16_msg.empty_int16);

  int16_msg.empty_int16 = INT16_MAX;
  EXPECT_EQ(INT16_MAX, int16_msg.empty_int16);

  int32_msg.empty_int32 = INT32_MIN;
  EXPECT_EQ(INT32_MIN, int32_msg.empty_int32);

  int32_msg.empty_int32 = INT32_MAX;
  EXPECT_EQ(INT32_MAX, int32_msg.empty_int32);

  int64_msg.empty_int64 = INT64_MIN;
  EXPECT_EQ(INT64_MIN, int64_msg.empty_int64);

  int64_msg.empty_int64 = INT64_MAX;
  EXPECT_EQ(INT64_MAX, int64_msg.empty_int64);

  uint8_msg.empty_uint8 = 0;
  EXPECT_EQ(0, uint8_msg.empty_uint8);

  uint8_msg.empty_uint8 = UINT8_MAX;
  EXPECT_EQ(UINT8_MAX, uint8_msg.empty_uint8);

  uint16_msg.empty_uint16 = 0;
  EXPECT_EQ(0, uint16_msg.empty_uint16);

  uint16_msg.empty_uint16 = UINT16_MAX;
  EXPECT_EQ(UINT16_MAX, uint16_msg.empty_uint16);

  uint32_msg.empty_uint32 = 0;
  EXPECT_EQ(0, uint32_msg.empty_uint32);

  uint32_msg.empty_uint32 = UINT32_MAX;
  EXPECT_EQ(UINT32_MAX, uint32_msg.empty_uint32);

  uint64_msg.empty_uint64 = 0;
  EXPECT_EQ(0, uint64_msg.empty_uint64);

  uint64_msg.empty_uint64 = UINT64_MAX;
  EXPECT_EQ(UINT64_MAX, uint64_msg.empty_uint64);

  return 0;
}

/**
 * Test message with simple primitive types using a default value initializer.
 */
int test_primitives_default_value(void)
{
  rosidl_generator_c__msg__PrimitiveValues * primitive_values = NULL;

  primitive_values = rosidl_generator_c__msg__PrimitiveValues__create();

  EXPECT_NE(primitive_values, NULL);

  EXPECT_EQ(true, primitive_values->def_bool_1);
  EXPECT_EQ(false, primitive_values->def_bool_2);
  EXPECT_EQ(66, primitive_values->def_byte);
  EXPECT_EQ(66, primitive_values->def_char);
  EXPECT_EQ(1.125f, primitive_values->def_float32);
  EXPECT_EQ(1.125, primitive_values->def_float64);
  EXPECT_EQ(3, primitive_values->def_int8);
  EXPECT_EQ(6, primitive_values->def_int16);
  EXPECT_EQ(10, primitive_values->def_int32);
  EXPECT_EQ(15, primitive_values->def_int64);
  EXPECT_EQ(33, primitive_values->def_uint8);
  EXPECT_EQ(36, primitive_values->def_uint16);
  EXPECT_EQ(310, primitive_values->def_uint32);
  EXPECT_EQ(315, primitive_values->def_uint64);

  rosidl_generator_c__msg__PrimitiveValues__destroy(primitive_values);

  return 0;
}

/**
 * Test message with different string types.
 */
int test_strings(void)
{
  bool res = false;
  rosidl_generator_c__msg__Strings * strings = NULL;

  strings = rosidl_generator_c__msg__Strings__create();
  EXPECT_NE(strings, NULL);

  res = rosidl_generator_c__String__assign(&strings->empty_string, TEST_STRING);
  EXPECT_EQ(true, res);
  EXPECT_EQ(0, strcmp(strings->empty_string.data, TEST_STRING));
  EXPECT_EQ(0, strcmp(strings->def_string.data, "Hello world!"));
  EXPECT_EQ(0, strcmp(strings->def_string2.data, "Hello'world!"));
  EXPECT_EQ(0, strcmp(strings->def_string3.data, "Hello\"world!"));
  EXPECT_EQ(0, strcmp(strings->def_string4.data, "Hello'world!"));
  EXPECT_EQ(0, strcmp(strings->def_string5.data, "Hello\"world!"));
  // since upper-bound checking is not implemented yet, we restrict the string copying
  // TODO(mikaelarguedas) Test string length properly instead of cheating copy
  // res = rosidl_generator_c__String__assign(&strings->ub_string, TEST_STRING);
  res = rosidl_generator_c__String__assignn(&strings->ub_string, TEST_STRING, 22);
  EXPECT_EQ(true, res);
  EXPECT_EQ(0, strcmp(strings->ub_string.data, "Deep into that darknes"));
  EXPECT_EQ(0, strcmp(strings->ub_def_string.data, "Upper bounded string."));

  rosidl_generator_c__msg__Strings__destroy(strings);

  return 0;
}

/**
 * Test message with different wstring types.
 */
int test_wstrings(void)
{
  bool res = false;
  rosidl_generator_c__msg__WStrings * wstrings = NULL;

  wstrings = rosidl_generator_c__msg__WStrings__create();
  EXPECT_NE(wstrings, NULL);

  res = rosidl_generator_c__U16String__assign(&wstrings->empty_wstring, TEST_WSTRING);
  EXPECT_EQ(true, res);
  // TODO(dirk-thomas) to be reenabled with the fields in the message
  // EXPECT_EQ(0, strcmp(wstrings->empty_wstring.data, TEST_WSTRING));
  // EXPECT_EQ(0, strcmp(wstrings->def_wstring.data, "Hello world!"));
  // EXPECT_EQ(0, strcmp(wstrings->def_wstring2.data, "Hello'world!"));
  // EXPECT_EQ(0, strcmp(wstrings->def_wstring3.data, "Hello\"world!"));
  // EXPECT_EQ(0, strcmp(wstrings->def_wstring4.data, "Hello'world!"));
  // EXPECT_EQ(0, strcmp(wstrings->def_wstring5.data, "Hello\"world!"));
  // since upper-bound checking is not implemented yet, we restrict the string copying
  // TODO(mikaelarguedas) Test string length properly instead of cheating copy
  // res = rosidl_generator_c__String__assign(&wstrings->ub_wstring, TEST_WSTRING);
  res = rosidl_generator_c__U16String__assignn(&wstrings->ub_wstring, TEST_WSTRING, 24);
  EXPECT_EQ(true, res);
  // EXPECT_EQ(0, strcmp(wstrings->ub_wstring.data, "Deep into that darknes"));
  // EXPECT_EQ(0, strcmp(wstrings->ub_def_wstring.data, "Upper bounded string."));

  rosidl_generator_c__msg__WStrings__destroy(wstrings);

  return 0;
}

/**
 * Test message with different string array types.
 */
int test_string_arrays(void)
{
  bool res = false;
  rosidl_generator_c__msg__StringArrays * strings = rosidl_generator_c__msg__StringArrays__create();

  EXPECT_NE(strings, NULL);

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
}

/**
 * Test different string array types using a default value initializer.
 */
int test_string_arrays_default_value(void)
{
  rosidl_generator_c__msg__StringArrays * string_arrays = NULL;

  string_arrays = rosidl_generator_c__msg__StringArrays__create();

  EXPECT_NE(string_arrays, NULL);

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
}


/**
 * Test message with different unbounded array types
 */
int test_primitives_unbounded_arrays(void)
{
  bool res = false;
  int i;
  rosidl_generator_c__msg__PrimitivesUnboundedArrays * arrays = NULL;

  arrays = rosidl_generator_c__msg__PrimitivesUnboundedArrays__create();
  EXPECT_NE(NULL, arrays);

  // bool_array
  res = rosidl_generator_c__boolean__Sequence__init(&arrays->bool_array, ARRAY_SIZE);
  EXPECT_EQ(true, res);
  // load values
  for (i = 0; i < ARRAY_SIZE; i++) {
    if (0 == (i % 2)) {
      arrays->bool_array.data[i] = true;
    } else {
      arrays->bool_array.data[i] = false;
    }
  }
  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    if (0 == (i % 2)) {
      EXPECT_EQ(true, arrays->bool_array.data[i]);
    } else {
      EXPECT_EQ(false, arrays->bool_array.data[i]);
    }
  }

  // byte_array
  res = rosidl_generator_c__octet__Sequence__init(&arrays->byte_array, ARRAY_SIZE);
  EXPECT_EQ(true, res);
  uint8_t test_array_byte[7] = {0, 57, 110, 177, 201, 240, 255};
  for (i = 0; i < ARRAY_SIZE; i++) {
    arrays->byte_array.data[i] = test_array_byte[i];
  }
  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    EXPECT_EQ(test_array_byte[i], arrays->byte_array.data[i]);
  }


  // char array
  res = rosidl_generator_c__uint8__Sequence__init(&arrays->char_array, ARRAY_SIZE);
  EXPECT_EQ(true, res);
  char test_array_char[7] = {'a', '5', '#', 'Z', '@', '-', ' '};
  for (i = 0; i < ARRAY_SIZE; i++) {
    arrays->char_array.data[i] = test_array_char[i];
  }
  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    EXPECT_EQ(test_array_char[i], arrays->char_array.data[i]);
  }

  // float32 array
  res = rosidl_generator_c__float__Sequence__init(&arrays->float32_array, ARRAY_SIZE);
  EXPECT_EQ(true, res);
  float test_array_float32[7] = {
    -3.000001f, 22143.541325f, 6331.00432f, -214.66241f, 0.000001f, 1415555.12345f, -1.11154f
  };
  for (i = 0; i < ARRAY_SIZE; i++) {
    arrays->float32_array.data[i] = test_array_float32[i];
  }
  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    EXPECT_EQ(test_array_float32[i], arrays->float32_array.data[i]);
  }

  // float64 array
  res = rosidl_generator_c__double__Sequence__init(&arrays->float64_array, ARRAY_SIZE);
  EXPECT_EQ(true, res);
  double test_array_float64[7] = {
    -120310.00843902140001, 22143.54483920141325, 6331.0048392104432, -214.62850432596241,
    0.0000000000001, 1415555.128294031432345, -1.111184329208454
  };
  for (i = 0; i < ARRAY_SIZE; i++) {
    arrays->float64_array.data[i] = test_array_float64[i];
  }
  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    EXPECT_EQ(test_array_float64[i], arrays->float64_array.data[i]);
  }

  // int8 array
  res = rosidl_generator_c__int8__Sequence__init(&arrays->int8_array, ARRAY_SIZE);
  EXPECT_EQ(true, res);
  int8_t test_array_int8[7] = {-127, -55, -30, 0, 58, 100, 127};
  for (i = 0; i < ARRAY_SIZE; i++) {
    arrays->int8_array.data[i] = test_array_int8[i];
  }
  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    EXPECT_EQ(test_array_int8[i], arrays->int8_array.data[i]);
  }

  // int16 array
  res = rosidl_generator_c__int16__Sequence__init(&arrays->int16_array, ARRAY_SIZE);
  EXPECT_EQ(true, res);
  int16_t test_array_int16[7] =
  {-32767, -22222, -11111, 0, 11111, 22222, 32767};
  for (i = 0; i < ARRAY_SIZE; i++) {
    arrays->int16_array.data[i] = test_array_int16[i];
  }
  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    EXPECT_EQ(test_array_int16[i], arrays->int16_array.data[i]);
  }

  // int32 array
  res = rosidl_generator_c__int32__Sequence__init(&arrays->int32_array, ARRAY_SIZE);
  EXPECT_EQ(true, res);
  int32_t test_array_int32[7] =
  {INT32_MIN, INT32_MIN / 2, INT32_MIN / 4, 0L, INT32_MAX / 4, INT32_MAX / 2, INT32_MAX};
  for (i = 0; i < ARRAY_SIZE; i++) {
    arrays->int32_array.data[i] = test_array_int32[i];
  }
  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    EXPECT_EQ(test_array_int32[i], arrays->int32_array.data[i]);
  }

  // int64 array
  res = rosidl_generator_c__int64__Sequence__init(&arrays->int64_array, ARRAY_SIZE);
  EXPECT_EQ(true, res);
  int64_t test_array_int64[7] = {
    -9223372036854775807LL, -5000000000000000000LL, -1111111111111111111LL, 0,
    1111111111111111111LL, 5000000000000000000LL, 9223372036854775807LL
  };
  for (i = 0; i < ARRAY_SIZE; i++) {
    arrays->int64_array.data[i] = test_array_int64[i];
  }
  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    EXPECT_EQ(test_array_int64[i], arrays->int64_array.data[i]);
  }

  // uint8 array
  res = rosidl_generator_c__uint8__Sequence__init(&arrays->uint8_array, ARRAY_SIZE);
  EXPECT_EQ(true, res);
  uint8_t test_array_uint8[7] = {0, 5, 70, 128, 180, 220, 255};
  for (i = 0; i < ARRAY_SIZE; i++) {
    arrays->uint8_array.data[i] = test_array_uint8[i];
  }
  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    EXPECT_EQ(test_array_uint8[i], arrays->uint8_array.data[i]);
  }

  // uint16 array
  res = rosidl_generator_c__uint16__Sequence__init(&arrays->uint16_array, ARRAY_SIZE);
  EXPECT_EQ(true, res);
  uint16_t test_array_uint16[7] = {0U, 11111U, 22222U, 33333U, 44444U, 55555U, 65535U};
  for (i = 0; i < ARRAY_SIZE; i++) {
    arrays->uint16_array.data[i] = test_array_uint16[i];
  }
  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    EXPECT_EQ(test_array_uint16[i], arrays->uint16_array.data[i]);
  }

  // uint32 array
  res = rosidl_generator_c__uint32__Sequence__init(&arrays->uint32_array, ARRAY_SIZE);
  EXPECT_EQ(true, res);
  uint32_t test_array_uint32[7] = {
    0UL, 100UL, 2000UL, 30000UL, 444444UL, 567890123UL, 4294967295UL
  };
  for (i = 0; i < ARRAY_SIZE; i++) {
    arrays->uint32_array.data[i] = test_array_uint32[i];
  }
  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    EXPECT_EQ(test_array_uint32[i], arrays->uint32_array.data[i]);
  }

  // uint64 array
  res = rosidl_generator_c__uint64__Sequence__init(&arrays->uint64_array, ARRAY_SIZE);
  EXPECT_EQ(true, res);
  uint64_t test_array_uint64[7] = {
    0ULL, 10000ULL, 30000000ULL, 444444444ULL, 567890123456789ULL,
    429496729578901234ULL, 18446744073709551615ULL
  };
  for (i = 0; i < ARRAY_SIZE; i++) {
    arrays->uint64_array.data[i] = test_array_uint64[i];
  }
  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    EXPECT_EQ(test_array_uint64[i], arrays->uint64_array.data[i]);
  }

  rosidl_generator_c__msg__PrimitivesUnboundedArrays__destroy(arrays);

  return 0;
}

/**
 * Test message with different bounded array types
 */
int test_primitives_bounded_arrays(void)
{
  bool res = false;
  int i;
  rosidl_generator_c__msg__PrimitivesBoundedArrays * arrays = NULL;

  arrays = rosidl_generator_c__msg__PrimitivesBoundedArrays__create();
  EXPECT_NE(NULL, arrays);

  // bool_array
  res = rosidl_generator_c__boolean__Sequence__init(&arrays->bool_array, ARRAY_SIZE);
  EXPECT_EQ(true, res);
  // load values
  for (i = 0; i < ARRAY_SIZE; i++) {
    if (0 == (i % 2)) {
      arrays->bool_array.data[i] = true;
    } else {
      arrays->bool_array.data[i] = false;
    }
  }
  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    if (0 == (i % 2)) {
      EXPECT_EQ(true, arrays->bool_array.data[i]);
    } else {
      EXPECT_EQ(false, arrays->bool_array.data[i]);
    }
  }

  // byte_array
  res = rosidl_generator_c__octet__Sequence__init(&arrays->byte_array, ARRAY_SIZE);
  EXPECT_EQ(true, res);
  uint8_t test_array_byte[8] = {0, 57, 110, 177, 201, 240, 255, 111};
  for (i = 0; i < ARRAY_SIZE; i++) {
    arrays->byte_array.data[i] = test_array_byte[i];
  }
  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    EXPECT_EQ(test_array_byte[i], arrays->byte_array.data[i]);
  }


  // char array
  res = rosidl_generator_c__uint8__Sequence__init(&arrays->char_array, ARRAY_SIZE);
  EXPECT_EQ(true, res);
  char test_array_char[7] = {'a', '5', '#', 'Z', '@', '-', ' '};
  for (i = 0; i < ARRAY_SIZE; i++) {
    arrays->char_array.data[i] = test_array_char[i];
  }

  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    EXPECT_EQ(test_array_char[i], arrays->char_array.data[i]);
  }

  // float32 array
  res = rosidl_generator_c__float__Sequence__init(&arrays->float32_array, ARRAY_SIZE);
  EXPECT_EQ(true, res);
  float test_array_float32[7] = {
    -3.000001f, 22143.541325f, 6331.00432f, -214.66241f, 0.000001f, 1415555.12345f, -1.11154f
  };
  for (i = 0; i < ARRAY_SIZE; i++) {
    arrays->float32_array.data[i] = test_array_float32[i];
  }
  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    EXPECT_EQ(test_array_float32[i], arrays->float32_array.data[i]);
  }

  // float64 array
  res = rosidl_generator_c__double__Sequence__init(&arrays->float64_array, ARRAY_SIZE);
  EXPECT_EQ(true, res);
  double test_array_float64[7] = {
    -120310.00843902140001, 22143.54483920141325, 6331.0048392104432, -214.62850432596241,
    0.0000000000001, 1415555.128294031432345, -1.111184329208454
  };
  for (i = 0; i < ARRAY_SIZE; i++) {
    arrays->float64_array.data[i] = test_array_float64[i];
  }
  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    EXPECT_EQ(test_array_float64[i], arrays->float64_array.data[i]);
  }

  // int8 array
  res = rosidl_generator_c__int8__Sequence__init(&arrays->int8_array, ARRAY_SIZE);
  EXPECT_EQ(true, res);
  int8_t test_array_int8[7] = {-127, -55, -30, 0, 58, 100, 127};
  for (i = 0; i < ARRAY_SIZE; i++) {
    arrays->int8_array.data[i] = test_array_int8[i];
  }
  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    EXPECT_EQ(test_array_int8[i], arrays->int8_array.data[i]);
  }

  // int16 array
  res = rosidl_generator_c__int16__Sequence__init(&arrays->int16_array, ARRAY_SIZE);
  EXPECT_EQ(true, res);
  int16_t test_array_int16[7] =
  {-32767, -22222, -11111, 0, 11111, 22222, 32767};
  for (i = 0; i < ARRAY_SIZE; i++) {
    arrays->int16_array.data[i] = test_array_int16[i];
  }
  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    EXPECT_EQ(test_array_int16[i], arrays->int16_array.data[i]);
  }

  // int32 array
  res = rosidl_generator_c__int32__Sequence__init(&arrays->int32_array, ARRAY_SIZE);
  EXPECT_EQ(true, res);
  int32_t test_array_int32[7] =
  {INT32_MIN, INT32_MIN / 2, INT32_MIN / 4, 0L, INT32_MAX / 4, INT32_MAX / 2, INT32_MAX};
  for (i = 0; i < ARRAY_SIZE; i++) {
    arrays->int32_array.data[i] = test_array_int32[i];
  }
  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    EXPECT_EQ(test_array_int32[i], arrays->int32_array.data[i]);
  }

  // int64 array
  res = rosidl_generator_c__int64__Sequence__init(&arrays->int64_array, ARRAY_SIZE);
  EXPECT_EQ(true, res);
  int64_t test_array_int64[7] = {
    -9223372036854775807LL, -5000000000000000000LL, -1111111111111111111, 0,
    1111111111111111111LL, 5000000000000000000LL, 9223372036854775807LL
  };
  for (i = 0; i < ARRAY_SIZE; i++) {
    arrays->int64_array.data[i] = test_array_int64[i];
  }
  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    EXPECT_EQ(test_array_int64[i], arrays->int64_array.data[i]);
  }

  // uint8 array
  res = rosidl_generator_c__uint8__Sequence__init(&arrays->uint8_array, ARRAY_SIZE);
  EXPECT_EQ(true, res);
  uint8_t test_array_uint8[7] = {0, 5, 70, 128, 180, 220, 255};
  for (i = 0; i < ARRAY_SIZE; i++) {
    arrays->uint8_array.data[i] = test_array_uint8[i];
  }
  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    EXPECT_EQ(test_array_uint8[i], arrays->uint8_array.data[i]);
  }

  // uint16 array
  res = rosidl_generator_c__uint16__Sequence__init(&arrays->uint16_array, ARRAY_SIZE);
  EXPECT_EQ(true, res);
  uint16_t test_array_uint16[7] = {0U, 11111U, 22222U, 33333U, 44444U, 55555U, 65535U};
  for (i = 0; i < ARRAY_SIZE; i++) {
    arrays->uint16_array.data[i] = test_array_uint16[i];
  }
  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    EXPECT_EQ(test_array_uint16[i], arrays->uint16_array.data[i]);
  }

  // uint32 array
  res = rosidl_generator_c__uint32__Sequence__init(&arrays->uint32_array, ARRAY_SIZE);
  EXPECT_EQ(true, res);
  uint32_t test_array_uint32[7] =
  {0UL, 100UL, 2000UL, 30000UL, 444444UL, 567890123UL, 4294967295UL};
  for (i = 0; i < ARRAY_SIZE; i++) {
    arrays->uint32_array.data[i] = test_array_uint32[i];
  }
  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    EXPECT_EQ(test_array_uint32[i], arrays->uint32_array.data[i]);
  }

  // uint64 array
  res = rosidl_generator_c__uint64__Sequence__init(&arrays->uint64_array, ARRAY_SIZE);
  EXPECT_EQ(true, res);
  uint64_t test_array_uint64[7] = {
    0ULL, 10000ULL, 30000000ULL, 444444444ULL, 567890123456789ULL,
    429496729578901234ULL, 18446744073709551615ULL
  };
  for (i = 0; i < ARRAY_SIZE; i++) {
    arrays->uint64_array.data[i] = test_array_uint64[i];
  }
  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    EXPECT_EQ(test_array_uint64[i], arrays->uint64_array.data[i]);
  }

  rosidl_generator_c__msg__PrimitivesBoundedArrays__destroy(arrays);

  return 0;
}

/**
 * Test message with different static array types
 */
int test_primitives_static_arrays(void)
{
  int i;
  rosidl_generator_c__msg__PrimitivesStaticArrays * arrays = NULL;

  arrays = rosidl_generator_c__msg__PrimitivesStaticArrays__create();
  EXPECT_NE(NULL, arrays);

  // bool_array
  // load values
  for (i = 0; i < ARRAY_SIZE; i++) {
    if (0 == (i % 2)) {
      arrays->bool_array[i] = true;
    } else {
      arrays->bool_array[i] = false;
    }
  }

  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    if (0 == (i % 2)) {
      EXPECT_EQ(true, arrays->bool_array[i]);
    } else {
      EXPECT_EQ(false, arrays->bool_array[i]);
    }
  }

  // byte_array
  uint8_t test_array_byte[ARRAY_SIZE] = {0, 57, 110, 177, 201, 240, 255};
  for (i = 0; i < ARRAY_SIZE; i++) {
    arrays->byte_array[i] = test_array_byte[i];
  }
  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    EXPECT_EQ(test_array_byte[i], arrays->byte_array[i]);
  }

  // char array
  char test_array_char[ARRAY_SIZE] = {'a', '5', '#', 'Z', '@', '-', ' '};
  for (i = 0; i < ARRAY_SIZE; i++) {
    arrays->char_array[i] = test_array_char[i];
  }

  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    EXPECT_EQ(test_array_char[i], arrays->char_array[i]);
  }

  // float32 array
  float test_array_float32[ARRAY_SIZE] = {
    -3.000001f, 22143.541325f, 6331.00432f, -214.66241f, 0.000001f, 1415555.12345f, -1.11154f
  };
  for (i = 0; i < ARRAY_SIZE; i++) {
    arrays->float32_array[i] = test_array_float32[i];
  }
  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    EXPECT_EQ(test_array_float32[i], arrays->float32_array[i]);
  }

  // float64 array
  double test_array_float64[ARRAY_SIZE] = {
    -120310.00843902140001, 22143.54483920141325, 6331.0048392104432, -214.62850432596241,
    0.0000000000001, 1415555.128294031432345, -1.111184329208454
  };
  for (i = 0; i < ARRAY_SIZE; i++) {
    arrays->float64_array[i] = test_array_float64[i];
  }
  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    EXPECT_EQ(test_array_float64[i], arrays->float64_array[i]);
  }

  // int8 array
  int8_t test_array_int8[ARRAY_SIZE] = {-127, -55, -30, 0, 58, 100, 127};
  for (i = 0; i < ARRAY_SIZE; i++) {
    arrays->int8_array[i] = test_array_int8[i];
  }
  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    EXPECT_EQ(test_array_int8[i], arrays->int8_array[i]);
  }

  // int16 array
  int16_t test_array_int16[ARRAY_SIZE] = {
    -32767, -22222, -11111, 0, 11111, 22222, 32767
  };
  for (i = 0; i < ARRAY_SIZE; i++) {
    arrays->int16_array[i] = test_array_int16[i];
  }
  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    EXPECT_EQ(test_array_int16[i], arrays->int16_array[i]);
  }

  // int32 array
  int32_t test_array_int32[ARRAY_SIZE] =
  {INT32_MIN, INT32_MIN / 2, INT32_MIN / 4, 0L, INT32_MAX / 4, INT32_MAX / 2, INT32_MAX};
  for (i = 0; i < ARRAY_SIZE; i++) {
    arrays->int32_array[i] = test_array_int32[i];
  }
  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    EXPECT_EQ(test_array_int32[i], arrays->int32_array[i]);
  }

  // int64 array
  int64_t test_array_int64[ARRAY_SIZE] = {
    -9223372036854775807LL, -5000000000000000000LL, -1111111111111111111LL, 0,
    1111111111111111111ULL, 5000000000000000000ULL, 9223372036854775807ULL
  };
  for (i = 0; i < ARRAY_SIZE; i++) {
    arrays->int64_array[i] = test_array_int64[i];
  }
  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    EXPECT_EQ(test_array_int64[i], arrays->int64_array[i]);
  }

  // uint8 array
  uint8_t test_array_uint8[ARRAY_SIZE] = {0, 5, 70, 128, 180, 220, 255};
  for (i = 0; i < ARRAY_SIZE; i++) {
    arrays->uint8_array[i] = test_array_uint8[i];
  }
  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    EXPECT_EQ(test_array_uint8[i], arrays->uint8_array[i]);
  }

  // uint16 array
  uint16_t test_array_uint16[ARRAY_SIZE] = {0U, 11111U, 22222U, 33333U, 44444U, 55555U, 65535U};
  for (i = 0; i < ARRAY_SIZE; i++) {
    arrays->uint16_array[i] = test_array_uint16[i];
  }
  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    EXPECT_EQ(test_array_uint16[i], arrays->uint16_array[i]);
  }

  // uint32 array
  uint32_t test_array_uint32[ARRAY_SIZE] = {
    0UL, 100UL, 2000UL, 30000UL, 444444UL, 567890123UL, 4294967295UL
  };
  for (i = 0; i < ARRAY_SIZE; i++) {
    arrays->uint32_array[i] = test_array_uint32[i];
  }
  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    EXPECT_EQ(test_array_uint32[i], arrays->uint32_array[i]);
  }

  // uint64 array
  uint64_t test_array_uint64[ARRAY_SIZE] = {
    0ULL, 10000ULL, 30000000ULL, 444444444ULL, 567890123456789ULL,
    429496729578901234ULL, 18446744073709551615ULL
  };
  for (i = 0; i < ARRAY_SIZE; i++) {
    arrays->uint64_array[i] = test_array_uint64[i];
  }
  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    EXPECT_EQ(test_array_uint64[i], arrays->uint64_array[i]);
  }

  rosidl_generator_c__msg__PrimitivesStaticArrays__destroy(arrays);

  return 0;
}

/**
 * Test message with sub-messages types
 */
int test_submessages(void)
{
  int i;
  bool res;
  rosidl_generator_c__msg__Wire * wire_msg = rosidl_generator_c__msg__Wire__create();

  for (i = 0; i < 3; i++) {
    res = rosidl_generator_c__String__assign(&wire_msg->cablegram1[i].text, TEST_STRING);
    EXPECT_EQ(true, res);
    wire_msg->cablegram1[i].number = 3.1415f;
  }
  res = rosidl_generator_c__String__assign(&wire_msg->cablegram2.text_array[0], "Test 1");
  EXPECT_EQ(true, res);
  res = rosidl_generator_c__String__assign(&wire_msg->cablegram2.text_array[1], "Test 2");
  EXPECT_EQ(true, res);
  res = rosidl_generator_c__String__assign(&wire_msg->cablegram2.text_array[2], "Test 3");
  EXPECT_EQ(true, res);
  wire_msg->cablegram2.number_array[0] = 3.1f;
  wire_msg->cablegram2.number_array[1] = 3.14f;
  wire_msg->cablegram2.number_array[2] = 3.141f;

  for (i = 0; i < 3; i++) {
    EXPECT_EQ(0, strcmp(wire_msg->cablegram1[i].text.data, TEST_STRING));
    EXPECT_EQ(3.1415f, wire_msg->cablegram1[i].number);
  }
  EXPECT_EQ(0, strcmp(wire_msg->cablegram2.text_array[0].data, "Test 1"));
  EXPECT_EQ(0, strcmp(wire_msg->cablegram2.text_array[1].data, "Test 2"));
  EXPECT_EQ(0, strcmp(wire_msg->cablegram2.text_array[2].data, "Test 3"));
  EXPECT_EQ(3.1f, wire_msg->cablegram2.number_array[0]);
  EXPECT_EQ(3.14f, wire_msg->cablegram2.number_array[1]);
  EXPECT_EQ(3.141f, wire_msg->cablegram2.number_array[2]);

  rosidl_generator_c__msg__Wire__destroy(wire_msg);

  return 0;
}

/**
 * Test message with bounded array nested types
 */
int test_bounded_array_nested(void)
{
  bool res;
  // We don't populate every array element to test the destruction of uninitialized arrays
  size_t size = 4;
  rosidl_generator_c__msg__BoundedArrayNested * msg =
    rosidl_generator_c__msg__BoundedArrayNested__create();

  rosidl_generator_c__msg__Primitives__Sequence__init(&msg->primitive_values, size);
  msg->primitive_values.data[0].bool_value = false;
  msg->primitive_values.data[1].bool_value = true;
  msg->primitive_values.data[0].byte_value = 0;
  msg->primitive_values.data[1].byte_value = UINT8_MAX;
  msg->primitive_values.data[0].char_value = 0;
  msg->primitive_values.data[1].char_value = UINT8_MAX;
  msg->primitive_values.data[0].float32_value = FLT_MIN;
  msg->primitive_values.data[1].float32_value = FLT_MAX;
  msg->primitive_values.data[0].float64_value = DBL_MIN;
  msg->primitive_values.data[1].float64_value = DBL_MAX;
  msg->primitive_values.data[0].int8_value = INT8_MIN;
  msg->primitive_values.data[1].int8_value = INT8_MAX;
  msg->primitive_values.data[0].uint8_value = 0;
  msg->primitive_values.data[1].uint8_value = UINT8_MAX;
  msg->primitive_values.data[0].int16_value = INT16_MIN;
  msg->primitive_values.data[1].int16_value = INT16_MAX;
  msg->primitive_values.data[0].uint16_value = 0;
  msg->primitive_values.data[1].uint16_value = UINT16_MAX;
  msg->primitive_values.data[0].int32_value = INT32_MIN;
  msg->primitive_values.data[1].int32_value = INT32_MAX;
  msg->primitive_values.data[0].uint32_value = 0;
  msg->primitive_values.data[1].uint32_value = UINT32_MAX;
  msg->primitive_values.data[0].int64_value = INT64_MIN;
  msg->primitive_values.data[1].int64_value = INT64_MAX;
  msg->primitive_values.data[0].uint64_value = 0;
  msg->primitive_values.data[1].uint64_value = UINT64_MAX;
  res = rosidl_generator_c__String__assign(
    &msg->primitive_values.data[0].string_value, TEST_STRING);
  EXPECT_EQ(true, res);
  res = rosidl_generator_c__String__assign(
    &msg->primitive_values.data[1].string_value, TEST_STRING);
  EXPECT_EQ(true, res);

  // now lets verify if it's actually the expected values
  EXPECT_EQ(false, msg->primitive_values.data[0].bool_value);
  EXPECT_EQ(true, msg->primitive_values.data[1].bool_value);
  EXPECT_EQ(0, msg->primitive_values.data[0].byte_value);
  EXPECT_EQ(UINT8_MAX, msg->primitive_values.data[1].byte_value);
  EXPECT_EQ(0, msg->primitive_values.data[0].char_value);
  EXPECT_EQ(UINT8_MAX, msg->primitive_values.data[1].char_value);
  EXPECT_EQ(FLT_MIN, msg->primitive_values.data[0].float32_value);
  EXPECT_EQ(FLT_MAX, msg->primitive_values.data[1].float32_value);
  EXPECT_EQ(DBL_MIN, msg->primitive_values.data[0].float64_value);
  EXPECT_EQ(DBL_MAX, msg->primitive_values.data[1].float64_value);
  EXPECT_EQ(INT8_MIN, msg->primitive_values.data[0].int8_value);
  EXPECT_EQ(INT8_MAX, msg->primitive_values.data[1].int8_value);
  EXPECT_EQ(0, msg->primitive_values.data[0].uint8_value);
  EXPECT_EQ(UINT8_MAX, msg->primitive_values.data[1].uint8_value);
  EXPECT_EQ(INT16_MIN, msg->primitive_values.data[0].int16_value);
  EXPECT_EQ(INT16_MAX, msg->primitive_values.data[1].int16_value);
  EXPECT_EQ(0, msg->primitive_values.data[0].uint16_value);
  EXPECT_EQ(UINT16_MAX, msg->primitive_values.data[1].uint16_value);
  EXPECT_EQ(INT32_MIN, msg->primitive_values.data[0].int32_value);
  EXPECT_EQ(INT32_MAX, msg->primitive_values.data[1].int32_value);
  EXPECT_EQ(0, msg->primitive_values.data[0].uint32_value);
  EXPECT_EQ(UINT32_MAX, msg->primitive_values.data[1].uint32_value);
  EXPECT_EQ(INT64_MIN, msg->primitive_values.data[0].int64_value);
  EXPECT_EQ(INT64_MAX, msg->primitive_values.data[1].int64_value);
  EXPECT_EQ(0, msg->primitive_values.data[0].uint64_value);
  EXPECT_EQ(UINT64_MAX, msg->primitive_values.data[1].uint64_value);

  EXPECT_EQ(0, strcmp(msg->primitive_values.data[0].string_value.data, TEST_STRING));
  EXPECT_EQ(0, strcmp(msg->primitive_values.data[1].string_value.data, TEST_STRING));

  rosidl_generator_c__msg__BoundedArrayNested__destroy(msg);

  return 0;
}

/**
 * Test message with sub-messages types
 */
int test_dynamic_array_nested(void)
{
  bool res;
  // We don't populate every array element to test the destruction of uninitialized arrays
  size_t size = 4;

  rosidl_generator_c__msg__DynamicArrayNested * msg =
    rosidl_generator_c__msg__DynamicArrayNested__create();

  rosidl_generator_c__msg__Primitives__Sequence__init(&msg->primitive_values, size);
  msg->primitive_values.data[0].bool_value = false;
  msg->primitive_values.data[1].bool_value = true;
  msg->primitive_values.data[0].byte_value = 0;
  msg->primitive_values.data[1].byte_value = UINT8_MAX;
  msg->primitive_values.data[0].char_value = 0;
  msg->primitive_values.data[1].char_value = UINT8_MAX;
  msg->primitive_values.data[0].float32_value = FLT_MIN;
  msg->primitive_values.data[1].float32_value = FLT_MAX;
  msg->primitive_values.data[0].float64_value = DBL_MIN;
  msg->primitive_values.data[1].float64_value = DBL_MAX;
  msg->primitive_values.data[0].int8_value = INT8_MIN;
  msg->primitive_values.data[1].int8_value = INT8_MAX;
  msg->primitive_values.data[0].uint8_value = 0;
  msg->primitive_values.data[1].uint8_value = UINT8_MAX;
  msg->primitive_values.data[0].int16_value = INT16_MIN;
  msg->primitive_values.data[1].int16_value = INT16_MAX;
  msg->primitive_values.data[0].uint16_value = 0;
  msg->primitive_values.data[1].uint16_value = UINT16_MAX;
  msg->primitive_values.data[0].int32_value = INT32_MIN;
  msg->primitive_values.data[1].int32_value = INT32_MAX;
  msg->primitive_values.data[0].uint32_value = 0;
  msg->primitive_values.data[1].uint32_value = UINT32_MAX;
  msg->primitive_values.data[0].int64_value = INT64_MIN;
  msg->primitive_values.data[1].int64_value = INT64_MAX;
  msg->primitive_values.data[0].uint64_value = 0;
  msg->primitive_values.data[1].uint64_value = UINT64_MAX;
  res = rosidl_generator_c__String__assign(
    &msg->primitive_values.data[0].string_value, TEST_STRING);
  EXPECT_EQ(true, res);
  res = rosidl_generator_c__String__assign(
    &msg->primitive_values.data[1].string_value, TEST_STRING);
  EXPECT_EQ(true, res);

  // now lets verify if it's actually the expected values
  EXPECT_EQ(false, msg->primitive_values.data[0].bool_value);
  EXPECT_EQ(true, msg->primitive_values.data[1].bool_value);
  EXPECT_EQ(0, msg->primitive_values.data[0].byte_value);
  EXPECT_EQ(UINT8_MAX, msg->primitive_values.data[1].byte_value);
  EXPECT_EQ(0, msg->primitive_values.data[0].char_value);
  EXPECT_EQ(UINT8_MAX, msg->primitive_values.data[1].char_value);
  EXPECT_EQ(FLT_MIN, msg->primitive_values.data[0].float32_value);
  EXPECT_EQ(FLT_MAX, msg->primitive_values.data[1].float32_value);
  EXPECT_EQ(DBL_MIN, msg->primitive_values.data[0].float64_value);
  EXPECT_EQ(DBL_MAX, msg->primitive_values.data[1].float64_value);
  EXPECT_EQ(INT8_MIN, msg->primitive_values.data[0].int8_value);
  EXPECT_EQ(INT8_MAX, msg->primitive_values.data[1].int8_value);
  EXPECT_EQ(0, msg->primitive_values.data[0].uint8_value);
  EXPECT_EQ(UINT8_MAX, msg->primitive_values.data[1].uint8_value);
  EXPECT_EQ(INT16_MIN, msg->primitive_values.data[0].int16_value);
  EXPECT_EQ(INT16_MAX, msg->primitive_values.data[1].int16_value);
  EXPECT_EQ(0, msg->primitive_values.data[0].uint16_value);
  EXPECT_EQ(UINT16_MAX, msg->primitive_values.data[1].uint16_value);
  EXPECT_EQ(INT32_MIN, msg->primitive_values.data[0].int32_value);
  EXPECT_EQ(INT32_MAX, msg->primitive_values.data[1].int32_value);
  EXPECT_EQ(0, msg->primitive_values.data[0].uint32_value);
  EXPECT_EQ(UINT32_MAX, msg->primitive_values.data[1].uint32_value);
  EXPECT_EQ(INT64_MIN, msg->primitive_values.data[0].int64_value);
  EXPECT_EQ(INT64_MAX, msg->primitive_values.data[1].int64_value);
  EXPECT_EQ(0, msg->primitive_values.data[0].uint64_value);
  EXPECT_EQ(UINT64_MAX, msg->primitive_values.data[1].uint64_value);

  EXPECT_EQ(0, strcmp(msg->primitive_values.data[0].string_value.data, TEST_STRING));
  EXPECT_EQ(0, strcmp(msg->primitive_values.data[1].string_value.data, TEST_STRING));

  rosidl_generator_c__msg__DynamicArrayNested__destroy(msg);

  return 0;
}

/**
 * Test message with sub-messages types
 */
int test_static_array_nested(void)
{
  size_t i;
  bool res;
  // We don't populate every array element to test the destruction of uninitialized arrays
  size_t size = 4;

  rosidl_generator_c__msg__StaticArrayNested * msg =
    rosidl_generator_c__msg__StaticArrayNested__create();

  for (i = 0; i < size; i++) {
    msg->primitive_values[i].bool_value = (i % 2 == 0) ? false : true;
    msg->primitive_values[i].byte_value = (i % 2 == 0) ? 0 : UINT8_MAX;
    msg->primitive_values[i].char_value = (i % 2 == 0) ? 0 : UINT8_MAX;
    msg->primitive_values[i].float32_value = (i % 2 == 0) ? FLT_MIN : FLT_MAX;
    msg->primitive_values[i].float64_value = (i % 2 == 0) ? DBL_MIN : DBL_MAX;
    msg->primitive_values[i].int8_value = (i % 2 == 0) ? INT8_MIN : INT8_MAX;
    msg->primitive_values[i].uint8_value = (i % 2 == 0) ? 0 : UINT8_MAX;
    msg->primitive_values[i].int16_value = (i % 2 == 0) ? INT16_MIN : INT16_MAX;
    msg->primitive_values[i].uint16_value = (i % 2 == 0) ? 0 : UINT16_MAX;
    msg->primitive_values[i].int32_value = (i % 2 == 0) ? INT32_MIN : INT32_MAX;
    msg->primitive_values[i].uint32_value = (i % 2 == 0) ? 0 : UINT32_MAX;
    msg->primitive_values[i].int64_value = (i % 2 == 0) ? INT64_MIN : INT64_MAX;
    msg->primitive_values[i].uint64_value = (i % 2 == 0) ? 0 : UINT64_MAX;
    res = rosidl_generator_c__String__assign(&msg->primitive_values[i].string_value, "Test 1");
    EXPECT_EQ(true, res);
  }

  for (i = 0; i < 4; i++) {
    EXPECT_EQ(((i % 2 == 0) ? false : true), msg->primitive_values[i].bool_value);
    EXPECT_EQ(((i % 2 == 0) ? 0 : UINT8_MAX), msg->primitive_values[i].byte_value);
    EXPECT_EQ(((i % 2 == 0) ? 0 : UINT8_MAX), msg->primitive_values[i].char_value);
    EXPECT_EQ(((i % 2 == 0) ? FLT_MIN : FLT_MAX), msg->primitive_values[i].float32_value);
    EXPECT_EQ(((i % 2 == 0) ? DBL_MIN : DBL_MAX), msg->primitive_values[i].float64_value);
    EXPECT_EQ(((i % 2 == 0) ? INT8_MIN : INT8_MAX), msg->primitive_values[i].int8_value);
    EXPECT_EQ(((i % 2 == 0) ? 0 : UINT8_MAX), msg->primitive_values[i].uint8_value);
    EXPECT_EQ(((i % 2 == 0) ? INT16_MIN : INT16_MAX), msg->primitive_values[i].int16_value);
    EXPECT_EQ(((i % 2 == 0) ? 0 : UINT16_MAX), msg->primitive_values[i].uint16_value);
    EXPECT_EQ(((i % 2 == 0) ? INT32_MIN : INT32_MAX), msg->primitive_values[i].int32_value);
    EXPECT_EQ(((i % 2 == 0) ? 0 : UINT32_MAX), msg->primitive_values[i].uint32_value);
    EXPECT_EQ(((i % 2 == 0) ? INT64_MIN : INT64_MAX), msg->primitive_values[i].int64_value);
    EXPECT_EQ(((i % 2 == 0) ? 0 : UINT64_MAX), msg->primitive_values[i].uint64_value);
    EXPECT_EQ(0, strcmp(msg->primitive_values[i].string_value.data, "Test 1"));
  }

  rosidl_generator_c__msg__StaticArrayNested__destroy(msg);

  return 0;
}

/**
 * Test message with sub-messages types
 */
int test_dynamic_array_primitives_nested(void)
{
  size_t i;
  bool res;
  // We don't populate every array element to test the destruction of uninitialized arrays
  size_t size = 4;
  rosidl_generator_c__msg__DynamicArrayPrimitivesNested * msg =
    rosidl_generator_c__msg__DynamicArrayPrimitivesNested__create();
  rosidl_generator_c__msg__DynamicArrayPrimitives__Sequence__init(&msg->msgs, size);

  for (i = 0; i < size; i++) {
    res = rosidl_generator_c__boolean__Sequence__init(&msg->msgs.data[i].bool_values, size);
    EXPECT_EQ(true, res);
    msg->msgs.data[i].bool_values.data[0] = false;
    msg->msgs.data[i].bool_values.data[1] = true;
    res = rosidl_generator_c__octet__Sequence__init(&msg->msgs.data[i].byte_values, size);
    EXPECT_EQ(true, res);
    msg->msgs.data[i].byte_values.data[0] = 0;
    msg->msgs.data[i].byte_values.data[1] = UINT8_MAX;
    res = rosidl_generator_c__uint8__Sequence__init(&msg->msgs.data[i].char_values, size);
    EXPECT_EQ(true, res);
    msg->msgs.data[i].char_values.data[0] = 0;
    msg->msgs.data[i].char_values.data[1] = UINT8_MAX;
    res = rosidl_generator_c__float__Sequence__init(&msg->msgs.data[i].float32_values, size);
    EXPECT_EQ(true, res);
    msg->msgs.data[i].float32_values.data[0] = FLT_MIN;
    msg->msgs.data[i].float32_values.data[1] = FLT_MAX;
    res = rosidl_generator_c__double__Sequence__init(&msg->msgs.data[i].float64_values, size);
    EXPECT_EQ(true, res);
    msg->msgs.data[i].float64_values.data[0] = DBL_MIN;
    msg->msgs.data[i].float64_values.data[1] = DBL_MAX;
    res = rosidl_generator_c__int8__Sequence__init(&msg->msgs.data[i].int8_values, size);
    EXPECT_EQ(true, res);
    msg->msgs.data[i].int8_values.data[0] = INT8_MIN;
    msg->msgs.data[i].int8_values.data[1] = INT8_MAX;
    res = rosidl_generator_c__uint8__Sequence__init(&msg->msgs.data[i].uint8_values, size);
    EXPECT_EQ(true, res);
    msg->msgs.data[i].uint8_values.data[0] = 0;
    msg->msgs.data[i].uint8_values.data[1] = UINT8_MAX;
    res = rosidl_generator_c__int16__Sequence__init(&msg->msgs.data[i].int16_values, size);
    EXPECT_EQ(true, res);
    msg->msgs.data[i].int16_values.data[0] = INT16_MIN;
    msg->msgs.data[i].int16_values.data[1] = INT16_MAX;
    res = rosidl_generator_c__uint16__Sequence__init(&msg->msgs.data[i].uint16_values, size);
    EXPECT_EQ(true, res);
    msg->msgs.data[i].uint16_values.data[0] = 0;
    msg->msgs.data[i].uint16_values.data[1] = UINT16_MAX;
    res = rosidl_generator_c__int32__Sequence__init(&msg->msgs.data[i].int32_values, size);
    EXPECT_EQ(true, res);
    msg->msgs.data[i].int32_values.data[0] = INT32_MIN;
    msg->msgs.data[i].int32_values.data[1] = INT32_MAX;
    res = rosidl_generator_c__uint32__Sequence__init(&msg->msgs.data[i].uint32_values, size);
    EXPECT_EQ(true, res);
    msg->msgs.data[i].uint32_values.data[0] = 0;
    msg->msgs.data[i].uint32_values.data[1] = UINT32_MAX;
    res = rosidl_generator_c__int64__Sequence__init(&msg->msgs.data[i].int64_values, size);
    EXPECT_EQ(true, res);
    msg->msgs.data[i].int64_values.data[0] = INT64_MIN;
    msg->msgs.data[i].int64_values.data[1] = INT64_MAX;
    res = rosidl_generator_c__uint64__Sequence__init(&msg->msgs.data[i].uint64_values, size);
    EXPECT_EQ(true, res);
    msg->msgs.data[i].uint64_values.data[0] = 0;
    msg->msgs.data[i].uint64_values.data[1] = UINT64_MAX;
    res = rosidl_generator_c__String__Sequence__init(&msg->msgs.data[i].string_values, size);
    EXPECT_EQ(true, res);
    res = rosidl_generator_c__String__assign(&msg->msgs.data[i].string_values.data[0], TEST_STRING);
    EXPECT_EQ(true, res);
    res = rosidl_generator_c__String__assign(&msg->msgs.data[i].string_values.data[1], TEST_STRING);
    EXPECT_EQ(true, res);
  }

  for (i = 0; i < size; i++) {
    EXPECT_EQ(false, msg->msgs.data[i].bool_values.data[0]);
    EXPECT_EQ(true, msg->msgs.data[i].bool_values.data[1]);
    EXPECT_EQ(0, msg->msgs.data[i].byte_values.data[0]);
    EXPECT_EQ(UINT8_MAX, msg->msgs.data[i].byte_values.data[1]);
    EXPECT_EQ(0, msg->msgs.data[i].char_values.data[0]);
    EXPECT_EQ(UINT8_MAX, msg->msgs.data[i].char_values.data[1]);
    EXPECT_EQ(FLT_MIN, msg->msgs.data[i].float32_values.data[0]);
    EXPECT_EQ(FLT_MAX, msg->msgs.data[i].float32_values.data[1]);
    EXPECT_EQ(DBL_MIN, msg->msgs.data[i].float64_values.data[0]);
    EXPECT_EQ(DBL_MAX, msg->msgs.data[i].float64_values.data[1]);
    EXPECT_EQ(INT8_MIN, msg->msgs.data[i].int8_values.data[0]);
    EXPECT_EQ(INT8_MAX, msg->msgs.data[i].int8_values.data[1]);
    EXPECT_EQ(0, msg->msgs.data[i].uint8_values.data[0]);
    EXPECT_EQ(UINT8_MAX, msg->msgs.data[i].uint8_values.data[1]);
    EXPECT_EQ(INT16_MIN, msg->msgs.data[i].int16_values.data[0]);
    EXPECT_EQ(INT16_MAX, msg->msgs.data[i].int16_values.data[1]);
    EXPECT_EQ(0, msg->msgs.data[i].uint16_values.data[0]);
    EXPECT_EQ(UINT16_MAX, msg->msgs.data[i].uint16_values.data[1]);
    EXPECT_EQ(INT32_MIN, msg->msgs.data[i].int32_values.data[0]);
    EXPECT_EQ(INT32_MAX, msg->msgs.data[i].int32_values.data[1]);
    EXPECT_EQ(0, msg->msgs.data[i].uint32_values.data[0]);
    EXPECT_EQ(UINT32_MAX, msg->msgs.data[i].uint32_values.data[1]);
    EXPECT_EQ(INT64_MIN, msg->msgs.data[i].int64_values.data[0]);
    EXPECT_EQ(INT64_MAX, msg->msgs.data[i].int64_values.data[1]);
    EXPECT_EQ(0, msg->msgs.data[i].uint64_values.data[0]);
    EXPECT_EQ(UINT64_MAX, msg->msgs.data[i].uint64_values.data[1]);
    EXPECT_EQ(0, strcmp(msg->msgs.data[i].string_values.data[0].data, TEST_STRING));
    EXPECT_EQ(0, strcmp(msg->msgs.data[i].string_values.data[1].data, TEST_STRING));
  }

  rosidl_generator_c__msg__DynamicArrayPrimitivesNested__destroy(msg);

  return 0;
}
