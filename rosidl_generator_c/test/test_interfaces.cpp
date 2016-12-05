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
#include <float.h>
#include <gtest/gtest.h>
#include <limits.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>


#include "rosidl_generator_c/primitives_array_functions.h"
#include "rosidl_generator_c/string_functions.h"

#include "rosidl_generator_c/msg/various__struct.h"
#include "rosidl_generator_c/msg/bool__struct.h"
#include "rosidl_generator_c/msg/byte__struct.h"
#include "rosidl_generator_c/msg/char__struct.h"
#include "rosidl_generator_c/msg/float32__struct.h"
#include "rosidl_generator_c/msg/float64__struct.h"
#include "rosidl_generator_c/msg/int8__struct.h"
#include "rosidl_generator_c/msg/int16__struct.h"
#include "rosidl_generator_c/msg/int32__struct.h"
#include "rosidl_generator_c/msg/int64__struct.h"
#include "rosidl_generator_c/msg/uint8__struct.h"
#include "rosidl_generator_c/msg/uint16__struct.h"
#include "rosidl_generator_c/msg/uint32__struct.h"
#include "rosidl_generator_c/msg/uint64__struct.h"
#include "rosidl_generator_c/msg/primitive_values__struct.h"
#include "rosidl_generator_c/msg/strings__struct.h"
#include "rosidl_generator_c/msg/primitives_unbounded_arrays__struct.h"
#include "rosidl_generator_c/msg/primitives_bounded_arrays__struct.h"
#include "rosidl_generator_c/msg/primitives_static_arrays__struct.h"
#include "rosidl_generator_c/msg/telegram1__struct.h"
#include "rosidl_generator_c/msg/telegram2__struct.h"
#include "rosidl_generator_c/msg/wire__struct.h"
#include "rosidl_generator_c/msg/constants__struct.h"

#include "rosidl_generator_c/msg/various__functions.h"
#include "rosidl_generator_c/msg/bool__functions.h"
#include "rosidl_generator_c/msg/byte__functions.h"
#include "rosidl_generator_c/msg/char__functions.h"
#include "rosidl_generator_c/msg/float32__functions.h"
#include "rosidl_generator_c/msg/float64__functions.h"
#include "rosidl_generator_c/msg/int8__functions.h"
#include "rosidl_generator_c/msg/int16__functions.h"
#include "rosidl_generator_c/msg/int32__functions.h"
#include "rosidl_generator_c/msg/int64__functions.h"
#include "rosidl_generator_c/msg/uint8__functions.h"
#include "rosidl_generator_c/msg/uint16__functions.h"
#include "rosidl_generator_c/msg/uint32__functions.h"
#include "rosidl_generator_c/msg/uint64__functions.h"
#include "rosidl_generator_c/msg/primitive_values__functions.h"
#include "rosidl_generator_c/msg/strings__functions.h"
#include "rosidl_generator_c/msg/primitives_unbounded_arrays__functions.h"
#include "rosidl_generator_c/msg/primitives_bounded_arrays__functions.h"
#include "rosidl_generator_c/msg/primitives_static_arrays__functions.h"
#include "rosidl_generator_c/msg/telegram1__functions.h"
#include "rosidl_generator_c/msg/telegram2__functions.h"
#include "rosidl_generator_c/msg/wire__functions.h"
#include "rosidl_generator_c/msg/constants__functions.h"

#define TEST_STRING \
  "Deep into that darkness peering, long I stood there wondering, fearing"

#define ARRAY_SIZE 7

/**
 * Test message with simple primitive types.
 */
TEST(Test_c_interfaces, test_primitives) {
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
  ASSERT_EQ(true, bool_msg.empty_bool);

  bool_msg.empty_bool = false;
  ASSERT_EQ(false, bool_msg.empty_bool);

  byte_msg.empty_byte = 0;
  ASSERT_EQ(0, byte_msg.empty_byte);

  byte_msg.empty_byte = 255;
  ASSERT_EQ(255, byte_msg.empty_byte);

  ASSERT_EQ(123, rosidl_generator_c__msg__Constants__X);
  ASSERT_EQ(-123, rosidl_generator_c__msg__Constants__Y);
  ASSERT_EQ(0, strncmp(rosidl_generator_c__msg__Constants__FOO, "foo", 3));
  ASSERT_EQ(127, rosidl_generator_c__msg__Constants__TOTO);
  ASSERT_EQ(48, rosidl_generator_c__msg__Constants__TATA);

  char_msg.empty_char = CHAR_MIN;
  ASSERT_EQ(CHAR_MIN, char_msg.empty_char);

  char_msg.empty_char = CHAR_MAX;
  ASSERT_EQ(CHAR_MAX, char_msg.empty_char);

  float32_msg.empty_float32 = FLT_MIN;
  ASSERT_EQ(FLT_MIN, float32_msg.empty_float32);

  float32_msg.empty_float32 = FLT_MAX;
  ASSERT_EQ(FLT_MAX, float32_msg.empty_float32);

  float64_msg.empty_float64 = DBL_MIN;
  ASSERT_EQ(DBL_MIN, float64_msg.empty_float64);

  float64_msg.empty_float64 = DBL_MAX;
  ASSERT_EQ(DBL_MAX, float64_msg.empty_float64);

  int8_msg.empty_int8 = INT8_MIN;
  ASSERT_EQ(INT8_MIN, int8_msg.empty_int8);

  int8_msg.empty_int8 = INT8_MAX;
  ASSERT_EQ(INT8_MAX, int8_msg.empty_int8);

  int16_msg.empty_int16 = INT16_MIN;
  ASSERT_EQ(INT16_MIN, int16_msg.empty_int16);

  int16_msg.empty_int16 = INT16_MAX;
  ASSERT_EQ(INT16_MAX, int16_msg.empty_int16);

  int32_msg.empty_int32 = INT32_MIN;
  ASSERT_EQ(INT32_MIN, int32_msg.empty_int32);

  int32_msg.empty_int32 = INT32_MAX;
  ASSERT_EQ(INT32_MAX, int32_msg.empty_int32);

  int64_msg.empty_int64 = INT64_MIN;
  ASSERT_EQ(INT64_MIN, int64_msg.empty_int64);

  int64_msg.empty_int64 = INT64_MAX;
  ASSERT_EQ(INT64_MAX, int64_msg.empty_int64);

  uint8_msg.empty_uint8 = 0;
  ASSERT_EQ(0, uint8_msg.empty_uint8);

  uint8_msg.empty_uint8 = UINT8_MAX;
  ASSERT_EQ(UINT8_MAX, uint8_msg.empty_uint8);

  uint16_msg.empty_uint16 = 0;
  ASSERT_EQ(0, uint16_msg.empty_uint16);

  uint16_msg.empty_uint16 = UINT16_MAX;
  ASSERT_EQ(UINT16_MAX, uint16_msg.empty_uint16);

  uint32_msg.empty_uint32 = 0;
  ASSERT_EQ(0, uint32_msg.empty_uint32);

  uint32_msg.empty_uint32 = UINT32_MAX;
  ASSERT_EQ(UINT32_MAX, uint32_msg.empty_uint32);

  uint64_msg.empty_uint64 = 0;
  ASSERT_EQ(0, uint64_msg.empty_uint64);

  uint64_msg.empty_uint64 = UINT64_MAX;
  ASSERT_EQ(UINT64_MAX, uint64_msg.empty_uint64);
}

/**
 * Test message with simple primitive types using a default value initializer.
 */

TEST(Test_c_interfaces, test_primitives_default_value) {
  rosidl_generator_c__msg__PrimitiveValues * primitive_values = NULL;

  primitive_values = rosidl_generator_c__msg__PrimitiveValues__create();

  ASSERT_TRUE(NULL != primitive_values);

  ASSERT_EQ(true, primitive_values->def_bool_1);
  ASSERT_EQ(false, primitive_values->def_bool_2);
  ASSERT_EQ(66, primitive_values->def_byte);
  ASSERT_EQ(-66, primitive_values->def_char);
  ASSERT_EQ(1.125f, primitive_values->def_float32);
  ASSERT_EQ(1.125, primitive_values->def_float64);
  ASSERT_EQ(3, primitive_values->def_int8);
  ASSERT_EQ(6, primitive_values->def_int16);
  ASSERT_EQ(10, primitive_values->def_int32);
  ASSERT_EQ(15, primitive_values->def_int64);
  ASSERT_EQ(33, primitive_values->def_uint8);
  ASSERT_EQ(36, primitive_values->def_uint16);
  ASSERT_EQ(310, primitive_values->def_uint32);
  ASSERT_EQ(315, primitive_values->def_uint64);

  rosidl_generator_c__msg__PrimitiveValues__destroy(primitive_values);
}

/**
 * Test message with different string types.
 */
TEST(Test_c_interfaces, test_strings) {
  bool res = false;
  rosidl_generator_c__msg__Strings * strings = NULL;

  strings = rosidl_generator_c__msg__Strings__create();
  ASSERT_TRUE(NULL != strings);

  res = rosidl_generator_c__String__assign(&strings->empty_string, TEST_STRING);
  ASSERT_EQ(true, res);
  ASSERT_STREQ(TEST_STRING, strings->empty_string.data);
  ASSERT_STREQ("Hello world!", strings->def_string.data);
  // since upper-bound checking is not implemented yet, we restrict the string copying
  res = rosidl_generator_c__String__assignn(&strings->ub_string, TEST_STRING, 22);
  ASSERT_EQ(true, res);
  ASSERT_STREQ("Deep into that darknes", strings->ub_string.data);
  ASSERT_STREQ("Upper bounded string.", strings->ub_def_string.data);

  rosidl_generator_c__msg__Strings__destroy(strings);
}

/**
 * Test message with different unbounded array types
 */
TEST(Test_c_interfaces, test_primitives_unbounded_arrays) {
  bool res = false;
  int i;
  rosidl_generator_c__msg__PrimitivesUnboundedArrays * arrays = NULL;

  arrays = rosidl_generator_c__msg__PrimitivesUnboundedArrays__create();
  ASSERT_TRUE(NULL != arrays);

  // bool_array
  res = rosidl_generator_c__bool__Array__init(&arrays->bool_array, ARRAY_SIZE);
  ASSERT_EQ(true, res);
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
      ASSERT_EQ(true, arrays->bool_array.data[i]);
    } else {
      ASSERT_EQ(false, arrays->bool_array.data[i]);
    }
  }

  // byte_array
  res = rosidl_generator_c__byte__Array__init(&arrays->byte_array, ARRAY_SIZE);
  ASSERT_EQ(true, res);
  uint8_t test_array_byte[7] = {0, 57, 110, 177, 201, 240, 255};
  for (i = 0; i < ARRAY_SIZE; i++) {
    arrays->byte_array.data[i] = test_array_byte[i];
  }
  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    ASSERT_EQ(test_array_byte[i], arrays->byte_array.data[i]);
  }


  // char array
  res = rosidl_generator_c__char__Array__init(&arrays->char_array, ARRAY_SIZE);
  ASSERT_EQ(true, res);
  char test_array_char[7] = {'a', '5', '#', 'Z', '@', '-', ' '};
  for (i = 0; i < ARRAY_SIZE; i++) {
    arrays->char_array.data[i] = test_array_char[i];
  }
  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    ASSERT_EQ(test_array_char[i], arrays->char_array.data[i]);
  }

  // float32 array
  res = rosidl_generator_c__float32__Array__init(&arrays->float32_array, ARRAY_SIZE);
  ASSERT_EQ(true, res);
  float test_array_float32[7] =
  {-3.000001f, 22143.541325f, 6331.00432f, -214.66241f, 0.000001f, 1415555.12345f,
   -1.11154f};
  for (i = 0; i < ARRAY_SIZE; i++) {
    arrays->float32_array.data[i] = test_array_float32[i];
  }
  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    ASSERT_EQ(test_array_float32[i], arrays->float32_array.data[i]);
  }

  // float64 array
  res = rosidl_generator_c__float64__Array__init(&arrays->float64_array, ARRAY_SIZE);
  ASSERT_EQ(true, res);
  double test_array_float64[7] =
  {-120310.00843902140001, 22143.54483920141325, 6331.0048392104432,
   -214.62850432596241, 0.0000000000001, 1415555.128294031432345,
   -1.111184329208454};
  for (i = 0; i < ARRAY_SIZE; i++) {
    arrays->float64_array.data[i] = test_array_float64[i];
  }
  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    ASSERT_EQ(test_array_float64[i], arrays->float64_array.data[i]);
  }

  // int8 array
  res = rosidl_generator_c__int8__Array__init(&arrays->int8_array, ARRAY_SIZE);
  ASSERT_EQ(true, res);
  int8_t test_array_int8[7] = {-127, -55, -30, 0, 58, 100, 127};
  for (i = 0; i < ARRAY_SIZE; i++) {
    arrays->int8_array.data[i] = test_array_int8[i];
  }
  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    ASSERT_EQ(test_array_int8[i], arrays->int8_array.data[i]);
  }

  // int16 array
  res = rosidl_generator_c__int16__Array__init(&arrays->int16_array, ARRAY_SIZE);
  ASSERT_EQ(true, res);
  int16_t test_array_int16[7] =
  {-32767, -22222, -11111, 0, 11111, 22222, 32767};
  for (i = 0; i < ARRAY_SIZE; i++) {
    arrays->int16_array.data[i] = test_array_int16[i];
  }
  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    ASSERT_EQ(test_array_int16[i], arrays->int16_array.data[i]);
  }

  // int32 array
  res = rosidl_generator_c__int32__Array__init(&arrays->int32_array, ARRAY_SIZE);
  ASSERT_EQ(true, res);
  int32_t test_array_int32[7] =
  {INT32_MIN, INT32_MIN / 2, INT32_MIN / 4, 0L, INT32_MAX / 4, INT32_MAX / 2, INT32_MAX};
  for (i = 0; i < ARRAY_SIZE; i++) {
    arrays->int32_array.data[i] = test_array_int32[i];
  }
  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    ASSERT_EQ(test_array_int32[i], arrays->int32_array.data[i]);
  }

  // int64 array
  res = rosidl_generator_c__int64__Array__init(&arrays->int64_array, ARRAY_SIZE);
  ASSERT_EQ(true, res);
  int64_t test_array_int64[7] =
  {-9223372036854775807LL, -5000000000000000000LL, -1111111111111111111LL, 0,
   1111111111111111111LL, 5000000000000000000LL, 9223372036854775807LL};
  for (i = 0; i < ARRAY_SIZE; i++) {
    arrays->int64_array.data[i] = test_array_int64[i];
  }
  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    ASSERT_EQ(test_array_int64[i], arrays->int64_array.data[i]);
  }

  // uint8 array
  res = rosidl_generator_c__uint8__Array__init(&arrays->uint8_array, ARRAY_SIZE);
  ASSERT_EQ(true, res);
  uint8_t test_array_uint8[7] = {0, 5, 70, 128, 180, 220, 255};
  for (i = 0; i < ARRAY_SIZE; i++) {
    arrays->uint8_array.data[i] = test_array_uint8[i];
  }
  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    ASSERT_EQ(test_array_uint8[i], arrays->uint8_array.data[i]);
  }

  // uint16 array
  res = rosidl_generator_c__uint16__Array__init(&arrays->uint16_array, ARRAY_SIZE);
  ASSERT_EQ(true, res);
  uint16_t test_array_uint16[7] = {0U, 11111U, 22222U, 33333U, 44444U, 55555U, 65535U};
  for (i = 0; i < ARRAY_SIZE; i++) {
    arrays->uint16_array.data[i] = test_array_uint16[i];
  }
  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    ASSERT_EQ(test_array_uint16[i], arrays->uint16_array.data[i]);
  }

  // uint32 array
  res = rosidl_generator_c__uint32__Array__init(&arrays->uint32_array, ARRAY_SIZE);
  ASSERT_EQ(true, res);
  uint32_t test_array_uint32[7] =
  {0UL, 100UL, 2000UL, 30000UL, 444444UL, 567890123UL, 4294967295UL};
  for (i = 0; i < ARRAY_SIZE; i++) {
    arrays->uint32_array.data[i] = test_array_uint32[i];
  }
  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    ASSERT_EQ(test_array_uint32[i], arrays->uint32_array.data[i]);
  }

  // uint64 array
  res = rosidl_generator_c__uint64__Array__init(&arrays->uint64_array, ARRAY_SIZE);
  ASSERT_EQ(true, res);
  uint64_t test_array_uint64[7] =
  {0ULL, 10000ULL, 30000000ULL, 444444444ULL, 567890123456789ULL, 429496729578901234ULL,
   18446744073709551615ULL};
  for (i = 0; i < ARRAY_SIZE; i++) {
    arrays->uint64_array.data[i] = test_array_uint64[i];
  }
  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    ASSERT_EQ(test_array_uint64[i], arrays->uint64_array.data[i]);
  }

  rosidl_generator_c__msg__PrimitivesUnboundedArrays__destroy(arrays);
}

/**
 * Test message with different bounded array types
 */

TEST(Test_c_interfaces, test_primitives_bounded_arrays) {
  bool res = false;
  int i;
  rosidl_generator_c__msg__PrimitivesBoundedArrays * arrays = NULL;

  arrays = rosidl_generator_c__msg__PrimitivesBoundedArrays__create();
  ASSERT_TRUE(NULL != arrays);

  // bool_array
  res = rosidl_generator_c__bool__Array__init(&arrays->bool_array, ARRAY_SIZE);
  ASSERT_EQ(true, res);
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
      ASSERT_EQ(true, arrays->bool_array.data[i]);
    } else {
      ASSERT_EQ(false, arrays->bool_array.data[i]);
    }
  }

  // byte_array
  res = rosidl_generator_c__byte__Array__init(&arrays->byte_array, ARRAY_SIZE);
  ASSERT_EQ(true, res);
  uint8_t test_array_byte[8] = {0, 57, 110, 177, 201, 240, 255, 111};
  for (i = 0; i < ARRAY_SIZE; i++) {
    arrays->byte_array.data[i] = test_array_byte[i];
  }
  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    ASSERT_EQ(test_array_byte[i], arrays->byte_array.data[i]);
  }


  // char array
  res = rosidl_generator_c__char__Array__init(&arrays->char_array, ARRAY_SIZE);
  ASSERT_EQ(true, res);
  char test_array_char[7] = {'a', '5', '#', 'Z', '@', '-', ' '};
  for (i = 0; i < ARRAY_SIZE; i++) {
    arrays->char_array.data[i] = test_array_char[i];
  }

  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    ASSERT_EQ(test_array_char[i], arrays->char_array.data[i]);
  }

  // float32 array
  res = rosidl_generator_c__float32__Array__init(&arrays->float32_array, ARRAY_SIZE);
  ASSERT_EQ(true, res);
  float test_array_float32[7] =
  {-3.000001f, 22143.541325f, 6331.00432f, -214.66241f, 0.000001f, 1415555.12345f,
   -1.11154f};
  for (i = 0; i < ARRAY_SIZE; i++) {
    arrays->float32_array.data[i] = test_array_float32[i];
  }
  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    ASSERT_EQ(test_array_float32[i], arrays->float32_array.data[i]);
  }

  // float64 array
  res = rosidl_generator_c__float64__Array__init(&arrays->float64_array, ARRAY_SIZE);
  ASSERT_EQ(true, res);
  double test_array_float64[7] =
  {-120310.00843902140001, 22143.54483920141325, 6331.0048392104432,
   -214.62850432596241, 0.0000000000001, 1415555.128294031432345,
   -1.111184329208454};
  for (i = 0; i < ARRAY_SIZE; i++) {
    arrays->float64_array.data[i] = test_array_float64[i];
  }
  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    ASSERT_EQ(test_array_float64[i], arrays->float64_array.data[i]);
  }

  // int8 array
  res = rosidl_generator_c__int8__Array__init(&arrays->int8_array, ARRAY_SIZE);
  ASSERT_EQ(true, res);
  int8_t test_array_int8[7] = {-127, -55, -30, 0, 58, 100, 127};
  for (i = 0; i < ARRAY_SIZE; i++) {
    arrays->int8_array.data[i] = test_array_int8[i];
  }
  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    ASSERT_EQ(test_array_int8[i], arrays->int8_array.data[i]);
  }

  // int16 array
  res = rosidl_generator_c__int16__Array__init(&arrays->int16_array, ARRAY_SIZE);
  ASSERT_EQ(true, res);
  int16_t test_array_int16[7] =
  {-32767, -22222, -11111, 0, 11111, 22222, 32767};
  for (i = 0; i < ARRAY_SIZE; i++) {
    arrays->int16_array.data[i] = test_array_int16[i];
  }
  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    ASSERT_EQ(test_array_int16[i], arrays->int16_array.data[i]);
  }

  // int32 array
  res = rosidl_generator_c__int32__Array__init(&arrays->int32_array, ARRAY_SIZE);
  ASSERT_EQ(true, res);
  int32_t test_array_int32[7] =
  {INT32_MIN, INT32_MIN / 2, INT32_MIN / 4, 0L, INT32_MAX / 4, INT32_MAX / 2, INT32_MAX};
  for (i = 0; i < ARRAY_SIZE; i++) {
    arrays->int32_array.data[i] = test_array_int32[i];
  }
  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    ASSERT_EQ(test_array_int32[i], arrays->int32_array.data[i]);
  }

  // int64 array
  res = rosidl_generator_c__int64__Array__init(&arrays->int64_array, ARRAY_SIZE);
  ASSERT_EQ(true, res);
  int64_t test_array_int64[7] =
  {-9223372036854775807LL, -5000000000000000000LL, -1111111111111111111, 0,
   1111111111111111111LL, 5000000000000000000LL, 9223372036854775807LL};
  for (i = 0; i < ARRAY_SIZE; i++) {
    arrays->int64_array.data[i] = test_array_int64[i];
  }
  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    ASSERT_EQ(test_array_int64[i], arrays->int64_array.data[i]);
  }

  // uint8 array
  res = rosidl_generator_c__uint8__Array__init(&arrays->uint8_array, ARRAY_SIZE);
  ASSERT_EQ(true, res);
  uint8_t test_array_uint8[7] = {0, 5, 70, 128, 180, 220, 255};
  for (i = 0; i < ARRAY_SIZE; i++) {
    arrays->uint8_array.data[i] = test_array_uint8[i];
  }
  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    ASSERT_EQ(test_array_uint8[i], arrays->uint8_array.data[i]);
  }

  // uint16 array
  res = rosidl_generator_c__uint16__Array__init(&arrays->uint16_array, ARRAY_SIZE);
  ASSERT_EQ(true, res);
  uint16_t test_array_uint16[7] = {0U, 11111U, 22222U, 33333U, 44444U, 55555U, 65535U};
  for (i = 0; i < ARRAY_SIZE; i++) {
    arrays->uint16_array.data[i] = test_array_uint16[i];
  }
  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    ASSERT_EQ(test_array_uint16[i], arrays->uint16_array.data[i]);
  }

  // uint32 array
  res = rosidl_generator_c__uint32__Array__init(&arrays->uint32_array, ARRAY_SIZE);
  ASSERT_EQ(true, res);
  uint32_t test_array_uint32[7] =
  {0UL, 100UL, 2000UL, 30000UL, 444444UL, 567890123UL, 4294967295UL};
  for (i = 0; i < ARRAY_SIZE; i++) {
    arrays->uint32_array.data[i] = test_array_uint32[i];
  }
  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    ASSERT_EQ(test_array_uint32[i], arrays->uint32_array.data[i]);
  }

  // uint64 array
  res = rosidl_generator_c__uint64__Array__init(&arrays->uint64_array, ARRAY_SIZE);
  ASSERT_EQ(true, res);
  uint64_t test_array_uint64[7] =
  {0ULL, 10000ULL, 30000000ULL, 444444444ULL, 567890123456789ULL, 429496729578901234ULL,
   18446744073709551615ULL};
  for (i = 0; i < ARRAY_SIZE; i++) {
    arrays->uint64_array.data[i] = test_array_uint64[i];
  }
  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    ASSERT_EQ(test_array_uint64[i], arrays->uint64_array.data[i]);
  }

  rosidl_generator_c__msg__PrimitivesBoundedArrays__destroy(arrays);
}

/**
 * Test message with different static array types
 */

TEST(Test_c_interfaces, test_primitives_static_arrays) {
  int i;
  rosidl_generator_c__msg__PrimitivesStaticArrays * arrays = NULL;

  arrays = rosidl_generator_c__msg__PrimitivesStaticArrays__create();
  ASSERT_TRUE(NULL != arrays);

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
      ASSERT_EQ(true, arrays->bool_array[i]);
    } else {
      ASSERT_EQ(false, arrays->bool_array[i]);
    }
  }

  // byte_array
  uint8_t test_array_byte[ARRAY_SIZE] = {0, 57, 110, 177, 201, 240, 255};
  for (i = 0; i < ARRAY_SIZE; i++) {
    arrays->byte_array[i] = test_array_byte[i];
  }
  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    ASSERT_EQ(test_array_byte[i], arrays->byte_array[i]);
  }

  // char array
  char test_array_char[ARRAY_SIZE] = {'a', '5', '#', 'Z', '@', '-', ' '};
  for (i = 0; i < ARRAY_SIZE; i++) {
    arrays->char_array[i] = test_array_char[i];
  }

  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    ASSERT_EQ(test_array_char[i], arrays->char_array[i]);
  }

  // float32 array
  float test_array_float32[ARRAY_SIZE] =
  {-3.000001f, 22143.541325f, 6331.00432f, -214.66241f, 0.000001f, 1415555.12345f,
   -1.11154f};
  for (i = 0; i < ARRAY_SIZE; i++) {
    arrays->float32_array[i] = test_array_float32[i];
  }
  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    ASSERT_EQ(test_array_float32[i], arrays->float32_array[i]);
  }

  // float64 array
  double test_array_float64[ARRAY_SIZE] =
  {-120310.00843902140001, 22143.54483920141325, 6331.0048392104432,
   -214.62850432596241, 0.0000000000001, 1415555.128294031432345,
   -1.111184329208454};
  for (i = 0; i < ARRAY_SIZE; i++) {
    arrays->float64_array[i] = test_array_float64[i];
  }
  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    ASSERT_EQ(test_array_float64[i], arrays->float64_array[i]);
  }

  // int8 array
  int8_t test_array_int8[ARRAY_SIZE] = {-127, -55, -30, 0, 58, 100, 127};
  for (i = 0; i < ARRAY_SIZE; i++) {
    arrays->int8_array[i] = test_array_int8[i];
  }
  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    ASSERT_EQ(test_array_int8[i], arrays->int8_array[i]);
  }

  // int16 array
  int16_t test_array_int16[ARRAY_SIZE] =
  {-32767, -22222, -11111, 0, 11111, 22222, 32767};
  for (i = 0; i < ARRAY_SIZE; i++) {
    arrays->int16_array[i] = test_array_int16[i];
  }
  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    ASSERT_EQ(test_array_int16[i], arrays->int16_array[i]);
  }

  // int32 array
  int32_t test_array_int32[ARRAY_SIZE] =
  {INT32_MIN, INT32_MIN / 2, INT32_MIN / 4, 0L, INT32_MAX / 4, INT32_MAX / 2, INT32_MAX};
  for (i = 0; i < ARRAY_SIZE; i++) {
    arrays->int32_array[i] = test_array_int32[i];
  }
  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    ASSERT_EQ(test_array_int32[i], arrays->int32_array[i]);
  }

  // int64 array
  int64_t test_array_int64[ARRAY_SIZE] =
  {-9223372036854775807LL, -5000000000000000000LL, -1111111111111111111LL, 0,
   1111111111111111111ULL, 5000000000000000000ULL, 9223372036854775807ULL};
  for (i = 0; i < ARRAY_SIZE; i++) {
    arrays->int64_array[i] = test_array_int64[i];
  }
  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    ASSERT_EQ(test_array_int64[i], arrays->int64_array[i]);
  }

  // uint8 array
  uint8_t test_array_uint8[ARRAY_SIZE] = {0, 5, 70, 128, 180, 220, 255};
  for (i = 0; i < ARRAY_SIZE; i++) {
    arrays->uint8_array[i] = test_array_uint8[i];
  }
  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    ASSERT_EQ(test_array_uint8[i], arrays->uint8_array[i]);
  }

  // uint16 array
  uint16_t test_array_uint16[ARRAY_SIZE] = {0U, 11111U, 22222U, 33333U, 44444U, 55555U, 65535U};
  for (i = 0; i < ARRAY_SIZE; i++) {
    arrays->uint16_array[i] = test_array_uint16[i];
  }
  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    ASSERT_EQ(test_array_uint16[i], arrays->uint16_array[i]);
  }

  // uint32 array
  uint32_t test_array_uint32[ARRAY_SIZE] =
  {0UL, 100UL, 2000UL, 30000UL, 444444UL, 567890123UL, 4294967295UL};
  for (i = 0; i < ARRAY_SIZE; i++) {
    arrays->uint32_array[i] = test_array_uint32[i];
  }
  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    ASSERT_EQ(test_array_uint32[i], arrays->uint32_array[i]);
  }

  // uint64 array
  uint64_t test_array_uint64[ARRAY_SIZE] =
  {0ULL, 10000ULL, 30000000ULL, 444444444ULL, 567890123456789ULL, 429496729578901234ULL,
   18446744073709551615ULL};
  for (i = 0; i < ARRAY_SIZE; i++) {
    arrays->uint64_array[i] = test_array_uint64[i];
  }
  // test values
  for (i = 0; i < ARRAY_SIZE; i++) {
    ASSERT_EQ(test_array_uint64[i], arrays->uint64_array[i]);
  }

  rosidl_generator_c__msg__PrimitivesStaticArrays__destroy(arrays);
}

/**
 * Test message with sub-messages types
 */
TEST(Test_c_interfaces, test_submessages) {
  int i;
  bool res;
  rosidl_generator_c__msg__Wire * wire_msg = rosidl_generator_c__msg__Wire__create();

  for (i = 0; i < 3; i++) {
    res = rosidl_generator_c__String__assign(&wire_msg->cablegram1[i].text, TEST_STRING);
    ASSERT_EQ(true, res);
    wire_msg->cablegram1[i].number = 3.1415f;
  }
  res = rosidl_generator_c__String__assign(&wire_msg->cablegram2.text_array[0], "Test 1");
  ASSERT_EQ(true, res);
  res = rosidl_generator_c__String__assign(&wire_msg->cablegram2.text_array[1], "Test 2");
  ASSERT_EQ(true, res);
  res = rosidl_generator_c__String__assign(&wire_msg->cablegram2.text_array[2], "Test 3");
  ASSERT_EQ(true, res);
  wire_msg->cablegram2.number_array[0] = 3.1f;
  wire_msg->cablegram2.number_array[1] = 3.14f;
  wire_msg->cablegram2.number_array[2] = 3.141f;

  for (i = 0; i < 3; i++) {
    ASSERT_STREQ(TEST_STRING, wire_msg->cablegram1[i].text.data);
    ASSERT_EQ(3.1415f, wire_msg->cablegram1[i].number);
  }
  ASSERT_STREQ("Test 1", wire_msg->cablegram2.text_array[0].data);
  ASSERT_STREQ("Test 2", wire_msg->cablegram2.text_array[1].data);
  ASSERT_STREQ("Test 3", wire_msg->cablegram2.text_array[2].data);
  ASSERT_EQ(3.1f, wire_msg->cablegram2.number_array[0]);
  ASSERT_EQ(3.14f, wire_msg->cablegram2.number_array[1]);
  ASSERT_EQ(3.141f, wire_msg->cablegram2.number_array[2]);

  rosidl_generator_c__msg__Wire__destroy(wire_msg);
}
