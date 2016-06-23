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

#include <gtest/gtest.h>
#include <iostream>
#include <climits>
#include <cfloat>
#include <cstdint>
#include <string>
#include <vector>
#include <algorithm>
#include "test_array_generator.hpp"

#include "rosidl_generator_cpp/msg/empty.hpp"

#include "rosidl_generator_cpp/msg/bounded_array_bounded.hpp"
#include "rosidl_generator_cpp/msg/bounded_array_static.hpp"
#include "rosidl_generator_cpp/msg/bounded_array_unbounded.hpp"

#include "rosidl_generator_cpp/msg/primitive_static_arrays.hpp"

#include "rosidl_generator_cpp/msg/primitives_bounded.hpp"
#include "rosidl_generator_cpp/msg/primitives_static.hpp"
#include "rosidl_generator_cpp/msg/primitives_unbounded.hpp"

#include "rosidl_generator_cpp/msg/static_array_bounded.hpp"
#include "rosidl_generator_cpp/msg/static_array_static.hpp"
#include "rosidl_generator_cpp/msg/static_array_unbounded.hpp"

#include "rosidl_generator_cpp/msg/unbounded_array_bounded.hpp"
#include "rosidl_generator_cpp/msg/unbounded_array_static.hpp"
#include "rosidl_generator_cpp/msg/unbounded_array_unbounded.hpp"

#define PRIMITIVES_ARRAY_SIZE 10
#define SUBMESSAGE_ARRAY_SIZE 3

TEST(Test_rosidl_generator_traits, has_fixed_size) {
  static_assert(
    rosidl_generator_traits::has_fixed_size<rosidl_generator_cpp::msg::Empty>::value,
    "Empty::has_fixed_size is false");

  static_assert(
    rosidl_generator_traits::has_fixed_size<rosidl_generator_cpp::msg::PrimitivesStatic>::value,
    "PrimitivesStatic::has_fixed_size is false");

  static_assert(
    !rosidl_generator_traits::has_fixed_size<rosidl_generator_cpp::msg::PrimitivesBounded>::value,
    "PrimitivesBounded::has_fixed_size is true");

  static_assert(
    !rosidl_generator_traits::has_fixed_size<rosidl_generator_cpp::msg::PrimitivesUnbounded>::value,
    "PrimitivesUnbounded::has_fixed_size is true");

  static_assert(
    rosidl_generator_traits::has_fixed_size<
      rosidl_generator_cpp::msg::PrimitiveStaticArrays>::value,
    "PrimitivesStaticArray::has_fixed_size is false");

  static_assert(
    rosidl_generator_traits::has_fixed_size<rosidl_generator_cpp::msg::StaticArrayStatic>::value,
    "StaticArrayStatic::has_fixed_size is false");

  static_assert(
    !rosidl_generator_traits::has_fixed_size<rosidl_generator_cpp::msg::StaticArrayBounded>::value,
    "StaticArrayBounded::has_fixed_size is true");

  static_assert(
    !rosidl_generator_traits::has_fixed_size<
      rosidl_generator_cpp::msg::StaticArrayUnbounded>::value,
    "StaticArrayUnbounded::has_fixed_size is true");

  static_assert(
    !rosidl_generator_traits::has_fixed_size<rosidl_generator_cpp::msg::BoundedArrayStatic>::value,
    "BoundedArrayStatic::has_fixed_size is true");

  static_assert(
    !rosidl_generator_traits::has_fixed_size<rosidl_generator_cpp::msg::BoundedArrayBounded>::value,
    "BoundedArrayBounded::has_fixed_size is true");

  static_assert(
    !rosidl_generator_traits::has_fixed_size<
      rosidl_generator_cpp::msg::BoundedArrayUnbounded>::value,
    "BoundedArrayUnbounded::has_fixed_size is true");

  static_assert(
    !rosidl_generator_traits::has_fixed_size<
      rosidl_generator_cpp::msg::UnboundedArrayStatic>::value,
    "UnboundedArrayStatic::has_fixed_size is true");

  static_assert(
    !rosidl_generator_traits::has_fixed_size<
      rosidl_generator_cpp::msg::UnboundedArrayBounded>::value,
    "UnboundedArrayBounded::has_fixed_size is true");

  static_assert(
    !rosidl_generator_traits::has_fixed_size<
      rosidl_generator_cpp::msg::UnboundedArrayUnbounded>::value,
    "UnboundedArrayUnbounded::has_fixed_size is true");
}

#define TEST_PRIMITIVE_FIELD_ASSIGNMENT(Message, FieldName, InitialValue, FinalValue) \
  Message.FieldName = InitialValue; \
  ASSERT_EQ(InitialValue, Message.FieldName); \
  Message.FieldName = FinalValue; \
  ASSERT_EQ(FinalValue, Message.FieldName);

void test_message_primitives_static(rosidl_generator_cpp::msg::PrimitivesStatic message)
{
// workaround for https://github.com/google/googletest/issues/322
#ifdef __linux__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion-null"
#endif
  TEST_PRIMITIVE_FIELD_ASSIGNMENT(message, bool_value, false, true)
#ifdef __linux__
#pragma GCC diagnostic pop
#endif
  TEST_PRIMITIVE_FIELD_ASSIGNMENT(message, byte_value, 0, 255)
  TEST_PRIMITIVE_FIELD_ASSIGNMENT(message, char_value, CHAR_MIN, CHAR_MAX)
  TEST_PRIMITIVE_FIELD_ASSIGNMENT(message, float32_value, FLT_MIN, FLT_MAX)
  TEST_PRIMITIVE_FIELD_ASSIGNMENT(message, float64_value, DBL_MIN, DBL_MAX)
  TEST_PRIMITIVE_FIELD_ASSIGNMENT(message, int8_value, INT8_MIN, INT8_MAX)
  TEST_PRIMITIVE_FIELD_ASSIGNMENT(message, uint8_value, 0, UINT8_MAX)
  TEST_PRIMITIVE_FIELD_ASSIGNMENT(message, int16_value, INT16_MIN, INT16_MAX)
  TEST_PRIMITIVE_FIELD_ASSIGNMENT(message, uint16_value, 0, UINT16_MAX)
  TEST_PRIMITIVE_FIELD_ASSIGNMENT(message, int32_value, INT32_MIN, INT32_MAX)
  TEST_PRIMITIVE_FIELD_ASSIGNMENT(message, uint32_value, 0ul, UINT32_MAX)
  TEST_PRIMITIVE_FIELD_ASSIGNMENT(message, int64_value, INT64_MIN, INT64_MAX)
  TEST_PRIMITIVE_FIELD_ASSIGNMENT(message, uint64_value, 0ull, UINT64_MAX)
}

#define TEST_BOUNDED_ARRAY_PRIMITIVE( \
    Message, FieldName, PrimitiveType, ArraySize, MinVal, MaxVal) \
  rosidl_generator_cpp::BoundedVector<PrimitiveType, ArraySize> pattern_ ## FieldName; \
  Message.FieldName.resize(ArraySize); \
  pattern_ ## FieldName.resize(ArraySize); \
  test_vector_fill<decltype(pattern_ ## FieldName)>( \
    &pattern_ ## FieldName, ArraySize, MinVal, MaxVal); \
  std::copy_n(pattern_ ## FieldName.begin(), Message.FieldName.size(), Message.FieldName.begin()); \
  ASSERT_EQ(pattern_ ## FieldName, Message.FieldName); \

void test_message_primitives_bounded(rosidl_generator_cpp::msg::PrimitivesBounded message)
{
  TEST_BOUNDED_ARRAY_PRIMITIVE(message, bool_value, bool, PRIMITIVES_ARRAY_SIZE, \
    false, true)
  TEST_BOUNDED_ARRAY_PRIMITIVE(message, char_value, char, PRIMITIVES_ARRAY_SIZE, \
    CHAR_MIN, CHAR_MAX)
  TEST_BOUNDED_ARRAY_PRIMITIVE(message, byte_value, uint8_t, PRIMITIVES_ARRAY_SIZE, \
    0, UINT8_MAX)
  TEST_BOUNDED_ARRAY_PRIMITIVE(message, float32_value, float, PRIMITIVES_ARRAY_SIZE, \
    FLT_MIN, FLT_MAX)
  TEST_BOUNDED_ARRAY_PRIMITIVE(message, float64_value, double, PRIMITIVES_ARRAY_SIZE, \
    DBL_MIN, DBL_MAX)
  TEST_BOUNDED_ARRAY_PRIMITIVE(message, int8_value, int8_t, PRIMITIVES_ARRAY_SIZE, \
    INT8_MIN, INT8_MAX)
  TEST_BOUNDED_ARRAY_PRIMITIVE(message, uint8_value, uint8_t, PRIMITIVES_ARRAY_SIZE, \
    0, UINT8_MAX)
  TEST_BOUNDED_ARRAY_PRIMITIVE(message, int16_value, int16_t, PRIMITIVES_ARRAY_SIZE, \
    INT16_MIN, INT16_MAX)
  TEST_BOUNDED_ARRAY_PRIMITIVE(message, uint16_value, uint16_t, PRIMITIVES_ARRAY_SIZE, \
    0, UINT16_MAX)
  TEST_BOUNDED_ARRAY_PRIMITIVE(message, int32_value, int32_t, PRIMITIVES_ARRAY_SIZE, \
    INT32_MIN, INT32_MAX)
  TEST_BOUNDED_ARRAY_PRIMITIVE(message, uint32_value, uint32_t, PRIMITIVES_ARRAY_SIZE, \
    0, UINT32_MAX)
  TEST_BOUNDED_ARRAY_PRIMITIVE(message, int64_value, int64_t, PRIMITIVES_ARRAY_SIZE, \
    INT64_MIN, INT64_MAX)
  TEST_BOUNDED_ARRAY_PRIMITIVE(message, uint64_value, uint64_t, PRIMITIVES_ARRAY_SIZE, \
    0, UINT64_MAX)
  // Arrays of strings not supported yet
  TEST_PRIMITIVE_FIELD_ASSIGNMENT(message, string_value, "", "Deep into that darkness peering")
}

#define TEST_UNBOUNDED_ARRAY_PRIMITIVE( \
    Message, FieldName, PrimitiveType, ArraySize, MinVal, MaxVal) \
  std::vector<PrimitiveType> pattern_ ## FieldName(ArraySize); \
  pattern_ ## FieldName.resize(ArraySize); \
  test_vector_fill<decltype(pattern_ ## FieldName)>( \
    &pattern_ ## FieldName, ArraySize, MinVal, MaxVal); \
  Message.FieldName.resize(ArraySize); \
  std::copy_n(pattern_ ## FieldName.begin(), ArraySize, Message.FieldName.begin()); \
  ASSERT_EQ(pattern_ ## FieldName, Message.FieldName); \

void test_message_primitives_unbounded(rosidl_generator_cpp::msg::PrimitivesUnbounded message)
{
  TEST_UNBOUNDED_ARRAY_PRIMITIVE(message, bool_value, bool, PRIMITIVES_ARRAY_SIZE, \
    false, true)
  TEST_UNBOUNDED_ARRAY_PRIMITIVE(message, char_value, char, PRIMITIVES_ARRAY_SIZE, \
    CHAR_MIN, CHAR_MAX)
  TEST_UNBOUNDED_ARRAY_PRIMITIVE(message, byte_value, uint8_t, PRIMITIVES_ARRAY_SIZE, \
    0, UINT8_MAX)
  TEST_UNBOUNDED_ARRAY_PRIMITIVE(message, float32_value, float, PRIMITIVES_ARRAY_SIZE, \
    FLT_MIN, FLT_MAX)
  TEST_UNBOUNDED_ARRAY_PRIMITIVE(message, float64_value, double, PRIMITIVES_ARRAY_SIZE, \
    DBL_MIN, DBL_MAX)
  TEST_UNBOUNDED_ARRAY_PRIMITIVE(message, int8_value, int8_t, PRIMITIVES_ARRAY_SIZE, \
    INT8_MIN, INT8_MAX)
  TEST_UNBOUNDED_ARRAY_PRIMITIVE(message, uint8_value, uint8_t, PRIMITIVES_ARRAY_SIZE, \
    0, UINT8_MAX)
  TEST_UNBOUNDED_ARRAY_PRIMITIVE(message, int16_value, int16_t, PRIMITIVES_ARRAY_SIZE, \
    INT16_MIN, INT16_MAX)
  TEST_UNBOUNDED_ARRAY_PRIMITIVE(message, uint16_value, uint16_t, PRIMITIVES_ARRAY_SIZE, \
    0, UINT16_MAX)
  TEST_UNBOUNDED_ARRAY_PRIMITIVE(message, int32_value, int32_t, PRIMITIVES_ARRAY_SIZE, \
    INT32_MIN, INT32_MAX)
  TEST_UNBOUNDED_ARRAY_PRIMITIVE(message, uint32_value, uint32_t, PRIMITIVES_ARRAY_SIZE, \
    0, UINT32_MAX)
  TEST_UNBOUNDED_ARRAY_PRIMITIVE(message, int64_value, int64_t, PRIMITIVES_ARRAY_SIZE, \
    INT64_MIN, INT64_MAX)
  TEST_UNBOUNDED_ARRAY_PRIMITIVE(message, uint64_value, uint64_t, PRIMITIVES_ARRAY_SIZE, \
    0, UINT64_MAX)
  // Arrays of strings not supported yet
  TEST_PRIMITIVE_FIELD_ASSIGNMENT(message, string_value, "", "Deep into that darkness peering")
}

// Primitives static
TEST(Test_messages, primitives_static) {
  rosidl_generator_cpp::msg::PrimitivesStatic message;
  test_message_primitives_static(message);
}

// Primitives bounded arrays
TEST(Test_messages, primitives_bounded) {
  rosidl_generator_cpp::msg::PrimitivesBounded message;
  test_message_primitives_bounded(message);
}

// Primitives unbounded arrays
TEST(Test_messages, primitives_unbounded) {
  rosidl_generator_cpp::msg::PrimitivesUnbounded message;
  test_message_primitives_unbounded(message);
}

// Static array of a submessage of static primitives
TEST(Test_messages, static_array_static) {
  rosidl_generator_cpp::msg::StaticArrayStatic message;
  for (int i = 0; i < SUBMESSAGE_ARRAY_SIZE; i++) {
    test_message_primitives_static(message.primitive_values[i]);
  }
}

// Static array of a submessage of bounded array of primitives
TEST(Test_messages, static_array_bounded) {
  rosidl_generator_cpp::msg::StaticArrayBounded message;
  for (int i = 0; i < SUBMESSAGE_ARRAY_SIZE; i++) {
    test_message_primitives_bounded(message.primitive_values[i]);
  }
}

// Static array of a submessage of unbounded array of primitives
TEST(Test_messages, static_array_unbounded) {
  rosidl_generator_cpp::msg::StaticArrayUnbounded message;
  for (int i = 0; i < SUBMESSAGE_ARRAY_SIZE; i++) {
    test_message_primitives_unbounded(message.primitive_values[i]);
  }
}

// Bounded array of a submessage of static primitive
TEST(Test_messages, bounded_array_static) {
  rosidl_generator_cpp::msg::BoundedArrayStatic message;
  message.primitive_values.resize(SUBMESSAGE_ARRAY_SIZE);
  for (int i = 0; i < SUBMESSAGE_ARRAY_SIZE; i++) {
    test_message_primitives_static(message.primitive_values[i]);
  }
}

// Bounded array of a submessage of bounded array of primitives
TEST(Test_messages, bounded_array_bounded) {
  rosidl_generator_cpp::msg::BoundedArrayBounded message;
  message.primitive_values.resize(SUBMESSAGE_ARRAY_SIZE);
  for (int i = 0; i < SUBMESSAGE_ARRAY_SIZE; i++) {
    test_message_primitives_bounded(message.primitive_values[i]);
  }
}

// Bounded array of a submessage of unbounded array of primitives
TEST(Test_messages, bounded_array_unbounded) {
  rosidl_generator_cpp::msg::BoundedArrayUnbounded message;
  message.primitive_values.resize(SUBMESSAGE_ARRAY_SIZE);
  for (int i = 0; i < SUBMESSAGE_ARRAY_SIZE; i++) {
    test_message_primitives_unbounded(message.primitive_values[i]);
  }
}

// Unbounded array of a submessage of static primitives
TEST(Test_messages, unbounded_array_static) {
  rosidl_generator_cpp::msg::UnboundedArrayStatic message;
  message.primitive_values.resize(SUBMESSAGE_ARRAY_SIZE);
  for (int i = 0; i < SUBMESSAGE_ARRAY_SIZE; i++) {
    test_message_primitives_static(message.primitive_values[i]);
  }
}

// Unbounded array of a submessage of bounded primitive
TEST(Test_messages, unbounded_array_bounded) {
  rosidl_generator_cpp::msg::UnboundedArrayBounded message;
  message.primitive_values.resize(SUBMESSAGE_ARRAY_SIZE);
  for (int i = 0; i < SUBMESSAGE_ARRAY_SIZE; i++) {
    test_message_primitives_bounded(message.primitive_values[i]);
  }
}

// Unbounded array of a submessage of unbounded array of primitives
TEST(Test_messages, unbounded_array_unbounded) {
  rosidl_generator_cpp::msg::UnboundedArrayUnbounded message;
  message.primitive_values.resize(SUBMESSAGE_ARRAY_SIZE);
  for (int i = 0; i < SUBMESSAGE_ARRAY_SIZE; i++) {
    test_message_primitives_unbounded(message.primitive_values[i]);
  }
}
