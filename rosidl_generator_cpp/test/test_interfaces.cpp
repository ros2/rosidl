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
#include "rosidl_generator_cpp/msg/primitives_constants.hpp"
#include "rosidl_generator_cpp/msg/primitives_default.hpp"
#include "rosidl_generator_cpp/msg/primitives_static.hpp"
#include "rosidl_generator_cpp/msg/primitives_unbounded.hpp"

#include "rosidl_generator_cpp/msg/static_array_bounded.hpp"
#include "rosidl_generator_cpp/msg/static_array_static.hpp"
#include "rosidl_generator_cpp/msg/static_array_unbounded.hpp"

#include "rosidl_generator_cpp/msg/string.hpp"
#include "rosidl_generator_cpp/msg/string_bounded.hpp"
#include "rosidl_generator_cpp/msg/string_array_static.hpp"
#include "rosidl_generator_cpp/msg/string_arrays.hpp"

#include "rosidl_generator_cpp/msg/unbounded_array_bounded.hpp"
#include "rosidl_generator_cpp/msg/unbounded_array_static.hpp"
#include "rosidl_generator_cpp/msg/unbounded_array_unbounded.hpp"

#include "rosidl_generator_cpp/msg/w_string.hpp"

#define PRIMITIVES_ARRAY_SIZE 10
#define BOUNDED_STRING_LENGTH 10
#define SUBMESSAGE_ARRAY_SIZE 3

TEST(Test_rosidl_generator_traits, has_fixed_size) {
  static_assert(
    rosidl_generator_traits::has_fixed_size<rosidl_generator_cpp::msg::Empty>::value,
    "Empty::has_fixed_size is false");

  static_assert(
    rosidl_generator_traits::has_fixed_size<rosidl_generator_cpp::msg::PrimitivesConstants>::value,
    "PrimitivesConstants::has_fixed_size is false");

  static_assert(
    !rosidl_generator_traits::has_fixed_size<rosidl_generator_cpp::msg::PrimitivesDefault>::value,
    "PrimitivesDefault::has_fixed_size is true");

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
    !rosidl_generator_traits::has_fixed_size<rosidl_generator_cpp::msg::String>::value,
    "String::has_fixed_size is true");

  static_assert(
    !rosidl_generator_traits::has_fixed_size<rosidl_generator_cpp::msg::StringBounded>::value,
    "StringBounded::has_fixed_size is true");

  static_assert(
    !rosidl_generator_traits::has_fixed_size<
      rosidl_generator_cpp::msg::StringArrayStatic>::value,
    "StringArrayStatic::has_fixed_size is true");

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

  static_assert(
    !rosidl_generator_traits::has_fixed_size<rosidl_generator_cpp::msg::WString>::value,
    "WString::has_fixed_size is true");
}

TEST(Test_rosidl_generator_traits, has_bounded_size) {
  static_assert(
    rosidl_generator_traits::has_bounded_size<rosidl_generator_cpp::msg::Empty>::value,
    "Empty::has_bounded_size is false");

  static_assert(
    rosidl_generator_traits::has_bounded_size<
      rosidl_generator_cpp::msg::PrimitivesConstants>::value,
    "PrimitivesConstants::has_bounded_size is false");

  static_assert(
    !rosidl_generator_traits::has_bounded_size<rosidl_generator_cpp::msg::PrimitivesDefault>::value,
    "PrimitivesDefault::has_bounded_size is true");

  static_assert(
    rosidl_generator_traits::has_bounded_size<rosidl_generator_cpp::msg::PrimitivesStatic>::value,
    "PrimitivesStatic::has_bounded_size is false");

  static_assert(
    rosidl_generator_traits::has_bounded_size<rosidl_generator_cpp::msg::PrimitivesBounded>::value,
    "PrimitivesBounded::has_bounded_size is false");

  static_assert(
    !rosidl_generator_traits::has_bounded_size<
      rosidl_generator_cpp::msg::PrimitivesUnbounded>::value,
    "PrimitivesUnbounded::has_bounded_size is true");

  static_assert(
    rosidl_generator_traits::has_bounded_size<
      rosidl_generator_cpp::msg::PrimitiveStaticArrays>::value,
    "PrimitivesStaticArray::has_bounded_size is false");

  static_assert(
    rosidl_generator_traits::has_bounded_size<rosidl_generator_cpp::msg::StaticArrayStatic>::value,
    "StaticArrayStatic::has_bounded_size is false");

  static_assert(
    rosidl_generator_traits::has_bounded_size<rosidl_generator_cpp::msg::StaticArrayBounded>::value,
    "StaticArrayBounded::has_bounded_size is false");

  static_assert(
    !rosidl_generator_traits::has_bounded_size<
      rosidl_generator_cpp::msg::StaticArrayUnbounded>::value,
    "StaticArrayUnbounded::has_bounded_size is true");

  static_assert(
    !rosidl_generator_traits::has_bounded_size<rosidl_generator_cpp::msg::String>::value,
    "String::has_bounded_size is true");

  static_assert(
    rosidl_generator_traits::has_bounded_size<rosidl_generator_cpp::msg::StringBounded>::value,
    "StringBounded::has_bounded_size is false");

  static_assert(
    !rosidl_generator_traits::has_bounded_size<
      rosidl_generator_cpp::msg::StringArrayStatic>::value,
    "StringArrayStatic::has_bounded_size is true");

  static_assert(
    rosidl_generator_traits::has_bounded_size<rosidl_generator_cpp::msg::BoundedArrayStatic>::value,
    "BoundedArrayStatic::has_bounded_size is false");

  static_assert(
    rosidl_generator_traits::has_bounded_size<
      rosidl_generator_cpp::msg::BoundedArrayBounded>::value,
    "BoundedArrayBounded::has_bounded_size is false");

  static_assert(
    !rosidl_generator_traits::has_bounded_size<
      rosidl_generator_cpp::msg::BoundedArrayUnbounded>::value,
    "BoundedArrayUnbounded::has_bounded_size is true");

  static_assert(
    !rosidl_generator_traits::has_bounded_size<
      rosidl_generator_cpp::msg::UnboundedArrayStatic>::value,
    "UnboundedArrayStatic::has_bounded_size is true");

  static_assert(
    !rosidl_generator_traits::has_bounded_size<
      rosidl_generator_cpp::msg::UnboundedArrayBounded>::value,
    "UnboundedArrayBounded::has_bounded_size is true");

  static_assert(
    !rosidl_generator_traits::has_bounded_size<
      rosidl_generator_cpp::msg::UnboundedArrayUnbounded>::value,
    "UnboundedArrayUnbounded::has_bounded_size is true");

  static_assert(
    !rosidl_generator_traits::has_bounded_size<rosidl_generator_cpp::msg::WString>::value,
    "WString::has_bounded_size is true");
}

#define TEST_PRIMITIVE_FIELD_ASSIGNMENT(Message, FieldName, InitialValue, FinalValue) \
  Message.FieldName = InitialValue; \
  ASSERT_EQ(InitialValue, Message.FieldName); \
  Message.FieldName = FinalValue; \
  ASSERT_EQ(FinalValue, Message.FieldName);

#define TEST_STRING_FIELD_ASSIGNMENT(Message, FieldName, InitialValue, FinalValue) \
  Message.FieldName = InitialValue; \
  ASSERT_STREQ(InitialValue, Message.FieldName.c_str()); \
  Message.FieldName = FinalValue; \
  ASSERT_STREQ(FinalValue, Message.FieldName.c_str());

#define TEST_WSTRING_FIELD_ASSIGNMENT(Message, FieldName, InitialValue, FinalValue) \
  Message.FieldName = InitialValue; \
  Message.FieldName = FinalValue;

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
  TEST_PRIMITIVE_FIELD_ASSIGNMENT(message, char_value, 0, UINT8_MAX)
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

#define TEST_BOUNDED_ARRAY_STRING( \
    Message, FieldName, PrimitiveType, ArraySize, MinVal, MaxVal, MinLength, MaxLength) \
  rosidl_generator_cpp::BoundedVector<PrimitiveType, ArraySize> pattern_ ## FieldName; \
  Message.FieldName.resize(ArraySize); \
  pattern_ ## FieldName.resize(ArraySize); \
  test_vector_fill<decltype(pattern_ ## FieldName)>( \
    &pattern_ ## FieldName, ArraySize, MinVal, MaxVal, MinLength, MaxLength); \
  std::copy_n(pattern_ ## FieldName.begin(), Message.FieldName.size(), Message.FieldName.begin()); \
  ASSERT_EQ(pattern_ ## FieldName, Message.FieldName); \

void test_message_primitives_bounded(rosidl_generator_cpp::msg::PrimitivesBounded message)
{
  TEST_BOUNDED_ARRAY_PRIMITIVE(message, bool_value, bool, PRIMITIVES_ARRAY_SIZE, \
    false, true)
  TEST_BOUNDED_ARRAY_PRIMITIVE(message, char_value, unsigned char, PRIMITIVES_ARRAY_SIZE, \
    0, UINT8_MAX)
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
  TEST_BOUNDED_ARRAY_STRING(message, string_values, std::string, PRIMITIVES_ARRAY_SIZE, \
    0, UINT32_MAX, 0, BOUNDED_STRING_LENGTH)
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

#define TEST_UNBOUNDED_ARRAY_STRING( \
    Message, FieldName, PrimitiveType, ArraySize, MinVal, MaxVal, MinLength, MaxLength) \
  std::vector<PrimitiveType> pattern_ ## FieldName; \
  Message.FieldName.resize(ArraySize); \
  pattern_ ## FieldName.resize(ArraySize); \
  test_vector_fill<decltype(pattern_ ## FieldName)>( \
    &pattern_ ## FieldName, ArraySize, MinVal, MaxVal, MinLength, MaxLength); \
  std::copy_n(pattern_ ## FieldName.begin(), Message.FieldName.size(), Message.FieldName.begin()); \
  ASSERT_EQ(pattern_ ## FieldName, Message.FieldName); \

void test_message_primitives_unbounded(rosidl_generator_cpp::msg::PrimitivesUnbounded message)
{
  TEST_UNBOUNDED_ARRAY_PRIMITIVE(message, bool_value, bool, PRIMITIVES_ARRAY_SIZE, \
    false, true)
  TEST_UNBOUNDED_ARRAY_PRIMITIVE(message, char_value, unsigned char, PRIMITIVES_ARRAY_SIZE, \
    0, UINT8_MAX)
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
  TEST_UNBOUNDED_ARRAY_STRING(message, string_value, std::string, PRIMITIVES_ARRAY_SIZE, \
    0, UINT32_MAX, 0, UINT16_MAX)
}

#define TEST_STATIC_ARRAY_PRIMITIVE( \
    Message, FieldName, PrimitiveType, ArraySize, MinVal, MaxVal) \
  std::array<PrimitiveType, ArraySize> pattern_ ## FieldName; \
  test_vector_fill<decltype(pattern_ ## FieldName)>( \
    &pattern_ ## FieldName, ArraySize, MinVal, MaxVal); \
  std::copy_n(pattern_ ## FieldName.begin(), ArraySize, Message.FieldName.begin()); \
  ASSERT_EQ(pattern_ ## FieldName, Message.FieldName); \

void test_message_primitives_static_arrays(rosidl_generator_cpp::msg::PrimitiveStaticArrays message)
{
  TEST_STATIC_ARRAY_PRIMITIVE(message, bool_value, bool, PRIMITIVES_ARRAY_SIZE, \
    false, true)
  TEST_STATIC_ARRAY_PRIMITIVE(message, char_value, unsigned char, PRIMITIVES_ARRAY_SIZE, \
    0, UINT8_MAX)
  TEST_STATIC_ARRAY_PRIMITIVE(message, byte_value, uint8_t, PRIMITIVES_ARRAY_SIZE, \
    0, UINT8_MAX)
  TEST_STATIC_ARRAY_PRIMITIVE(message, float32_value, float, PRIMITIVES_ARRAY_SIZE, \
    FLT_MIN, FLT_MAX)
  TEST_STATIC_ARRAY_PRIMITIVE(message, float64_value, double, PRIMITIVES_ARRAY_SIZE, \
    DBL_MIN, DBL_MAX)
  TEST_STATIC_ARRAY_PRIMITIVE(message, int8_value, int8_t, PRIMITIVES_ARRAY_SIZE, \
    INT8_MIN, INT8_MAX)
  TEST_STATIC_ARRAY_PRIMITIVE(message, uint8_value, uint8_t, PRIMITIVES_ARRAY_SIZE, \
    0, UINT8_MAX)
  TEST_STATIC_ARRAY_PRIMITIVE(message, int16_value, int16_t, PRIMITIVES_ARRAY_SIZE, \
    INT16_MIN, INT16_MAX)
  TEST_STATIC_ARRAY_PRIMITIVE(message, uint16_value, uint16_t, PRIMITIVES_ARRAY_SIZE, \
    0, UINT16_MAX)
  TEST_STATIC_ARRAY_PRIMITIVE(message, int32_value, int32_t, PRIMITIVES_ARRAY_SIZE, \
    INT32_MIN, INT32_MAX)
  TEST_STATIC_ARRAY_PRIMITIVE(message, uint32_value, uint32_t, PRIMITIVES_ARRAY_SIZE, \
    0, UINT32_MAX)
  TEST_STATIC_ARRAY_PRIMITIVE(message, int64_value, int64_t, PRIMITIVES_ARRAY_SIZE, \
    INT64_MIN, INT64_MAX)
  TEST_STATIC_ARRAY_PRIMITIVE(message, uint64_value, uint64_t, PRIMITIVES_ARRAY_SIZE, \
    0, UINT64_MAX)
}

// Primitives static
TEST(Test_messages, primitives_static) {
  rosidl_generator_cpp::msg::PrimitivesStatic message;
  test_message_primitives_static(message);
}

// Primitives static arrays
TEST(Test_messages, primitives_static_arrays) {
  rosidl_generator_cpp::msg::PrimitiveStaticArrays message;
  test_message_primitives_static_arrays(message);
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

// Constant Primitives
TEST(Test_messages, primitives_constants) {
  rosidl_generator_cpp::msg::PrimitivesConstants message;
  ASSERT_EQ(true, message.BOOL_CONST);
  ASSERT_EQ(50, message.BYTE_CONST);
  ASSERT_EQ(100, message.CHAR_CONST);
  ASSERT_EQ(1.125f, message.FLOAT32_CONST);
  ASSERT_EQ(1.125, message.FLOAT64_CONST);
  ASSERT_EQ(-50, message.INT8_CONST);
  ASSERT_EQ(200u, message.UINT8_CONST);
  ASSERT_EQ(-1000, message.INT16_CONST);
  ASSERT_EQ(2000u, message.UINT16_CONST);
  ASSERT_EQ(-30000, message.INT32_CONST);
  ASSERT_EQ(60000ul, message.UINT32_CONST);
  ASSERT_EQ(-40000000, message.INT64_CONST);
  ASSERT_EQ(50000000ull, message.UINT64_CONST);
  ASSERT_STREQ("foo", message.STRING_CONST.c_str());
}

// Primitives with default values
TEST(Test_messages, primitives_default) {
  rosidl_generator_cpp::msg::PrimitivesDefault message;

// workaround for https://github.com/google/googletest/issues/322
#ifdef __linux__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion-null"
#endif
  TEST_PRIMITIVE_FIELD_ASSIGNMENT(message, bool_value, true, false);
#ifdef __linux__
#pragma GCC diagnostic pop
#endif
  TEST_PRIMITIVE_FIELD_ASSIGNMENT(message, byte_value, 50, 255);
  TEST_PRIMITIVE_FIELD_ASSIGNMENT(message, char_value, 100, UINT8_MAX);
  TEST_PRIMITIVE_FIELD_ASSIGNMENT(message, float32_value, 1.125f, FLT_MAX);
  TEST_PRIMITIVE_FIELD_ASSIGNMENT(message, float64_value, 1.125, DBL_MAX);
  TEST_PRIMITIVE_FIELD_ASSIGNMENT(message, int8_value, -50, INT8_MAX);
  TEST_PRIMITIVE_FIELD_ASSIGNMENT(message, uint8_value, 200, UINT8_MAX);
  TEST_PRIMITIVE_FIELD_ASSIGNMENT(message, int16_value, -1000, INT16_MAX);
  TEST_PRIMITIVE_FIELD_ASSIGNMENT(message, uint16_value, 2000, UINT16_MAX);
  TEST_PRIMITIVE_FIELD_ASSIGNMENT(message, int32_value, -30000, INT32_MAX);
  TEST_PRIMITIVE_FIELD_ASSIGNMENT(message, uint32_value, 60000ul, UINT32_MAX);
  TEST_PRIMITIVE_FIELD_ASSIGNMENT(message, int64_value, -40000000, INT64_MAX);
  TEST_PRIMITIVE_FIELD_ASSIGNMENT(message, uint64_value, 50000000ull, UINT64_MAX);
  TEST_STRING_FIELD_ASSIGNMENT(message, string_value, "bar", "Hello World!")
}

// String arrays with default values
TEST(Test_messages, string_arrays_default) {
  // rosidl_generator_cpp::msg::StringArrays message;
  rosidl_generator_cpp::msg::StringArrays message;

  ASSERT_STREQ("Hello", message.def_string_static_array_value[0].c_str());
  ASSERT_STREQ("World", message.def_string_static_array_value[1].c_str());
  ASSERT_STREQ("!", message.def_string_static_array_value[2].c_str());

  ASSERT_EQ(3ul, message.def_string_bounded_array_value.size());
  ASSERT_STREQ("Hello", message.def_string_bounded_array_value[0].c_str());
  ASSERT_STREQ("World", message.def_string_bounded_array_value[1].c_str());
  ASSERT_STREQ("!", message.def_string_bounded_array_value[2].c_str());

  ASSERT_EQ(5ul, message.def_string_dynamic_array_value.size());
  ASSERT_STREQ("What", message.def_string_dynamic_array_value[0].c_str());
  ASSERT_STREQ("a", message.def_string_dynamic_array_value[1].c_str());
  ASSERT_STREQ("wonderful", message.def_string_dynamic_array_value[2].c_str());
  ASSERT_STREQ("world", message.def_string_dynamic_array_value[3].c_str());
  ASSERT_STREQ("!", message.def_string_dynamic_array_value[4].c_str());

  ASSERT_EQ(4ul, message.def_various_commas.size());
  ASSERT_STREQ("Hel,lo", message.def_various_commas[0].c_str());
  ASSERT_STREQ(",World", message.def_various_commas[1].c_str());
  ASSERT_STREQ("abcd", message.def_various_commas[2].c_str());
  ASSERT_STREQ("!,", message.def_various_commas[3].c_str());

  ASSERT_EQ(2ul, message.def_various_quotes.size());
  ASSERT_STREQ("H\"el'lo", message.def_various_quotes[0].c_str());
  ASSERT_STREQ("Wo'r\"ld", message.def_various_quotes[1].c_str());
}

// TODO(mikaelarguedas) reenable this test when bounded strings enforce length
TEST(Test_messages, DISABLED_Test_bounded_strings) {
  rosidl_generator_cpp::msg::StringBounded message;
  TEST_STRING_FIELD_ASSIGNMENT(message, string_value, "", "Deep into")
  std::string tooLongString = std::string("Too long string");
  message.string_value = tooLongString;
  tooLongString.resize(BOUNDED_STRING_LENGTH);
  ASSERT_STREQ(tooLongString.c_str(), message.string_value.c_str());
}

TEST(Test_messages, Test_string) {
  rosidl_generator_cpp::msg::String message;
  TEST_STRING_FIELD_ASSIGNMENT(message, string_value, "", "Deep into")
}

TEST(Test_messages, Test_wstring) {
  rosidl_generator_cpp::msg::WString message;
  TEST_WSTRING_FIELD_ASSIGNMENT(message, wstring_value, u"", u"wstring_value_\u2122")
}

#define TEST_STATIC_ARRAY_STRING( \
    Message, FieldName, PrimitiveType, ArraySize, MinVal, MaxVal, MinLength, MaxLength) \
  std::array<PrimitiveType, ArraySize> pattern_ ## FieldName; \
  test_vector_fill<decltype(pattern_ ## FieldName)>( \
    &pattern_ ## FieldName, ArraySize, MinVal, MaxVal, MinLength, MaxLength); \
  std::copy_n(pattern_ ## FieldName.begin(), Message.FieldName.size(), Message.FieldName.begin()); \
  ASSERT_EQ(pattern_ ## FieldName, Message.FieldName); \

TEST(Test_messages, Test_string_array_static) {
  rosidl_generator_cpp::msg::StringArrayStatic message;
  TEST_STATIC_ARRAY_STRING(message, string_value, std::string, PRIMITIVES_ARRAY_SIZE, \
    0, UINT32_MAX, 0, UINT16_MAX)
}
