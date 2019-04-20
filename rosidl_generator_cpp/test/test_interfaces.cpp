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

#include "rosidl_generator_cpp/msg/array_primitives.hpp"
#include "rosidl_generator_cpp/msg/array_nested.hpp"
// TODO(jacobperron): Add test case for ArrayBoundedPrimitivesNested (need interface)
// #include "rosidl_generator_cpp/msg/array_bounded_primitives_nested.hpp"
// TODO(jacobperron): Add test case for ArrayUnboundedPrimitivesNested (need interface)
// #include "rosidl_generator_cpp/msg/array_unbounded_primitives_nested.hpp"
#include "rosidl_generator_cpp/msg/bounded_sequence_nested.hpp"
#include "rosidl_generator_cpp/msg/bounded_sequence_primitives.hpp"
#include "rosidl_generator_cpp/msg/bounded_sequence_primitives_nested.hpp"
// TODO(jacobperron): Add test case for BoundedSequenceUnboundedPrimitivesNested (need interface)
// #include "rosidl_generator_cpp/msg/bounded_sequence_unbounded_primitives_nested.hpp"
#include "rosidl_generator_cpp/msg/bounded_string.hpp"
#include "rosidl_generator_cpp/msg/empty.hpp"
#include "rosidl_generator_cpp/msg/primitives.hpp"
#include "rosidl_generator_cpp/msg/primitives_constants.hpp"
#include "rosidl_generator_cpp/msg/primitives_default.hpp"
#include "rosidl_generator_cpp/msg/unbounded_sequence_nested.hpp"
#include "rosidl_generator_cpp/msg/unbounded_sequence_primitives.hpp"
#include "rosidl_generator_cpp/msg/unbounded_sequence_primitives_nested.hpp"
// TODO(jacobperron): Add test case for UnboundedSequenceBoundedPrimitivesNested (need interface)
// #include "rosidl_generator_cpp/msg/unbounded_sequence_bounded_primitives_nested.hpp"

#define ARRAY_PRIMITIVES_SIZE 3
#define BOUNDED_SEQUENCE_PRIMITIVES_SIZE 3
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
    !rosidl_generator_traits::has_fixed_size<rosidl_generator_cpp::msg::Primitives>::value,
    "Primitives::has_fixed_size is true");

  static_assert(
    !rosidl_generator_traits::has_fixed_size<
      rosidl_generator_cpp::msg::BoundedSequencePrimitives>::value,
    "BoundedSequencePrimitives::has_fixed_size is true");

  static_assert(
    !rosidl_generator_traits::has_fixed_size<
      rosidl_generator_cpp::msg::UnboundedSequencePrimitives>::value,
    "UnboundedSequencePrimitives::has_fixed_size is true");

  static_assert(
    !rosidl_generator_traits::has_fixed_size<rosidl_generator_cpp::msg::ArrayPrimitives>::value,
    "ArrayPrimitives::has_fixed_size is true");

  static_assert(
    !rosidl_generator_traits::has_fixed_size<rosidl_generator_cpp::msg::ArrayNested>::value,
    "ArrayNested::has_fixed_size is true");

  // TODO(jacobperron): Add interface file
  // static_assert(
  //   !rosidl_generator_traits::has_fixed_size<rosidl_generator_cpp::msg::ArrayBoundedPrimitivesNested>::value,
  //   "ArrayBoundedPrimitivesNested::has_fixed_size is true");

  // TODO(jacobperron): Add interface file
  // static_assert(
  //   !rosidl_generator_traits::has_fixed_size<
  //     rosidl_generator_cpp::msg::ArrayUnboundedPrimitivesNested>::value,
  //   "ArrayUnboundedPrimitivesNested::has_fixed_size is true");

  static_assert(
    !rosidl_generator_traits::has_fixed_size<rosidl_generator_cpp::msg::BoundedString>::value,
    "BoundedString::has_fixed_size is true");

  static_assert(
    !rosidl_generator_traits::has_fixed_size<
      rosidl_generator_cpp::msg::BoundedSequenceNested>::value,
    "BoundedSequenceNested::has_fixed_size is true");

  static_assert(
    !rosidl_generator_traits::has_fixed_size<
      rosidl_generator_cpp::msg::BoundedSequencePrimitivesNested>::value,
    "BoundedSequencePrimitivesNested::has_fixed_size is true");

  // TODO(jacobperron): Add test interface
  // static_assert(
  //   !rosidl_generator_traits::has_fixed_size<
  //     rosidl_generator_cpp::msg::BoundedSequenceUnboundedPrimitivesNested>::value,
  //   "BoundedSequenceUnboundedPrimitivesNested::has_fixed_size is true");

  static_assert(
    !rosidl_generator_traits::has_fixed_size<
      rosidl_generator_cpp::msg::UnboundedSequenceNested>::value,
    "UnboundedSequenceNested::has_fixed_size is true");

  // TODO(jacobperron): Add test interface
  // static_assert(
  //   !rosidl_generator_traits::has_fixed_size<
  //     rosidl_generator_cpp::msg::UnboundedSequenceBoundedPrimitivesNested>::value,
  //   "UnboundedSequenceBoundedPrimitivesNested::has_fixed_size is true");

  static_assert(
    !rosidl_generator_traits::has_fixed_size<
      rosidl_generator_cpp::msg::UnboundedSequencePrimitivesNested>::value,
    "UnboundedSequencePrimitivesNested::has_fixed_size is true");
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
    !rosidl_generator_traits::has_bounded_size<rosidl_generator_cpp::msg::Primitives>::value,
    "Primitives::has_bounded_size is true");

  static_assert(
    !rosidl_generator_traits::has_bounded_size<
      rosidl_generator_cpp::msg::BoundedSequencePrimitives>::value,
    "BoundedSequencePrimitives::has_bounded_size is true");

  static_assert(
    !rosidl_generator_traits::has_bounded_size<
      rosidl_generator_cpp::msg::UnboundedSequencePrimitives>::value,
    "UnboundedSequencePrimitives::has_bounded_size is true");

  static_assert(
    !rosidl_generator_traits::has_bounded_size<
      rosidl_generator_cpp::msg::ArrayPrimitives>::value,
    "ArrayPrimitives::has_bounded_size is true");

  static_assert(
    !rosidl_generator_traits::has_bounded_size<rosidl_generator_cpp::msg::ArrayNested>::value,
    "ArrayNested::has_bounded_size is true");

  // TODO(jacobperron): Add interface file
  // static_assert(
  //   rosidl_generator_traits::has_bounded_size<rosidl_generator_cpp::msg::ArrayBoundedPrimitivesNested>::value,
  //   "ArrayBoundedPrimitivesNested::has_bounded_size is false");

  // TODO(jacobperron): Add interface file
  // static_assert(
  //   !rosidl_generator_traits::has_bounded_size<
  //     rosidl_generator_cpp::msg::ArrayUnboundedPrimitivesNested>::value,
  //   "ArrayUnboundedPrimitivesNested::has_bounded_size is true");

  static_assert(
    rosidl_generator_traits::has_bounded_size<rosidl_generator_cpp::msg::BoundedString>::value,
    "BoundedString::has_bounded_size is false");

  static_assert(
    !rosidl_generator_traits::has_bounded_size<
      rosidl_generator_cpp::msg::BoundedSequenceNested>::value,
    "BoundedSequenceNested::has_bounded_size is true");

  static_assert(
    !rosidl_generator_traits::has_bounded_size<
      rosidl_generator_cpp::msg::BoundedSequencePrimitivesNested>::value,
    "BoundedSequencePrimitivesNested::has_bounded_size is true");

  // TODO(jacobperron): Add test interface
  // static_assert(
  //   !rosidl_generator_traits::has_bounded_size<
  //     rosidl_generator_cpp::msg::BoundedSequenceUnboundedPrimitivesNested>::value,
  //   "BoundedSequenceUnboundedPrimitivesNested::has_bounded_size is true");

  static_assert(
    !rosidl_generator_traits::has_bounded_size<
      rosidl_generator_cpp::msg::UnboundedSequenceNested>::value,
    "UnboundedSequenceNested::has_bounded_size is true");

  // TODO(jacobperron): Add test interface
  // static_assert(
  //   !rosidl_generator_traits::has_bounded_size<
  //     rosidl_generator_cpp::msg::UnboundedSequenceBoundedPrimitivesNested>::value,
  //   "UnboundedSequenceBoundedPrimitivesNested::has_bounded_size is true");

  static_assert(
    !rosidl_generator_traits::has_bounded_size<
      rosidl_generator_cpp::msg::UnboundedSequencePrimitivesNested>::value,
    "UnboundedSequencePrimitivesNested::has_bounded_size is true");
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

void test_message_primitives(rosidl_generator_cpp::msg::Primitives message)
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
  TEST_STRING_FIELD_ASSIGNMENT(message, string_value, "bar", "Hello World!")
}

#define TEST_BOUNDED_SEQUENCE_PRIMITIVES( \
    Message, FieldName, PrimitiveType, ArraySize, MinVal, MaxVal) \
  rosidl_generator_cpp::BoundedVector<PrimitiveType, ArraySize> pattern_ ## FieldName; \
  Message.FieldName.resize(ArraySize); \
  pattern_ ## FieldName.resize(ArraySize); \
  test_vector_fill<decltype(pattern_ ## FieldName)>( \
    &pattern_ ## FieldName, ArraySize, MinVal, MaxVal); \
  std::copy_n(pattern_ ## FieldName.begin(), Message.FieldName.size(), Message.FieldName.begin()); \
  ASSERT_EQ(pattern_ ## FieldName, Message.FieldName); \

#define TEST_BOUNDED_SEQUENCE_STRING( \
    Message, FieldName, PrimitiveType, ArraySize, MinVal, MaxVal, MinLength, MaxLength) \
  rosidl_generator_cpp::BoundedVector<PrimitiveType, ArraySize> pattern_ ## FieldName; \
  Message.FieldName.resize(ArraySize); \
  pattern_ ## FieldName.resize(ArraySize); \
  test_vector_fill<decltype(pattern_ ## FieldName)>( \
    &pattern_ ## FieldName, ArraySize, MinVal, MaxVal, MinLength, MaxLength); \
  std::copy_n(pattern_ ## FieldName.begin(), Message.FieldName.size(), Message.FieldName.begin()); \
  ASSERT_EQ(pattern_ ## FieldName, Message.FieldName); \

void test_message_bounded_sequence_primitives(
  rosidl_generator_cpp::msg::BoundedSequencePrimitives message)
{
  TEST_BOUNDED_SEQUENCE_PRIMITIVES(message, bool_values, bool, \
    BOUNDED_SEQUENCE_PRIMITIVES_SIZE, false, true)
  TEST_BOUNDED_SEQUENCE_PRIMITIVES(message, char_values, unsigned char, \
    BOUNDED_SEQUENCE_PRIMITIVES_SIZE, 0, UINT8_MAX)
  TEST_BOUNDED_SEQUENCE_PRIMITIVES(message, byte_values, uint8_t, \
    BOUNDED_SEQUENCE_PRIMITIVES_SIZE, 0, UINT8_MAX)
  TEST_BOUNDED_SEQUENCE_PRIMITIVES(message, float32_values, float, \
    BOUNDED_SEQUENCE_PRIMITIVES_SIZE, FLT_MIN, FLT_MAX)
  TEST_BOUNDED_SEQUENCE_PRIMITIVES(message, float64_values, double, \
    BOUNDED_SEQUENCE_PRIMITIVES_SIZE, DBL_MIN, DBL_MAX)
  TEST_BOUNDED_SEQUENCE_PRIMITIVES(message, int8_values, int8_t, \
    BOUNDED_SEQUENCE_PRIMITIVES_SIZE, INT8_MIN, INT8_MAX)
  TEST_BOUNDED_SEQUENCE_PRIMITIVES(message, uint8_values, uint8_t, \
    BOUNDED_SEQUENCE_PRIMITIVES_SIZE, 0, UINT8_MAX)
  TEST_BOUNDED_SEQUENCE_PRIMITIVES(message, int16_values, int16_t, \
    BOUNDED_SEQUENCE_PRIMITIVES_SIZE, INT16_MIN, INT16_MAX)
  TEST_BOUNDED_SEQUENCE_PRIMITIVES(message, uint16_values, uint16_t, \
    BOUNDED_SEQUENCE_PRIMITIVES_SIZE, 0, UINT16_MAX)
  TEST_BOUNDED_SEQUENCE_PRIMITIVES(message, int32_values, int32_t, \
    BOUNDED_SEQUENCE_PRIMITIVES_SIZE, INT32_MIN, INT32_MAX)
  TEST_BOUNDED_SEQUENCE_PRIMITIVES(message, uint32_values, uint32_t, \
    BOUNDED_SEQUENCE_PRIMITIVES_SIZE, 0, UINT32_MAX)
  TEST_BOUNDED_SEQUENCE_PRIMITIVES(message, int64_values, int64_t, \
    BOUNDED_SEQUENCE_PRIMITIVES_SIZE, INT64_MIN, INT64_MAX)
  TEST_BOUNDED_SEQUENCE_PRIMITIVES(message, uint64_values, uint64_t, \
    BOUNDED_SEQUENCE_PRIMITIVES_SIZE, 0, UINT64_MAX)
  TEST_BOUNDED_SEQUENCE_STRING(message, string_values, std::string, \
    BOUNDED_SEQUENCE_PRIMITIVES_SIZE, 0, UINT32_MAX, 0, BOUNDED_STRING_LENGTH)
}

#define TEST_UNBOUNDED_SEQUENCE_PRIMITIVES( \
    Message, FieldName, PrimitiveType, ArraySize, MinVal, MaxVal) \
  std::vector<PrimitiveType> pattern_ ## FieldName(ArraySize); \
  pattern_ ## FieldName.resize(ArraySize); \
  test_vector_fill<decltype(pattern_ ## FieldName)>( \
    &pattern_ ## FieldName, ArraySize, MinVal, MaxVal); \
  Message.FieldName.resize(ArraySize); \
  std::copy_n(pattern_ ## FieldName.begin(), ArraySize, Message.FieldName.begin()); \
  ASSERT_EQ(pattern_ ## FieldName, Message.FieldName); \

#define TEST_UNBOUNDED_SEQUENCE_STRING( \
    Message, FieldName, PrimitiveType, ArraySize, MinVal, MaxVal, MinLength, MaxLength) \
  std::vector<PrimitiveType> pattern_ ## FieldName; \
  Message.FieldName.resize(ArraySize); \
  pattern_ ## FieldName.resize(ArraySize); \
  test_vector_fill<decltype(pattern_ ## FieldName)>( \
    &pattern_ ## FieldName, ArraySize, MinVal, MaxVal, MinLength, MaxLength); \
  std::copy_n(pattern_ ## FieldName.begin(), Message.FieldName.size(), Message.FieldName.begin()); \
  ASSERT_EQ(pattern_ ## FieldName, Message.FieldName); \

void test_message_unbounded_sequence_primitives(
  rosidl_generator_cpp::msg::UnboundedSequencePrimitives message)
{
  TEST_UNBOUNDED_SEQUENCE_PRIMITIVES(message, bool_values, bool, ARRAY_PRIMITIVES_SIZE, \
    false, true)
  TEST_UNBOUNDED_SEQUENCE_PRIMITIVES(message, char_values, unsigned char, ARRAY_PRIMITIVES_SIZE, \
    0, UINT8_MAX)
  TEST_UNBOUNDED_SEQUENCE_PRIMITIVES(message, byte_values, uint8_t, ARRAY_PRIMITIVES_SIZE, \
    0, UINT8_MAX)
  TEST_UNBOUNDED_SEQUENCE_PRIMITIVES(message, float32_values, float, ARRAY_PRIMITIVES_SIZE, \
    FLT_MIN, FLT_MAX)
  TEST_UNBOUNDED_SEQUENCE_PRIMITIVES(message, float64_values, double, ARRAY_PRIMITIVES_SIZE, \
    DBL_MIN, DBL_MAX)
  TEST_UNBOUNDED_SEQUENCE_PRIMITIVES(message, int8_values, int8_t, ARRAY_PRIMITIVES_SIZE, \
    INT8_MIN, INT8_MAX)
  TEST_UNBOUNDED_SEQUENCE_PRIMITIVES(message, uint8_values, uint8_t, ARRAY_PRIMITIVES_SIZE, \
    0, UINT8_MAX)
  TEST_UNBOUNDED_SEQUENCE_PRIMITIVES(message, int16_values, int16_t, ARRAY_PRIMITIVES_SIZE, \
    INT16_MIN, INT16_MAX)
  TEST_UNBOUNDED_SEQUENCE_PRIMITIVES(message, uint16_values, uint16_t, ARRAY_PRIMITIVES_SIZE, \
    0, UINT16_MAX)
  TEST_UNBOUNDED_SEQUENCE_PRIMITIVES(message, int32_values, int32_t, ARRAY_PRIMITIVES_SIZE, \
    INT32_MIN, INT32_MAX)
  TEST_UNBOUNDED_SEQUENCE_PRIMITIVES(message, uint32_values, uint32_t, ARRAY_PRIMITIVES_SIZE, \
    0, UINT32_MAX)
  TEST_UNBOUNDED_SEQUENCE_PRIMITIVES(message, int64_values, int64_t, ARRAY_PRIMITIVES_SIZE, \
    INT64_MIN, INT64_MAX)
  TEST_UNBOUNDED_SEQUENCE_PRIMITIVES(message, uint64_values, uint64_t, ARRAY_PRIMITIVES_SIZE, \
    0, UINT64_MAX)
  TEST_UNBOUNDED_SEQUENCE_STRING(message, string_values, std::string, ARRAY_PRIMITIVES_SIZE, \
    0, UINT32_MAX, 0, UINT16_MAX)
}

#define TEST_ARRAY_PRIMITIVES( \
    Message, FieldName, PrimitiveType, ArraySize, MinVal, MaxVal) \
  std::array<PrimitiveType, ArraySize> pattern_ ## FieldName; \
  test_vector_fill<decltype(pattern_ ## FieldName)>( \
    &pattern_ ## FieldName, ArraySize, MinVal, MaxVal); \
  std::copy_n(pattern_ ## FieldName.begin(), ArraySize, Message.FieldName.begin()); \
  ASSERT_EQ(pattern_ ## FieldName, Message.FieldName); \

#define TEST_ARRAY_STRING( \
    Message, FieldName, PrimitiveType, ArraySize, MinVal, MaxVal, MinLength, MaxLength) \
  std::array<PrimitiveType, ArraySize> pattern_ ## FieldName; \
  test_vector_fill<decltype(pattern_ ## FieldName)>( \
    &pattern_ ## FieldName, ArraySize, MinVal, MaxVal, MinLength, MaxLength); \
  std::copy_n(pattern_ ## FieldName.begin(), Message.FieldName.size(), Message.FieldName.begin()); \
  ASSERT_EQ(pattern_ ## FieldName, Message.FieldName); \

void test_message_array_primitives(rosidl_generator_cpp::msg::ArrayPrimitives message)
{
  TEST_ARRAY_PRIMITIVES(message, bool_values, bool, ARRAY_PRIMITIVES_SIZE, \
    false, true)
  TEST_ARRAY_PRIMITIVES(message, char_values, unsigned char, ARRAY_PRIMITIVES_SIZE, \
    0, UINT8_MAX)
  TEST_ARRAY_PRIMITIVES(message, byte_values, uint8_t, ARRAY_PRIMITIVES_SIZE, \
    0, UINT8_MAX)
  TEST_ARRAY_PRIMITIVES(message, float32_values, float, ARRAY_PRIMITIVES_SIZE, \
    FLT_MIN, FLT_MAX)
  TEST_ARRAY_PRIMITIVES(message, float64_values, double, ARRAY_PRIMITIVES_SIZE, \
    DBL_MIN, DBL_MAX)
  TEST_ARRAY_PRIMITIVES(message, int8_values, int8_t, ARRAY_PRIMITIVES_SIZE, \
    INT8_MIN, INT8_MAX)
  TEST_ARRAY_PRIMITIVES(message, uint8_values, uint8_t, ARRAY_PRIMITIVES_SIZE, \
    0, UINT8_MAX)
  TEST_ARRAY_PRIMITIVES(message, int16_values, int16_t, ARRAY_PRIMITIVES_SIZE, \
    INT16_MIN, INT16_MAX)
  TEST_ARRAY_PRIMITIVES(message, uint16_values, uint16_t, ARRAY_PRIMITIVES_SIZE, \
    0, UINT16_MAX)
  TEST_ARRAY_PRIMITIVES(message, int32_values, int32_t, ARRAY_PRIMITIVES_SIZE, \
    INT32_MIN, INT32_MAX)
  TEST_ARRAY_PRIMITIVES(message, uint32_values, uint32_t, ARRAY_PRIMITIVES_SIZE, \
    0, UINT32_MAX)
  TEST_ARRAY_PRIMITIVES(message, int64_values, int64_t, ARRAY_PRIMITIVES_SIZE, \
    INT64_MIN, INT64_MAX)
  TEST_ARRAY_PRIMITIVES(message, uint64_values, uint64_t, ARRAY_PRIMITIVES_SIZE, \
    0, UINT64_MAX)
  TEST_ARRAY_STRING(message, string_values, std::string, ARRAY_PRIMITIVES_SIZE, \
    0, UINT32_MAX, 0, UINT16_MAX)
}

// Primitives
TEST(Test_messages, primitives) {
  rosidl_generator_cpp::msg::Primitives message;
  test_message_primitives(message);
}

// Arrays of primitives
TEST(Test_messages, array_primitives) {
  rosidl_generator_cpp::msg::ArrayPrimitives message;
  test_message_array_primitives(message);
}

// Bounded sequences of primitives
TEST(Test_messages, bounded_sequence_primitives) {
  rosidl_generator_cpp::msg::BoundedSequencePrimitives message;
  test_message_bounded_sequence_primitives(message);
}

// Unbounded sequences of primitives
TEST(Test_messages, unbounded_sequence_primitives) {
  rosidl_generator_cpp::msg::UnboundedSequencePrimitives message;
  test_message_unbounded_sequence_primitives(message);
}

// Array of Primitives messages
TEST(Test_messages, array_nested) {
  rosidl_generator_cpp::msg::ArrayNested message;
  for (int i = 0; i < SUBMESSAGE_ARRAY_SIZE; i++) {
    test_message_primitives(message.primitives_values[i]);
  }
}

// TODO(jacobperron): Add interface file
// Array of BoundedSequencePrimitives messages
// TEST(Test_messages, array_bounded_primitives_nested) {
//   rosidl_generator_cpp::msg::ArrayBoundedPrimitivesNested message;
//   for (int i = 0; i < SUBMESSAGE_ARRAY_SIZE; i++) {
//     test_message_bounded_sequence_primitives(message.bounded_sequence_primitives_values[i]);
//   }
// }

// TODO(jacobperron): Add interface file
// Array of UnboundedSequencePrimitives messages
// TEST(Test_messages, array_unbounded_primitives_nested) {
//   rosidl_generator_cpp::msg::ArrayUnboundedPrimitivesNested message;
//   for (int i = 0; i < SUBMESSAGE_ARRAY_SIZE; i++) {
//     test_message_unbounded_sequence_primitives(message.unbounded_sequence_primitives_values[i]);
//   }
// }

// Bounded sequence of Primitives messages
TEST(Test_messages, bounded_sequence_nested) {
  rosidl_generator_cpp::msg::BoundedSequenceNested message;
  message.primitives_values.resize(SUBMESSAGE_ARRAY_SIZE);
  for (int i = 0; i < SUBMESSAGE_ARRAY_SIZE; i++) {
    test_message_primitives(message.primitives_values[i]);
  }
}

// Bounded sequence of BoundedSequencePrimitives messages
TEST(Test_messages, bounded_sequence_primitives_nested) {
  rosidl_generator_cpp::msg::BoundedSequencePrimitivesNested message;
  message.bounded_sequence_primitives_values.resize(SUBMESSAGE_ARRAY_SIZE);
  for (int i = 0; i < SUBMESSAGE_ARRAY_SIZE; i++) {
    test_message_bounded_sequence_primitives(message.bounded_sequence_primitives_values[i]);
  }
}

// TODO(jacobperron): Add test interface
// Bounded sequence of UnboundedSequencePrimitives messages
// TEST(Test_messages, bounded_array_unbounded) {
//   rosidl_generator_cpp::msg::BoundedSequenceUnboundedPrimitivesNested message;
//   message.unbounded_seqeuence_primitives_values.resize(SUBMESSAGE_ARRAY_SIZE);
//   for (int i = 0; i < SUBMESSAGE_ARRAY_SIZE; i++) {
//     test_message_unbounded_sequence_primitives(message.unbounded_sequence_primitive_values[i]);
//   }
// }

// Unbounded sequence of Primitives messages
TEST(Test_messages, unbounded_sequence_nested) {
  rosidl_generator_cpp::msg::UnboundedSequenceNested message;
  message.primitives_values.resize(SUBMESSAGE_ARRAY_SIZE);
  for (int i = 0; i < SUBMESSAGE_ARRAY_SIZE; i++) {
    test_message_primitives(message.primitives_values[i]);
  }
}

// Unbounded sequence of BoundedSequencePrimitives messages
// TODO(jacobperron): Add test interface
// TEST(Test_messages, unbounded_sequence_bounded_primitives_nested) {
//   rosidl_generator_cpp::msg::UnboundedSequenceBoundedPrimitivesNested message;
//   message.bounded_sequence_primitives_values.resize(SUBMESSAGE_ARRAY_SIZE);
//   for (int i = 0; i < SUBMESSAGE_ARRAY_SIZE; i++) {
//     test_message_bounded_sequence_primitives(message.bounded_sequence_primitive_values[i]);
//   }
// }

// Unbounded sequence of UnboundedSequencePrimitives messages
TEST(Test_messages, unbounded_sequence_primitives_nested) {
  rosidl_generator_cpp::msg::UnboundedSequencePrimitivesNested message;
  message.unbounded_sequence_primitives_values.resize(SUBMESSAGE_ARRAY_SIZE);
  for (int i = 0; i < SUBMESSAGE_ARRAY_SIZE; i++) {
    test_message_unbounded_sequence_primitives(message.unbounded_sequence_primitives_values[i]);
  }
}

// Primitives with constants
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

// TODO(mikaelarguedas) reenable this test when bounded strings enforce length
TEST(Test_messages, DISABLED_Test_bounded_strings) {
  rosidl_generator_cpp::msg::BoundedString message;
  TEST_STRING_FIELD_ASSIGNMENT(message, bounded_string_value, "", "Deep into")
  std::string tooLongString = std::string("Too long string");
  message.bounded_string_value = tooLongString;
  tooLongString.resize(BOUNDED_STRING_LENGTH);
  ASSERT_STREQ(tooLongString.c_str(), message.bounded_string_value.c_str());
}
