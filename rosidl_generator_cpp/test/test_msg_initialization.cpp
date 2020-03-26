// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#include <cstring>

#include <string>

#include "rosidl_generator_cpp/msg/defaults.hpp"
#include "rosidl_generator_cpp/msg/bounded_sequences.hpp"
#include "rosidl_generator_cpp/msg/multi_nested.hpp"
#include "rosidl_generator_cpp/msg/nested.hpp"
#include "rosidl_generator_cpp/msg/empty.hpp"

template<typename Callable>
struct ScopeExit
{
  explicit ScopeExit(Callable callable)
  : callable_(callable) {}
  ~ScopeExit() {callable_();}

private:
  Callable callable_;
};

template<typename Callable>
ScopeExit<Callable>
make_scope_exit(Callable callable)
{
  return ScopeExit<Callable>(callable);
}

#define DO_STRING_JOIN(arg1, arg2) arg1 ## arg2
#define STRING_JOIN(arg1, arg2) DO_STRING_JOIN(arg1, arg2)
#define SCOPE_EXIT(code) \
  auto STRING_JOIN(scope_exit_, __LINE__) = make_scope_exit([&]() {code;})

TEST(Test_msg_initialization, no_arg_constructor) {
  rosidl_generator_cpp::msg::Defaults def;
  ASSERT_TRUE(def.bool_value);
  ASSERT_EQ(50, def.byte_value);
  ASSERT_EQ(100, def.char_value);
  ASSERT_EQ(1.125f, def.float32_value);
  ASSERT_EQ(1.125, def.float64_value);
  ASSERT_EQ(-50, def.int8_value);
  ASSERT_EQ(200, def.uint8_value);
  ASSERT_EQ(-1000, def.int16_value);
  ASSERT_EQ(2000, def.uint16_value);
  ASSERT_EQ(-30000L, def.int32_value);
  ASSERT_EQ(60000UL, def.uint32_value);
  ASSERT_EQ(-40000000LL, def.int64_value);
  ASSERT_EQ(50000000ULL, def.uint64_value);

  rosidl_generator_cpp::msg::BasicTypes basic;
  ASSERT_FALSE(basic.bool_value);
  ASSERT_EQ(0, basic.byte_value);
  ASSERT_EQ(0, basic.char_value);
  ASSERT_EQ(0.0f, basic.float32_value);
  ASSERT_EQ(0.0, basic.float64_value);
  ASSERT_EQ(0, basic.int8_value);
  ASSERT_EQ(0, basic.uint8_value);
  ASSERT_EQ(0, basic.int16_value);
  ASSERT_EQ(0, basic.uint16_value);
  ASSERT_EQ(0L, basic.int32_value);
  ASSERT_EQ(0UL, basic.uint32_value);
  ASSERT_EQ(0LL, basic.int64_value);
}

TEST(Test_msg_initialization, all_constructor) {
  rosidl_generator_cpp::msg::Defaults def(
    rosidl_generator_cpp::MessageInitialization::ALL);
  ASSERT_TRUE(def.bool_value);
  ASSERT_EQ(50, def.byte_value);
  ASSERT_EQ(100, def.char_value);
  ASSERT_EQ(1.125f, def.float32_value);
  ASSERT_EQ(1.125, def.float64_value);
  ASSERT_EQ(-50, def.int8_value);
  ASSERT_EQ(200, def.uint8_value);
  ASSERT_EQ(-1000, def.int16_value);
  ASSERT_EQ(2000, def.uint16_value);
  ASSERT_EQ(-30000L, def.int32_value);
  ASSERT_EQ(60000UL, def.uint32_value);
  ASSERT_EQ(-40000000LL, def.int64_value);
  ASSERT_EQ(50000000ULL, def.uint64_value);

  rosidl_generator_cpp::msg::BasicTypes basic;
  ASSERT_FALSE(basic.bool_value);
  ASSERT_EQ(0, basic.byte_value);
  ASSERT_EQ(0, basic.char_value);
  ASSERT_EQ(0.0f, basic.float32_value);
  ASSERT_EQ(0.0, basic.float64_value);
  ASSERT_EQ(0, basic.int8_value);
  ASSERT_EQ(0, basic.uint8_value);
  ASSERT_EQ(0, basic.int16_value);
  ASSERT_EQ(0, basic.uint16_value);
  ASSERT_EQ(0L, basic.int32_value);
  ASSERT_EQ(0UL, basic.uint32_value);
  ASSERT_EQ(0LL, basic.int64_value);
}

TEST(Test_msg_initialization, zero_constructor) {
  rosidl_generator_cpp::msg::Defaults def(
    rosidl_generator_cpp::MessageInitialization::ZERO);
  ASSERT_FALSE(def.bool_value);
  ASSERT_EQ(0, def.byte_value);
  ASSERT_EQ(0, def.char_value);
  ASSERT_EQ(0.0f, def.float32_value);
  ASSERT_EQ(0.0, def.float64_value);
  ASSERT_EQ(0, def.int8_value);
  ASSERT_EQ(0, def.uint8_value);
  ASSERT_EQ(0, def.int16_value);
  ASSERT_EQ(0, def.uint16_value);
  ASSERT_EQ(0L, def.int32_value);
  ASSERT_EQ(0UL, def.uint32_value);
  ASSERT_EQ(0LL, def.int64_value);
  ASSERT_EQ(0ULL, def.uint64_value);

  rosidl_generator_cpp::msg::BasicTypes basic;
  ASSERT_FALSE(basic.bool_value);
  ASSERT_EQ(0, basic.byte_value);
  ASSERT_EQ(0, basic.char_value);
  ASSERT_EQ(0.0f, basic.float32_value);
  ASSERT_EQ(0.0, basic.float64_value);
  ASSERT_EQ(0, basic.int8_value);
  ASSERT_EQ(0, basic.uint8_value);
  ASSERT_EQ(0, basic.int16_value);
  ASSERT_EQ(0, basic.uint16_value);
  ASSERT_EQ(0L, basic.int32_value);
  ASSERT_EQ(0UL, basic.uint32_value);
  ASSERT_EQ(0LL, basic.int64_value);
}

TEST(Test_msg_initialization, defaults_only_constructor) {
  rosidl_generator_cpp::msg::Defaults def(
    rosidl_generator_cpp::MessageInitialization::DEFAULTS_ONLY);
  ASSERT_TRUE(def.bool_value);
  ASSERT_EQ(50, def.byte_value);
  ASSERT_EQ(100, def.char_value);
  ASSERT_EQ(1.125f, def.float32_value);
  ASSERT_EQ(1.125, def.float64_value);
  ASSERT_EQ(-50, def.int8_value);
  ASSERT_EQ(200, def.uint8_value);
  ASSERT_EQ(-1000, def.int16_value);
  ASSERT_EQ(2000, def.uint16_value);
  ASSERT_EQ(-30000L, def.int32_value);
  ASSERT_EQ(60000UL, def.uint32_value);
  ASSERT_EQ(-40000000LL, def.int64_value);
  ASSERT_EQ(50000000ULL, def.uint64_value);

  rosidl_generator_cpp::msg::BasicTypes basic;
  ASSERT_FALSE(basic.bool_value);
  ASSERT_EQ(0, basic.byte_value);
  ASSERT_EQ(0, basic.char_value);
  ASSERT_EQ(0.0f, basic.float32_value);
  ASSERT_EQ(0.0, basic.float64_value);
  ASSERT_EQ(0, basic.int8_value);
  ASSERT_EQ(0, basic.uint8_value);
  ASSERT_EQ(0, basic.int16_value);
  ASSERT_EQ(0, basic.uint16_value);
  ASSERT_EQ(0L, basic.int32_value);
  ASSERT_EQ(0UL, basic.uint32_value);
  ASSERT_EQ(0LL, basic.int64_value);
}

// This is a test to ensure that when the user passes SKIP to the constructor,
// it does no initialization.
TEST(Test_msg_initialization, skip_constructor) {
  char * memory = new char[sizeof(rosidl_generator_cpp::msg::BoundedSequences)];
  ASSERT_NE(memory, nullptr);
  std::memset(memory, 0xfe, sizeof(rosidl_generator_cpp::msg::BoundedSequences));
  rosidl_generator_cpp::msg::BoundedSequences * bounded =
    new(memory) rosidl_generator_cpp::msg::BoundedSequences(
    rosidl_generator_cpp::MessageInitialization::SKIP);

  // ensures that the memory gets freed even if an ASSERT is raised
  SCOPE_EXIT(bounded->~BoundedSequences_(); delete[] memory);

  #ifndef _WIN32
  #  pragma GCC diagnostic push
  #  pragma GCC diagnostic ignored "-Wstrict-aliasing"
  #endif
  ASSERT_TRUE(
    std::all_of(
      bounded->float32_values_default.begin(),
      bounded->float32_values_default.end(), [](float i) {
        uint32_t float32_bit_pattern = *reinterpret_cast<uint32_t *>(&i);
        return 0xfefefefe == float32_bit_pattern;
      }));
  ASSERT_TRUE(
    std::all_of(
      bounded->float64_values_default.begin(),
      bounded->float64_values_default.end(), [](double i) {
        uint64_t float64_bit_pattern = *reinterpret_cast<uint64_t *>(&i);
        return 0xfefefefefefefefe == float64_bit_pattern;
      }));

  #ifndef _WIN32
  #  pragma GCC diagnostic pop
  #endif
  ASSERT_EQ(0UL, bounded->float64_values_default.size());
  ASSERT_EQ(0UL, bounded->float32_values_default.size());
  ASSERT_EQ(0UL, bounded->float32_values.size());
  ASSERT_TRUE(
    std::all_of(
      bounded->string_values_default.begin(),
      bounded->string_values_default.end(), [](std::string i) {
        return "" == i;
      }));
}

TEST(Test_msg_initialization, build) {
  rosidl_generator_cpp::msg::BasicTypes basic =
    rosidl_generator_cpp::msg::BasicTypes::build()
    .bool_value(true)
    .byte_value(5)
    .char_value(10)
    .float32_value(0.1125f)
    .float64_value(0.01125)
    .int8_value(-5)
    .uint8_value(20)
    .int16_value(-100)
    .uint16_value(200)
    .int32_value(-3000L)
    .uint32_value(60000UL)
    .int64_value(-4000000LL)
    .uint64_value(5000000ULL);

  ASSERT_TRUE(basic.bool_value);
  ASSERT_EQ(5, basic.byte_value);
  ASSERT_EQ(10, basic.char_value);
  ASSERT_EQ(0.1125f, basic.float32_value);
  ASSERT_EQ(0.01125, basic.float64_value);
  ASSERT_EQ(-5, basic.int8_value);
  ASSERT_EQ(20, basic.uint8_value);
  ASSERT_EQ(-100, basic.int16_value);
  ASSERT_EQ(200, basic.uint16_value);
  ASSERT_EQ(-3000L, basic.int32_value);
  ASSERT_EQ(60000UL, basic.uint32_value);
  ASSERT_EQ(-4000000LL, basic.int64_value);
  ASSERT_EQ(5000000ULL, basic.uint64_value);

  rosidl_generator_cpp::msg::Nested nested =
    rosidl_generator_cpp::msg::Nested::build()
    .basic_types_value(
      rosidl_generator_cpp::msg::BasicTypes::build()
      .bool_value(false)
      .byte_value(10)
      .char_value(20)
      .float32_value(0.225f)
      .float64_value(0.0225)
      .int8_value(-10)
      .uint8_value(40)
      .int16_value(-200)
      .uint16_value(400)
      .int32_value(-6000L)
      .uint32_value(120000UL)
      .int64_value(-8000000LL)
      .uint64_value(10000000ULL));

  ASSERT_FALSE(nested.basic_types_value.bool_value);
  ASSERT_EQ(10, nested.basic_types_value.byte_value);
  ASSERT_EQ(20, nested.basic_types_value.char_value);
  ASSERT_EQ(0.225f, nested.basic_types_value.float32_value);
  ASSERT_EQ(0.0225, nested.basic_types_value.float64_value);
  ASSERT_EQ(-10, nested.basic_types_value.int8_value);
  ASSERT_EQ(40, nested.basic_types_value.uint8_value);
  ASSERT_EQ(-200, nested.basic_types_value.int16_value);
  ASSERT_EQ(400, nested.basic_types_value.uint16_value);
  ASSERT_EQ(-6000L, nested.basic_types_value.int32_value);
  ASSERT_EQ(120000UL, nested.basic_types_value.uint32_value);
  ASSERT_EQ(-8000000LL, nested.basic_types_value.int64_value);
  ASSERT_EQ(10000000ULL, nested.basic_types_value.uint64_value);

  rosidl_generator_cpp::msg::Constants constants;
  rosidl_generator_cpp::msg::Defaults defaults;

  rosidl_generator_cpp::msg::Arrays arrays =
    rosidl_generator_cpp::msg::Arrays::build()
    .bool_values({true, false, true})
    .byte_values({5, 10, 5})
    .char_values({10, 20, 10})
    .float32_values({0.1125f, 0.225f, 0.1125f})
    .float64_values({0.01125, 0.0225, 0.01125})
    .int8_values({-5, -10, -5})
    .uint8_values({20, 40, 20})
    .int16_values({-100, -200, -100})
    .uint16_values({200, 400, 200})
    .int32_values({-3000L, -6000L, -3000L})
    .uint32_values({60000UL, 120000UL, 60000UL})
    .int64_values({-4000000LL, -8000000LL, -4000000LL})
    .uint64_values({5000000ULL, 10000000ULL, 5000000ULL})
    .string_values({"test", "test test", "test"})
    .basic_types_values({basic, basic, basic})
    .constants_values({constants, constants, constants})
    .defaults_values({defaults, defaults, defaults})
    .bool_values_default({false, true, false})
    .byte_values_default({10, 5, 10})
    .char_values_default({20, 10, 20})
    .float32_values_default({0.225f, 0.1125f, 0.225f})
    .float64_values_default({0.0225, 0.01125, 0.0225})
    .int8_values_default({-10, -5, -10})
    .uint8_values_default({40, 20, 40})
    .int16_values_default({-200, -100, -200})
    .uint16_values_default({40, 20, 40})
    .int32_values_default({-6000L, -3000L, -6000L})
    .uint32_values_default({120000UL, 60000UL, 120000UL})
    .int64_values_default({-8000000LL, -4000000LL, -8000000LL})
    .uint64_values_default({10000000ULL, 5000000ULL, 10000000ULL})
    .string_values_default({"test test", "test", "test test"})
    .alignment_check(0l);

  rosidl_generator_cpp::msg::BoundedSequences bounded_sequences;
  rosidl_generator_cpp::msg::UnboundedSequences unbounded_sequences;

  rosidl_generator_cpp::msg::MultiNested multi_nested =
    rosidl_generator_cpp::msg::MultiNested::build()
    .array_of_arrays({arrays, arrays, arrays})
    .array_of_bounded_sequences({
      bounded_sequences,
      bounded_sequences,
      bounded_sequences
    })
    .array_of_unbounded_sequences({
      unbounded_sequences,
      unbounded_sequences,
      unbounded_sequences
    })
    .bounded_sequence_of_arrays({arrays, arrays})
    .bounded_sequence_of_bounded_sequences({
      bounded_sequences,
      bounded_sequences
    })
    .bounded_sequence_of_unbounded_sequences({
      unbounded_sequences,
      unbounded_sequences
    })
    .unbounded_sequence_of_arrays({arrays})
    .unbounded_sequence_of_bounded_sequences({bounded_sequences})
    .unbounded_sequence_of_unbounded_sequences({unbounded_sequences});

  for (const auto & a : multi_nested.array_of_arrays) {
    ASSERT_EQ(arrays, a);
  }

  for (const auto & b : multi_nested.array_of_bounded_sequences) {
    ASSERT_EQ(bounded_sequences, b);
  }

  for (const auto & u : multi_nested.array_of_unbounded_sequences) {
    ASSERT_EQ(unbounded_sequences, u);
  }

  ASSERT_EQ(2u, multi_nested.bounded_sequence_of_arrays.size());
  for (const auto & a : multi_nested.bounded_sequence_of_arrays) {
    ASSERT_EQ(arrays, a);
  }

  ASSERT_EQ(2u, multi_nested.bounded_sequence_of_bounded_sequences.size());
  for (const auto & b : multi_nested.bounded_sequence_of_bounded_sequences) {
    ASSERT_EQ(bounded_sequences, b);
  }

  ASSERT_EQ(2u, multi_nested.bounded_sequence_of_unbounded_sequences.size());
  for (const auto & u : multi_nested.bounded_sequence_of_unbounded_sequences) {
    ASSERT_EQ(unbounded_sequences, u);
  }

  ASSERT_EQ(1u, multi_nested.unbounded_sequence_of_arrays.size());
  for (const auto & a : multi_nested.unbounded_sequence_of_arrays) {
    ASSERT_EQ(arrays, a);
  }

  ASSERT_EQ(1u, multi_nested.unbounded_sequence_of_bounded_sequences.size());
  for (const auto & b : multi_nested.unbounded_sequence_of_bounded_sequences) {
    ASSERT_EQ(bounded_sequences, b);
  }

  ASSERT_EQ(1u, multi_nested.unbounded_sequence_of_unbounded_sequences.size());
  for (const auto & u : multi_nested.unbounded_sequence_of_unbounded_sequences) {
    ASSERT_EQ(unbounded_sequences, u);
  }

  rosidl_generator_cpp::msg::Empty empty =
    rosidl_generator_cpp::msg::Empty::build()
    .structure_needs_at_least_one_member(5);
  ASSERT_EQ(5, empty.structure_needs_at_least_one_member);
}
