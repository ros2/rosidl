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

#include "rosidl_generator_cpp/msg/primitives.hpp"
#include "rosidl_generator_cpp/msg/primitives_default.hpp"
#include "rosidl_generator_cpp/msg/various.hpp"

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
  rosidl_generator_cpp::msg::PrimitivesDefault primitives_def;
  ASSERT_TRUE(primitives_def.bool_value);
  ASSERT_EQ(50, primitives_def.byte_value);
  ASSERT_EQ(100, primitives_def.char_value);
  ASSERT_EQ(1.125f, primitives_def.float32_value);
  ASSERT_EQ(1.125, primitives_def.float64_value);
  ASSERT_EQ(-50, primitives_def.int8_value);
  ASSERT_EQ(200, primitives_def.uint8_value);
  ASSERT_EQ(-1000, primitives_def.int16_value);
  ASSERT_EQ(2000, primitives_def.uint16_value);
  ASSERT_EQ(-30000L, primitives_def.int32_value);
  ASSERT_EQ(60000UL, primitives_def.uint32_value);
  ASSERT_EQ(-40000000LL, primitives_def.int64_value);
  ASSERT_EQ(50000000ULL, primitives_def.uint64_value);
  ASSERT_EQ("bar", primitives_def.string_value);

  rosidl_generator_cpp::msg::Primitives primitives;
  ASSERT_FALSE(primitives.bool_value);
  ASSERT_EQ(0, primitives.byte_value);
  ASSERT_EQ(0, primitives.char_value);
  ASSERT_EQ(0.0f, primitives.float32_value);
  ASSERT_EQ(0.0, primitives.float64_value);
  ASSERT_EQ(0, primitives.int8_value);
  ASSERT_EQ(0, primitives.uint8_value);
  ASSERT_EQ(0, primitives.int16_value);
  ASSERT_EQ(0, primitives.uint16_value);
  ASSERT_EQ(0L, primitives.int32_value);
  ASSERT_EQ(0UL, primitives.uint32_value);
  ASSERT_EQ(0LL, primitives.int64_value);
  ASSERT_EQ(0ULL, primitives.uint64_value);
  ASSERT_EQ("", primitives.string_value);

  rosidl_generator_cpp::msg::Various various;
  ASSERT_EQ(1.125f, various.float32_value_def);
  ASSERT_EQ(2.4, various.float64_value_def);
  ASSERT_EQ(0ULL, various.uint64_value);
  ASSERT_EQ("bar", various.string_value_def);
  ASSERT_EQ(10UL, various.float32_arr.size());
  for (const float value : various.float32_arr) {
    ASSERT_EQ(0.0f, value);
  }
  ASSERT_EQ(0, various.array_alignment_check);
  ASSERT_EQ(3UL, various.float64_arr_def.size());
  ASSERT_EQ(8.5, various.float64_arr_def[0]);
  ASSERT_EQ(1.2, various.float64_arr_def[1]);
  ASSERT_EQ(3.4, various.float64_arr_def[2]);
  ASSERT_EQ(3UL, various.string_arr.size());
  for (const std::string value : various.string_arr) {
    ASSERT_EQ("", value);
  }
  ASSERT_EQ(3UL, various.string_arr_def.size());
  ASSERT_EQ("hello", various.string_arr_def[0]);
  ASSERT_EQ("world", various.string_arr_def[1]);
  ASSERT_EQ("!", various.string_arr_def[2]);
  ASSERT_EQ(2UL, various.unbounded_seq_def.size());
  ASSERT_EQ(1.0f, various.unbounded_seq_def[0]);
  ASSERT_EQ(2.0f, various.unbounded_seq_def[1]);
  ASSERT_EQ(0UL, various.unbounded_seq.size());
  ASSERT_EQ(0, various.unbounded_alignment_check);
  ASSERT_EQ(2UL, various.bounded_seq_def.size());
  ASSERT_EQ(3.0f, various.bounded_seq_def[0]);
  ASSERT_EQ(4.0f, various.bounded_seq_def[1]);
  ASSERT_EQ(0UL, various.bounded_seq.size());
  ASSERT_EQ(0, various.bounded_alignment_check);
}

TEST(Test_msg_initialization, all_constructor) {
  rosidl_generator_cpp::msg::PrimitivesDefault primitives_def(
    rosidl_generator_cpp::MessageInitialization::ALL);
  ASSERT_TRUE(primitives_def.bool_value);
  ASSERT_EQ(50, primitives_def.byte_value);
  ASSERT_EQ(100, primitives_def.char_value);
  ASSERT_EQ(1.125f, primitives_def.float32_value);
  ASSERT_EQ(1.125, primitives_def.float64_value);
  ASSERT_EQ(-50, primitives_def.int8_value);
  ASSERT_EQ(200, primitives_def.uint8_value);
  ASSERT_EQ(-1000, primitives_def.int16_value);
  ASSERT_EQ(2000, primitives_def.uint16_value);
  ASSERT_EQ(-30000L, primitives_def.int32_value);
  ASSERT_EQ(60000UL, primitives_def.uint32_value);
  ASSERT_EQ(-40000000LL, primitives_def.int64_value);
  ASSERT_EQ(50000000ULL, primitives_def.uint64_value);
  ASSERT_EQ("bar", primitives_def.string_value);
}

TEST(Test_msg_initialization, zero_constructor) {
  rosidl_generator_cpp::msg::PrimitivesDefault primitives_def(
    rosidl_generator_cpp::MessageInitialization::ZERO);
  ASSERT_FALSE(primitives_def.bool_value);
  ASSERT_EQ(0, primitives_def.byte_value);
  ASSERT_EQ(0, primitives_def.char_value);
  ASSERT_EQ(0.0f, primitives_def.float32_value);
  ASSERT_EQ(0.0, primitives_def.float64_value);
  ASSERT_EQ(0, primitives_def.int8_value);
  ASSERT_EQ(0, primitives_def.uint8_value);
  ASSERT_EQ(0, primitives_def.int16_value);
  ASSERT_EQ(0, primitives_def.uint16_value);
  ASSERT_EQ(0L, primitives_def.int32_value);
  ASSERT_EQ(0UL, primitives_def.uint32_value);
  ASSERT_EQ(0LL, primitives_def.int64_value);
  ASSERT_EQ(0ULL, primitives_def.uint64_value);
  ASSERT_EQ("", primitives_def.string_value);
}

TEST(Test_msg_initialization, defaults_only_constructor) {
  rosidl_generator_cpp::msg::PrimitivesDefault primitives_def(
    rosidl_generator_cpp::MessageInitialization::DEFAULTS_ONLY);
  ASSERT_TRUE(primitives_def.bool_value);
  ASSERT_EQ(50, primitives_def.byte_value);
  ASSERT_EQ(100, primitives_def.char_value);
  ASSERT_EQ(1.125f, primitives_def.float32_value);
  ASSERT_EQ(1.125, primitives_def.float64_value);
  ASSERT_EQ(-50, primitives_def.int8_value);
  ASSERT_EQ(200, primitives_def.uint8_value);
  ASSERT_EQ(-1000, primitives_def.int16_value);
  ASSERT_EQ(2000, primitives_def.uint16_value);
  ASSERT_EQ(-30000L, primitives_def.int32_value);
  ASSERT_EQ(60000UL, primitives_def.uint32_value);
  ASSERT_EQ(-40000000LL, primitives_def.int64_value);
  ASSERT_EQ(50000000ULL, primitives_def.uint64_value);
  ASSERT_EQ("bar", primitives_def.string_value);

  char * memory = new char[sizeof(rosidl_generator_cpp::msg::Various)];
  ASSERT_NE(memory, nullptr);
  std::memset(memory, 0xfe, sizeof(rosidl_generator_cpp::msg::Various));

  rosidl_generator_cpp::msg::Various * def =
    new(memory) rosidl_generator_cpp::msg::Various(
    rosidl_generator_cpp::MessageInitialization::DEFAULTS_ONLY);

  // ensures that the memory gets freed even if an ASSERT is raised
  SCOPE_EXIT(def->~Various_(); delete[] memory;);

  ASSERT_EQ(1.125f, def->float32_value_def);
  ASSERT_EQ(2.4, def->float64_value_def);
  ASSERT_EQ(0xfefefefefefefefeULL, def->uint64_value);
  ASSERT_EQ("bar", def->string_value_def);
  ASSERT_EQ(10UL, def->float32_arr.size());
#ifndef _WIN32
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif
  ASSERT_TRUE(std::all_of(def->float32_arr.begin(), def->float32_arr.end(), [](float i) {
      uint32_t float32_bit_pattern = *reinterpret_cast<uint32_t *>(&i);
      return 0xfefefefe == float32_bit_pattern;
    }));
#ifndef _WIN32
#pragma GCC diagnostic pop
#endif
  ASSERT_EQ(0xfefefefe, static_cast<uint32_t>(def->array_alignment_check));
  ASSERT_EQ(3UL, def->float64_arr_def.size());
  ASSERT_EQ(8.5, def->float64_arr_def[0]);
  ASSERT_EQ(1.2, def->float64_arr_def[1]);
  ASSERT_EQ(3.4, def->float64_arr_def[2]);
  ASSERT_EQ(3UL, def->string_arr.size());
  ASSERT_TRUE(std::all_of(def->string_arr.begin(), def->string_arr.end(), [](std::string i) {
      return "" == i;
    }));
  ASSERT_EQ(3UL, def->string_arr_def.size());
  ASSERT_EQ("hello", def->string_arr_def[0]);
  ASSERT_EQ("world", def->string_arr_def[1]);
  ASSERT_EQ("!", def->string_arr_def[2]);
  ASSERT_EQ(2UL, def->unbounded_seq_def.size());
  ASSERT_EQ(1.0f, def->unbounded_seq_def[0]);
  ASSERT_EQ(2.0f, def->unbounded_seq_def[1]);
  ASSERT_EQ(0UL, def->bounded_seq.size());
  ASSERT_EQ(2UL, def->bounded_seq_def.size());
  ASSERT_EQ(0xfefefefe, static_cast<uint32_t>(def->unbounded_alignment_check));
  ASSERT_EQ(3.0f, def->bounded_seq_def[0]);
  ASSERT_EQ(4.0f, def->bounded_seq_def[1]);
  ASSERT_EQ(0UL, def->bounded_seq.size());
  ASSERT_EQ(0xfefefefe, static_cast<uint32_t>(def->bounded_alignment_check));
}

// This is a test to ensure that when the user passes SKIP to the constructor,
// it does no initialization.
TEST(Test_msg_initialization, skip_constructor) {
  char * memory = new char[sizeof(rosidl_generator_cpp::msg::Various)];
  ASSERT_NE(memory, nullptr);
  std::memset(memory, 0xfe, sizeof(rosidl_generator_cpp::msg::Various));

  rosidl_generator_cpp::msg::Various * def =
    new(memory) rosidl_generator_cpp::msg::Various(
    rosidl_generator_cpp::MessageInitialization::SKIP);

  // ensures that the memory gets freed even if an ASSERT is raised
  SCOPE_EXIT(def->~Various_(); delete[] memory;);

#ifndef _WIN32
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif
  uint32_t float32_bit_pattern = *reinterpret_cast<uint32_t *>(&def->float32_value_def);
  ASSERT_EQ(0xfefefefe, float32_bit_pattern);
  uint64_t float64_bit_pattern = *reinterpret_cast<uint64_t *>(&def->float64_value_def);
  ASSERT_EQ(0xfefefefefefefefeULL, float64_bit_pattern);
  ASSERT_EQ(0xfefefefefefefefeULL, def->uint64_value);
  ASSERT_EQ("", def->string_value_def);
  ASSERT_TRUE(std::all_of(def->float32_arr.begin(), def->float32_arr.end(), [](float i) {
      uint32_t float32_bit_pattern = *reinterpret_cast<uint32_t *>(&i);
      return 0xfefefefe == float32_bit_pattern;
    }));
  ASSERT_EQ(0xfefefefe, static_cast<uint32_t>(def->array_alignment_check));
  ASSERT_TRUE(std::all_of(def->float64_arr_def.begin(), def->float64_arr_def.end(), [](double i) {
      uint64_t float64_bit_pattern = *reinterpret_cast<uint64_t *>(&i);
      return 0xfefefefefefefefe == float64_bit_pattern;
    }));
#ifndef _WIN32
#pragma GCC diagnostic pop
#endif
  ASSERT_TRUE(std::all_of(def->string_arr.begin(), def->string_arr.end(), [](std::string i) {
      return "" == i;
    }));
  ASSERT_TRUE(
    std::all_of(def->string_arr_def.begin(), def->string_arr_def.end(), [](std::string i) {
      return "" == i;
    }));
  // std::vector memory doesn't come from the memory we allocated above, but
  // instead comes from the allocator.  Thus, we don't expect it to be our
  // bitpattern for unbounded
  ASSERT_EQ(0UL, def->unbounded_seq_def.size());
  ASSERT_EQ(0UL, def->unbounded_seq.size());
  ASSERT_EQ(0xfefefefe, static_cast<uint32_t>(def->unbounded_alignment_check));
  // BoundedVector memory doesn't come from the memory we allocated above, but
  // instead comes from the allocator.  Thus, we don't expect it to be our
  // bitpattern for unbounded
  ASSERT_EQ(0UL, def->bounded_seq_def.size());
  ASSERT_EQ(0UL, def->bounded_seq.size());
  ASSERT_EQ(0xfefefefe, static_cast<uint32_t>(def->bounded_alignment_check));
}
