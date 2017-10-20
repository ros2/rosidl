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

#include "rosidl_generator_cpp/msg/primitives_some_default.hpp"

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
  rosidl_generator_cpp::msg::PrimitivesSomeDefault def;

  ASSERT_EQ(1.125f, def.float32_value);
  ASSERT_EQ(2.4, def.float64_value);
  ASSERT_EQ(0ULL, def.uint64_value);
  ASSERT_EQ("bar", def.string_value);
  ASSERT_TRUE(std::all_of(def.float32_arr.begin(), def.float32_arr.end(), [](float i) {
      return 0.0f == i;
    }));
  ASSERT_EQ(8.5, def.float64_arr[0]);
  ASSERT_EQ(1.2, def.float64_arr[1]);
  ASSERT_EQ(3.4, def.float64_arr[2]);
  ASSERT_TRUE(std::all_of(def.string_arr.begin(), def.string_arr.end(), [](std::string i) {
      return "" == i;
    }));
  ASSERT_EQ(2UL, def.unbounded.size());
  ASSERT_EQ(1.0f, def.unbounded[0]);
  ASSERT_EQ(2.0f, def.unbounded[1]);
  ASSERT_EQ(0UL, def.bounded_no_def.size());
  ASSERT_EQ(2UL, def.bounded_def.size());
  ASSERT_EQ(3.0f, def.bounded_def[0]);
  ASSERT_EQ(4.0f, def.bounded_def[1]);
  ASSERT_EQ(0UL, def.vec3.x);
  ASSERT_EQ(45UL, def.vec3.y);
  ASSERT_EQ(0UL, def.vec3.z);
  ASSERT_EQ(2UL, def.vec3_fixed.size());
  ASSERT_EQ(0UL, def.vec3_fixed[0].x);
  ASSERT_EQ(0UL, def.vec3_fixed[1].x);
  ASSERT_EQ(45UL, def.vec3_fixed[0].y);
  ASSERT_EQ(45UL, def.vec3_fixed[1].y);
  ASSERT_EQ(0UL, def.vec3_fixed[0].z);
  ASSERT_EQ(0UL, def.vec3_fixed[1].z);
  ASSERT_EQ(0UL, def.vec3_unbounded.size());
  ASSERT_EQ(0UL, def.vec3_bounded.size());
}

TEST(Test_msg_initialization, all_constructor) {
  rosidl_generator_cpp::msg::PrimitivesSomeDefault def(
    rosidl_generator_cpp::MessageInitialization::ALL);

  ASSERT_EQ(1.125f, def.float32_value);
  ASSERT_EQ(2.4, def.float64_value);
  ASSERT_EQ(0ULL, def.uint64_value);
  ASSERT_EQ("bar", def.string_value);
  ASSERT_TRUE(std::all_of(def.float32_arr.begin(), def.float32_arr.end(), [](float i) {
      return 0.0f == i;
    }));
  ASSERT_EQ(8.5, def.float64_arr[0]);
  ASSERT_EQ(1.2, def.float64_arr[1]);
  ASSERT_EQ(3.4, def.float64_arr[2]);
  ASSERT_TRUE(std::all_of(def.string_arr.begin(), def.string_arr.end(), [](std::string i) {
      return "" == i;
    }));
  ASSERT_EQ(2UL, def.unbounded.size());
  ASSERT_EQ(1.0f, def.unbounded[0]);
  ASSERT_EQ(2.0f, def.unbounded[1]);
  ASSERT_EQ(0UL, def.bounded_no_def.size());
  ASSERT_EQ(2UL, def.bounded_def.size());
  ASSERT_EQ(3.0f, def.bounded_def[0]);
  ASSERT_EQ(4.0f, def.bounded_def[1]);
  ASSERT_EQ(0UL, def.vec3.x);
  ASSERT_EQ(45UL, def.vec3.y);
  ASSERT_EQ(0UL, def.vec3.z);
  ASSERT_EQ(2UL, def.vec3_fixed.size());
  ASSERT_EQ(0UL, def.vec3_fixed[0].x);
  ASSERT_EQ(0UL, def.vec3_fixed[1].x);
  ASSERT_EQ(45UL, def.vec3_fixed[0].y);
  ASSERT_EQ(45UL, def.vec3_fixed[1].y);
  ASSERT_EQ(0UL, def.vec3_fixed[0].z);
  ASSERT_EQ(0UL, def.vec3_fixed[1].z);
  ASSERT_EQ(0UL, def.vec3_unbounded.size());
  ASSERT_EQ(0UL, def.vec3_bounded.size());
}

TEST(Test_msg_initialization, zero_constructor) {
  rosidl_generator_cpp::msg::PrimitivesSomeDefault def(
    rosidl_generator_cpp::MessageInitialization::ZERO);

  ASSERT_EQ(0.0f, def.float32_value);
  ASSERT_EQ(0.0, def.float64_value);
  ASSERT_EQ(0ULL, def.uint64_value);
  ASSERT_EQ("", def.string_value);
  ASSERT_TRUE(std::all_of(def.float32_arr.begin(), def.float32_arr.end(), [](float i) {
      return 0.0f == i;
    }));
  ASSERT_TRUE(std::all_of(def.float64_arr.begin(), def.float64_arr.end(), [](double i) {
      return 0.0 == i;
    }));
  ASSERT_TRUE(std::all_of(def.string_arr.begin(), def.string_arr.end(), [](std::string i) {
      return "" == i;
    }));
  ASSERT_EQ(2UL, def.unbounded.size());
  ASSERT_EQ(0.0f, def.unbounded[0]);
  ASSERT_EQ(0.0f, def.unbounded[1]);
  ASSERT_EQ(0UL, def.bounded_no_def.size());
  ASSERT_EQ(2UL, def.bounded_def.size());
  ASSERT_EQ(0.0f, def.bounded_def[0]);
  ASSERT_EQ(0.0f, def.bounded_def[1]);
  ASSERT_EQ(0UL, def.vec3.x);
  ASSERT_EQ(0UL, def.vec3.y);
  ASSERT_EQ(0UL, def.vec3.z);
  ASSERT_EQ(2UL, def.vec3_fixed.size());
  ASSERT_EQ(0UL, def.vec3_fixed[0].x);
  ASSERT_EQ(0UL, def.vec3_fixed[1].x);
  ASSERT_EQ(0UL, def.vec3_fixed[0].y);
  ASSERT_EQ(0UL, def.vec3_fixed[1].y);
  ASSERT_EQ(0UL, def.vec3_fixed[0].z);
  ASSERT_EQ(0UL, def.vec3_fixed[1].z);
  ASSERT_EQ(0UL, def.vec3_unbounded.size());
  ASSERT_EQ(0UL, def.vec3_bounded.size());
}

TEST(Test_msg_initialization, defaults_only_constructor) {
  char * memory = new char[sizeof(rosidl_generator_cpp::msg::PrimitivesSomeDefault)];
  ASSERT_NE(memory, nullptr);
  std::memset(memory, 0xfe, sizeof(rosidl_generator_cpp::msg::PrimitivesSomeDefault));

  rosidl_generator_cpp::msg::PrimitivesSomeDefault * def =
    new(memory) rosidl_generator_cpp::msg::PrimitivesSomeDefault(
    rosidl_generator_cpp::MessageInitialization::DEFAULTS_ONLY);

  // ensures that the memory gets freed even if an ASSERT is raised
  SCOPE_EXIT(def->~PrimitivesSomeDefault_(); delete[] memory;);

  ASSERT_EQ(1.125f, def->float32_value);
  ASSERT_EQ(2.4, def->float64_value);
  ASSERT_EQ(0xfefefefefefefefeULL, def->uint64_value);
  ASSERT_EQ("bar", def->string_value);
  ASSERT_TRUE(std::all_of(def->float32_arr.begin(), def->float32_arr.end(), [](float i) {
      uint32_t float32_bit_pattern = *reinterpret_cast<uint32_t *>(&i);
      return 0xfefefefe == float32_bit_pattern;
    }));
  ASSERT_EQ(8.5, def->float64_arr[0]);
  ASSERT_EQ(1.2, def->float64_arr[1]);
  ASSERT_EQ(3.4, def->float64_arr[2]);
  ASSERT_TRUE(std::all_of(def->string_arr.begin(), def->string_arr.end(), [](std::string i) {
      return "" == i;
    }));
  ASSERT_EQ(1.0f, def->unbounded[0]);
  ASSERT_EQ(2.0f, def->unbounded[1]);
  ASSERT_EQ(0UL, def->bounded_no_def.size());
  ASSERT_EQ(2UL, def->bounded_def.size());
  ASSERT_EQ(3.0f, def->bounded_def[0]);
  ASSERT_EQ(4.0f, def->bounded_def[1]);
  ASSERT_EQ(0xfefefefe, def->vec3.x);
  ASSERT_EQ(45UL, def->vec3.y);
  ASSERT_EQ(0xfefefefe, def->vec3.z);
  ASSERT_EQ(2UL, def->vec3_fixed.size());
  ASSERT_EQ(45UL, def->vec3_fixed[0].y);
  ASSERT_EQ(45UL, def->vec3_fixed[1].y);
  ASSERT_EQ(0UL, def->vec3_unbounded.size());
  ASSERT_EQ(0UL, def->vec3_bounded.size());
}

// This is a test to ensure that when the user passes SKIP to the constructor,
// it does no initialization.
TEST(Test_msg_initialization, skip_constructor) {
  char * memory = new char[sizeof(rosidl_generator_cpp::msg::PrimitivesSomeDefault)];
  ASSERT_NE(memory, nullptr);
  std::memset(memory, 0xfe, sizeof(rosidl_generator_cpp::msg::PrimitivesSomeDefault));

  rosidl_generator_cpp::msg::PrimitivesSomeDefault * def =
    new(memory) rosidl_generator_cpp::msg::PrimitivesSomeDefault(
    rosidl_generator_cpp::MessageInitialization::SKIP);

  // ensures that the memory gets freed even if an ASSERT is raised
  SCOPE_EXIT(def->~PrimitivesSomeDefault_(); delete[] memory;);

  uint32_t float32_bit_pattern = *reinterpret_cast<uint32_t *>(&def->float32_value);
  ASSERT_EQ(0xfefefefe, float32_bit_pattern);
  uint64_t float64_bit_pattern = *reinterpret_cast<uint64_t *>(&def->float64_value);
  ASSERT_EQ(0xfefefefefefefefeULL, float64_bit_pattern);
  ASSERT_EQ(0xfefefefefefefefeULL, def->uint64_value);
  ASSERT_EQ("", def->string_value);
  ASSERT_TRUE(std::all_of(def->float32_arr.begin(), def->float32_arr.end(), [](float i) {
      uint32_t float32_bit_pattern = *reinterpret_cast<uint32_t *>(&i);
      return 0xfefefefe == float32_bit_pattern;
    }));
  ASSERT_TRUE(std::all_of(def->float64_arr.begin(), def->float64_arr.end(), [](double i) {
      uint64_t float64_bit_pattern = *reinterpret_cast<uint64_t *>(&i);
      return 0xfefefefefefefefe == float64_bit_pattern;
    }));
  ASSERT_TRUE(std::all_of(def->string_arr.begin(), def->string_arr.end(), [](std::string i) {
      return "" == i;
    }));
  // std::vector memory doesn't come from the memory we allocated above, but
  // instead comes from the allocator.  Thus, we don't expect it to be our
  // bitpattern for unbounded
  ASSERT_EQ(0UL, def->bounded_no_def.size());
  // BoundedVector memory doesn't come from the memory we allocated above, but
  // instead comes from the allocator.  Thus, we don't expect it to be our
  // bitpattern for unbounded
  ASSERT_EQ(0UL, def->bounded_def.size());
  ASSERT_EQ(0xfefefefe, def->vec3.x);
  ASSERT_EQ(0xfefefefe, def->vec3.y);
  ASSERT_EQ(0xfefefefe, def->vec3.z);
  ASSERT_EQ(2UL, def->vec3_fixed.size());
  ASSERT_EQ(0UL, def->vec3_unbounded.size());
  ASSERT_EQ(0UL, def->vec3_bounded.size());
}
