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

#include <array>
#include <cassert>
#include <iostream>
#include <type_traits>

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

template<typename T1, bool B>
struct expect_fixed
  : std::enable_if<rosidl_generator_traits::has_fixed_size<T1>::value == B, bool>{};

int main(int /*argc*/, char ** /*argv*/)
{
  {
    const bool expectation = true;
    expect_fixed<rosidl_generator_cpp::msg::Empty, expectation>::type x = expectation;
    printf("Test passed: type Empty::has_fixed_size == %d\n", x);
  }

  {
    const bool expectation = true;
    expect_fixed<rosidl_generator_cpp::msg::PrimitivesStatic, expectation>::type x = expectation;
    printf("Test passed: type PrimitivesStatic::has_fixed_size == %d\n", x);
  }

  {
    const bool expectation = false;
    expect_fixed<rosidl_generator_cpp::msg::PrimitivesBounded, expectation>::type x = expectation;
    printf("Test passed: type PrimitivesBounded::has_fixed_size == %d\n", x);
  }

  {
    const bool expectation = false;
    expect_fixed<rosidl_generator_cpp::msg::PrimitivesUnbounded, expectation>::type x = expectation;
    printf("Test passed: type PrimitivesUnbounded::has_fixed_size == %d\n", x);
  }

  {
    const bool expectation = true;
    expect_fixed<rosidl_generator_cpp::msg::PrimitiveStaticArrays,
    expectation>::type x = expectation;
    printf("Test passed: type PrimitivesStaticArrays::has_fixed_size == %d\n", x);
  }

  {
    const bool expectation = true;
    expect_fixed<rosidl_generator_cpp::msg::StaticArrayStatic, expectation>::type x = expectation;
    printf("Test passed: type StaticArrayStatic::has_fixed_size == %d\n", x);
  }

  {
    const bool expectation = false;
    expect_fixed<rosidl_generator_cpp::msg::StaticArrayBounded, expectation>::type x = expectation;
    printf("Test passed: type StaticArrayBounded::has_fixed_size == %d\n", x);
  }

  {
    const bool expectation = false;
    expect_fixed<rosidl_generator_cpp::msg::StaticArrayUnbounded,
    expectation>::type x = expectation;
    printf("Test passed: type StaticArrayUnbounded::has_fixed_size == %d\n", x);
  }

  {
    const bool expectation = false;
    expect_fixed<rosidl_generator_cpp::msg::BoundedArrayStatic, expectation>::type x = expectation;
    printf("Test passed: type BoundedArrayStatic::has_fixed_size == %d\n", x);
  }

  {
    const bool expectation = false;
    expect_fixed<rosidl_generator_cpp::msg::BoundedArrayBounded, expectation>::type x = expectation;
    printf("Test passed: type BoundedArrayBounded::has_fixed_size == %d\n", x);
  }

  {
    const bool expectation = false;
    expect_fixed<rosidl_generator_cpp::msg::BoundedArrayUnbounded,
    expectation>::type x = expectation;
    printf("Test passed: type BoundedArrayUnbounded::has_fixed_size == %d\n", x);
  }

  {
    const bool expectation = false;
    expect_fixed<rosidl_generator_cpp::msg::UnboundedArrayStatic,
    expectation>::type x = expectation;
    printf("Test passed: type UnboundedArrayStatic::has_fixed_size == %d\n", x);
  }

  {
    const bool expectation = false;
    expect_fixed<rosidl_generator_cpp::msg::UnboundedArrayBounded,
    expectation>::type x = expectation;
    printf("Test passed: type UnboundedArrayBounded::has_fixed_size == %d\n", x);
  }

  {
    const bool expectation = false;
    expect_fixed<rosidl_generator_cpp::msg::UnboundedArrayUnbounded,
    expectation>::type x = expectation;
    printf("Test passed: type UnboundedArrayUnbounded::has_fixed_size == %d\n", x);
  }

  // Showing that sizeof returns accurate values for messages containing static arrays
  size_t primitive_size = sizeof(rosidl_generator_cpp::msg::PrimitivesStatic);
  size_t array_primitives_size = sizeof(rosidl_generator_cpp::msg::StaticArrayStatic);
  size_t primitive_static_array_size = sizeof(rosidl_generator_cpp::msg::PrimitiveStaticArrays);

  assert(array_primitives_size == 3 * primitive_size);
  assert(primitive_static_array_size == 3 * primitive_size);

  fprintf(stderr, "All message tests passed.\n");
  return 0;
}
