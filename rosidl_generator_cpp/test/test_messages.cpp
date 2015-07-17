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

#include <cstdio>
#include <type_traits>

#include <rosidl_generator_cpp/msg/empty.hpp>
#include <rosidl_generator_cpp/msg/primitives.hpp>
#include <rosidl_generator_cpp/msg/dynamic_array_primitives.hpp>
#include <rosidl_generator_cpp/msg/dynamic_array_nested.hpp>
#include <rosidl_generator_cpp/msg/static_array_primitives.hpp>
#include <rosidl_generator_cpp/msg/static_array_nested.hpp>
#include <rosidl_generator_cpp/msg/nested.hpp>

#include <rosidl_generator_cpp/msg/primitives_bounded.hpp>
#include <rosidl_generator_cpp/msg/static_array_bounded.hpp>
#include <rosidl_generator_cpp/msg/static_array_nested_bounded.hpp>
#include <rosidl_generator_cpp/msg/nested_bounded.hpp>

template<typename T1, bool B>
struct expect_bounded : std::enable_if<has_bounded_size<T1>::value == B, std::true_type>
{
};


int main(int argc, char ** argv)
{

  expect_bounded<rosidl_generator_cpp::msg::Empty, true>::type empty_test;

  expect_bounded<rosidl_generator_cpp::msg::Primitives, false>::type primitives_test;

  expect_bounded<rosidl_generator_cpp::msg::PrimitivesBounded, true>::type primitives_bounded_test;

  expect_bounded<rosidl_generator_cpp::msg::StaticArrayPrimitives, false>::type static_array_test;

  expect_bounded<rosidl_generator_cpp::msg::StaticArrayBounded, true>::type static_bounded_test;

  expect_bounded<rosidl_generator_cpp::msg::StaticArrayPrimitives, false>::type static_primitives_test;

  expect_bounded<rosidl_generator_cpp::msg::StaticArrayBounded, true>::type static_bounded;

  expect_bounded<rosidl_generator_cpp::msg::DynamicArrayPrimitives, false>::type dynamic_array_primitives;

  expect_bounded<rosidl_generator_cpp::msg::Nested, false>::type nested;

  expect_bounded<rosidl_generator_cpp::msg::NestedBounded, true>::type nested_bounded;

  expect_bounded<rosidl_generator_cpp::msg::DynamicArrayNested, false>::type dynamic_array_nested;

  expect_bounded<rosidl_generator_cpp::msg::StaticArrayNested, false>::type static_array_nested;

  expect_bounded<rosidl_generator_cpp::msg::StaticArrayNestedBounded, true>::type static_array_nested_bounded;

  // TODO: Check these values at compile time.

/*


  if (has_bounded_size<rosidl_generator_cpp::msg::Nested>::value) {
    fprintf(stderr, "Nested::has_bounded_size returned true!\n");
    retcode = 1;
  }

  if (!has_bounded_size<rosidl_generator_cpp::msg::NestedBounded>::value) {
    fprintf(stderr, "NestedBounded::has_bounded_size returned false!\n");
    retcode = 1;
  }

  if (has_bounded_size<rosidl_generator_cpp::msg::DynamicArrayNested>::value) {
    fprintf(stderr, "DynamicArrayNested::has_bounded_size returned true!\n");
    retcode = 1;
  }

  if (has_bounded_size<rosidl_generator_cpp::msg::StaticArrayNested>::value) {
    fprintf(stderr, "StaticArrayNested::has_bounded_size returned true!\n");
    retcode = 1;
  }

  if (!has_bounded_size<rosidl_generator_cpp::msg::StaticArrayNestedBounded>::value) {
    fprintf(stderr, "StaticArrayNestedBounded::has_bounded_size returned false!\n");
    retcode = 1;
  }

  if (retcode == 0) {
    fprintf(stderr, "All tests passed.\n");
  }

  return retcode;
*/
}
