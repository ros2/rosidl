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

int main(int argc, char ** argv)
{

  int retcode = 0;
  if (!has_bounded_size<rosidl_generator_cpp::msg::Empty>::value) {
    fprintf(stderr, "Empty::has_bounded_size returned false!\n");
    retcode = 1;
  }

  if (has_bounded_size<rosidl_generator_cpp::msg::Primitives>::value) {
    fprintf(stderr, "Primitives::has_bounded_size returned true!\n");
    retcode = 1;
  }

  if (!has_bounded_size<rosidl_generator_cpp::msg::PrimitivesBounded>::value) {
    fprintf(stderr, "PrimitivesBounded::has_bounded_size returned false!\n");
    retcode = 1;
  }

  if (has_bounded_size<rosidl_generator_cpp::msg::StaticArrayPrimitives>::value) {
    fprintf(stderr, "StaticArrayPrimitives::has_bounded_size returned true!\n");
    retcode = 1;
  }

  if (!has_bounded_size<rosidl_generator_cpp::msg::StaticArrayBounded>::value) {
    fprintf(stderr, "StaticArrayBounded::has_bounded_size returned false!\n");
    retcode = 1;
  }

  if (has_bounded_size<rosidl_generator_cpp::msg::DynamicArrayPrimitives>::value) {
    fprintf(stderr, "DynamicArrayPrimitives::has_bounded_size returned true!\n");
    retcode = 1;
  }

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
}
