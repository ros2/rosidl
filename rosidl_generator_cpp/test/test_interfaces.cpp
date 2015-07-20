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

#include <iostream>
#include <type_traits>

#include <rosidl_generator_cpp/msg/empty.hpp>

#include <rosidl_generator_cpp/msg/bounded_array_bounded.hpp>
#include <rosidl_generator_cpp/msg/bounded_array_static.hpp>
#include <rosidl_generator_cpp/msg/bounded_array_unbounded.hpp>

#include <rosidl_generator_cpp/msg/primitives_bounded.hpp>
#include <rosidl_generator_cpp/msg/primitives_static.hpp>
#include <rosidl_generator_cpp/msg/primitives_unbounded.hpp>

#include <rosidl_generator_cpp/msg/static_array_bounded.hpp>
#include <rosidl_generator_cpp/msg/static_array_static.hpp>
#include <rosidl_generator_cpp/msg/static_array_unbounded.hpp>

#include <rosidl_generator_cpp/msg/unbounded_array_bounded.hpp>
#include <rosidl_generator_cpp/msg/unbounded_array_static.hpp>
#include <rosidl_generator_cpp/msg/unbounded_array_unbounded.hpp>


template<typename T1, bool B>
struct expect_fixed
  : std::enable_if<has_fixed_size<T1>::value == B, std::true_type>{};

template<typename T1, bool B>
struct expect_bounded
  : std::enable_if<has_bounded_size<T1>::value == B, std::true_type>{};

/*
Covered cases:
Empty message (fixed, bounded)

PrimitivesStatic: Contains static (fixed, bounded)
PrimitivesBounded: Contains bounded, dynamic (!fixed, bounded)
PrimitivesUnbounded: Contains unbounded, dynamic (!fixed, !bounded)

(Nested case)
Static arrays of:
PrimitivesStatic (StaticArrayStatic) (fixed, bounded)
PrimitivesBounded (StaticArrayBounded) (!fixed, bounded)
PrimitivesUnbounded (StaticArrayUnbounded) (!fixed, !bounded)

Bounded arrays of:
PrimitivesStatic (BoundedArrayStatic) (!fixed, bounded)
PrimitivesBounded (BoundedArrayBounded) (!fixed, bounded)
PrimitivesUnounded (BoundedArrayUnbounded) (!fixed, !bounded)

Unbounded arrays of:
PrimitivesStatic (UnboundedArrayStatic) (!fixed, !bounded)
PrimitivesBounded (UnboundedArrayBounded) (!fixed, !bounded)
PrimitivesUnbounded (UnboundedArrayUnbounded) (!fixed, !bounded)
*/

int main(int argc, char ** argv)
{
  { expect_fixed<rosidl_generator_cpp::msg::Empty, true>::type x; }
  { expect_bounded<rosidl_generator_cpp::msg::Empty, true>::type x; }

  { expect_fixed<rosidl_generator_cpp::msg::PrimitivesStatic, true>::type x; }
  { expect_bounded<rosidl_generator_cpp::msg::PrimitivesStatic, true>::type x; }

  { expect_fixed<rosidl_generator_cpp::msg::PrimitivesBounded, false>::type x; }
  { expect_bounded<rosidl_generator_cpp::msg::PrimitivesBounded, true>::type x; }

  { expect_fixed<rosidl_generator_cpp::msg::PrimitivesUnbounded, false>::type x; }
  { expect_bounded<rosidl_generator_cpp::msg::PrimitivesUnbounded, false>::type x; }

  { expect_fixed<rosidl_generator_cpp::msg::StaticArrayStatic, true>::type x; }
  { expect_bounded<rosidl_generator_cpp::msg::StaticArrayStatic, true>::type x; }

  { expect_fixed<rosidl_generator_cpp::msg::StaticArrayBounded, false>::type x; }
  { expect_bounded<rosidl_generator_cpp::msg::StaticArrayBounded, true>::type x; }

  { expect_fixed<rosidl_generator_cpp::msg::StaticArrayUnbounded, false>::type x; }
  { expect_bounded<rosidl_generator_cpp::msg::StaticArrayUnbounded, false>::type x; }

  { expect_fixed<rosidl_generator_cpp::msg::BoundedArrayStatic, false>::type x; }
  { expect_bounded<rosidl_generator_cpp::msg::BoundedArrayStatic, true>::type x; }

  { expect_fixed<rosidl_generator_cpp::msg::BoundedArrayBounded, false>::type x; }
  { expect_bounded<rosidl_generator_cpp::msg::BoundedArrayBounded, true>::type x; }

  { expect_fixed<rosidl_generator_cpp::msg::BoundedArrayUnbounded, false>::type x; }
  { expect_bounded<rosidl_generator_cpp::msg::BoundedArrayUnbounded, false>::type x; }

  { expect_fixed<rosidl_generator_cpp::msg::UnboundedArrayStatic, false>::type x; }
  { expect_bounded<rosidl_generator_cpp::msg::UnboundedArrayStatic, false>::type x; }

  { expect_fixed<rosidl_generator_cpp::msg::UnboundedArrayBounded, false>::type x; }
  { expect_bounded<rosidl_generator_cpp::msg::UnboundedArrayBounded, false>::type x; }

  { expect_fixed<rosidl_generator_cpp::msg::UnboundedArrayUnbounded, false>::type x; }
  { expect_bounded<rosidl_generator_cpp::msg::UnboundedArrayUnbounded, false>::type x; }

  size_t primitive_size_diff = bounded_size<rosidl_generator_cpp::msg::PrimitivesBounded>::value -
    bounded_size<rosidl_generator_cpp::msg::PrimitivesStatic>::value;
  if (primitive_size_diff != 10 + sizeof(std::string)) {
    fprintf(stderr, "Computed incorrect bounded_size for PrimitivesStatic!\n");
    return 1;
  }

  size_t nested_size_diff = bounded_size<rosidl_generator_cpp::msg::StaticArrayBounded>::value -
    bounded_size<rosidl_generator_cpp::msg::StaticArrayStatic>::value;
  if (nested_size_diff != (10 + sizeof(std::string))*3) {
    fprintf(stderr, "Computed incorrect bounded_size for NestedBounded!\n");
    return 1;
  }

  fprintf(stderr, "All message tests passed.\n");
  return 0;
}
