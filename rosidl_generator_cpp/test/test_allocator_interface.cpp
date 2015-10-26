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

#include <cassert>
#include <iostream>
#include <memory>

#include "rosidl_generator_cpp/msg/primitives_unbounded.hpp"
#include "null_allocator.hpp"

using namespace rosidl_generator_cpp;
using namespace rosidl_generator_cpp::test;

int main(int /*argc*/, char ** /*argv*/)
{
  // Try to instantiate messages with a custom allocator
  msg::PrimitivesUnbounded_<test::null_allocator<void>> primitives;

  // TODO: Assert that the fields have the correct allocator types

  (void) primitives;

  fprintf(stderr, "All message tests passed.\n");
  return 0;
}
