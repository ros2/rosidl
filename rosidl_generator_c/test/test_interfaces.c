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


#include <assert.h>
#include <stdio.h>

#include "rosidl_generator_c/msg/empty.h"
#include "rosidl_generator_c/msg/primitives_static.h"


int main(void)
{
  fprintf(stderr, "Testing rosidl_generator_c message types...\n");
  fprintf(stderr, "Testing Empty message...\n");
  test_empty();
  fprintf(stderr, "Testing PrimitivesStatic message...\n");
  test_primitives_static();
  fprintf(stderr, "Testing rosidl_generator_c message types was successful!\n");
  return 0;
}

void test_empty(void)
{
  rosidl_generator_c__msg__Empty Empty;

  assert(0 == sizeof(Empty));
}

void test_primitives_static(void)
{
  rosidl_generator_c__msg__PrimitivesStatic PrimitivesStatic;

  assert(1 == sizeof(PrimitivesStatic.bool_value));
  assert(1 == sizeof(PrimitivesStatic.byte_value));
  assert(1 == sizeof(PrimitivesStatic.char_value));
  assert(4 == sizeof(PrimitivesStatic.float32_value));
  assert(8 == sizeof(PrimitivesStatic.float64_value));
  assert(1 == sizeof(PrimitivesStatic.int8_value));
  assert(1 == sizeof(PrimitivesStatic.uint8_value));
  assert(2 == sizeof(PrimitivesStatic.int16_value));
  assert(2 == sizeof(PrimitivesStatic.uint16_value));
  assert(4 == sizeof(PrimitivesStatic.int32_value));
  assert(4 == sizeof(PrimitivesStatic.uint32_value));
  assert(8 == sizeof(PrimitivesStatic.int64_value));
  assert(8 == sizeof(PrimitivesStatic.uint64_value));
}
