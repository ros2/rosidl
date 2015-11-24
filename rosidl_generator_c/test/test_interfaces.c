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

#include "rosidl_generator_c/msg/various.h"

int main(void)
{
  fprintf(stderr, "Testing rosidl_generator_c message types...\n");
  fprintf(stderr, "Testing Various message...\n");
  test_size_of_various();
  fprintf(stderr, "All tests were good!\n");
  return 0;
}

void test_size_of_various(void)
{
  rosidl_generator_c__msg__Various various;
  int expectSize;

  expectSize = sizeof(various.bool_value);
  expectSize += sizeof(various.byte_value);
  expectSize += sizeof(various.char_value);
  expectSize += sizeof(various.float32_value);
  expectSize += sizeof(various.float64_value);
  expectSize += sizeof(various.int8_value);
  expectSize += sizeof(various.two_uint16_value) * 2;
  expectSize += sizeof(rosidl_generator_c__int32__Array);
  expectSize += sizeof(rosidl_generator_c__int32__Array);
  expectSize += sizeof(rosidl_generator_c__uint64__Array);
  expectSize += sizeof(rosidl_generator_c__msg__Empty);
  expectSize += sizeof(rosidl_generator_c__msg__Empty) * 2;
  expectSize += sizeof(rosidl_generator_c__msg__Empty__Array);
  expectSize += sizeof(rosidl_generator_c__msg__Nested);
  expectSize += sizeof(rosidl_generator_c__msg__Nested) * 2;
  expectSize += sizeof(rosidl_generator_c__msg__Nested__Array);
  expectSize += sizeof(rosidl_generator_c__msg__Nested__Array);

  fprintf(stderr, "Expected size of Various = %d bytes\n", expectSize);
  fprintf(stderr, "Actual size of Various = %d bytes\n", sizeof(various));
  // the actual size can be a bit greater than the expected because of the word
  // boundary padding in the struct
  assert(sizeof(various) >= expectSize);
}
