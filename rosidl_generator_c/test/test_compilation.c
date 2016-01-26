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

#include <stdio.h>
#include <string.h>

#include "rosidl_generator_c/msg/various__struct.h"
#include "rosidl_generator_c/msg/various__functions.h"

#include "./separate_compilation.h"

int main(int argc, char ** argv)
{
  (void)argc;
  (void)argv;

  if (!rosidl_generator_c__msg__Various__FOO) {
    fprintf(stderr, "wrong boolean constant\n");
    return 1;
  }

  if (rosidl_generator_c__msg__Various__BAZ != 42) {
    fprintf(stderr, "wrong integer constant\n");
    return 1;
  }

  int rc = func();

  if (!rc) {
    printf("all checks passed\n");
  }

  return 0;
}
