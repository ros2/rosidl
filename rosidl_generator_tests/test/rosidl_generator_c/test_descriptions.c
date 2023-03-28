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

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/type_description/type_source__struct.h"

#include "rosidl_generator_tests/msg/detail/defaults__struct.h"

int main(int argc, char ** argv)
{
  (void)argc;
  (void)argv;

  // Smoke test linkage for extern type description
  const rosidl_runtime_c__String * typename =
    &rosidl_generator_tests__msg__Defaults__TYPE_DESCRIPTION.type_description.type_name;
  // fprintf(stderr, "%s\n", typename->data);
  const char * expected_name = "rosidl_generator_tests/msg/Defaults";
  if (0 != memcmp(typename->data, expected_name, typename->size)) {
    fprintf(stderr, "Defaults typename incorrect.\n");
    return 1;
  }

  // Smoke test linkage for extern type sources
  const size_t expected_num_sources = 0;  // For now - this will fail when sources get added
  const rosidl_runtime_c__type_description__TypeSource__Sequence * sources =
    &rosidl_generator_tests__msg__Defaults__RAW_SOURCES;
  if (sources->size != expected_num_sources) {
    fprintf(stderr, "Defaults didn't have expected number of sources.\n");
    return 1;
  }

  return 0;
}
