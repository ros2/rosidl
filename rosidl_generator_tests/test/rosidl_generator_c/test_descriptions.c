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

#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/type_description/type_description__struct.h"
#include "rosidl_runtime_c/type_description/type_source__struct.h"

#include "rosidl_generator_tests/action/fibonacci.h"
#include "rosidl_generator_tests/msg/defaults.h"
#include "rosidl_generator_tests/srv/empty.h"


int test_basic(
  const rosidl_runtime_c__type_description__TypeDescription * description,
  const rosidl_runtime_c__type_description__TypeSource__Sequence * sources,
  const char * expected_name)
{
  const rosidl_runtime_c__String * typename = &description->type_description.type_name;
  if (0 != memcmp(typename->data, expected_name, typename->size)) {
    fprintf(stderr, "Typename incorrect, expected %s\n", expected_name);
    return 1;
  }
  if (sources->size != 0) {
    fprintf(stderr, "Encountered non-empty type sources unexpectedly.\n");
    return 1;
  }
  return 0;
}


int main(int argc, char ** argv)
{
  // Smoke test linkage and basic values for msg, srv, action
  (void)argc;
  (void)argv;

  // Message
  if (0 != test_basic(
      rosidl_generator_tests__msg__Defaults__get_type_description(NULL),
      rosidl_generator_tests__msg__Defaults__get_type_description_sources(NULL),
      "rosidl_generator_tests/msg/Defaults"))
  {
    return 1;
  }

  // Service
  if (0 != test_basic(
      rosidl_generator_tests__srv__Empty__get_type_description(NULL),
      rosidl_generator_tests__srv__Empty__get_type_description_sources(NULL),
      "rosidl_generator_tests/srv/Empty"))
  {
    return 1;
  }
  // Implicit message of a service
  if (0 != test_basic(
      rosidl_generator_tests__srv__Empty_Request__get_type_description(NULL),
      rosidl_generator_tests__srv__Empty_Request__get_type_description_sources(NULL),
      "rosidl_generator_tests/srv/Empty_Request"))
  {
    return 1;
  }

  // Action
  if (0 != test_basic(
      rosidl_generator_tests__action__Fibonacci__get_type_description(NULL),
      rosidl_generator_tests__action__Fibonacci__get_type_description_sources(NULL),
      "rosidl_generator_tests/action/Fibonacci"))
  {
    return 1;
  }
  // Implicit message of an action
  if (0 != test_basic(
      rosidl_generator_tests__action__Fibonacci_Feedback__get_type_description(NULL),
      rosidl_generator_tests__action__Fibonacci_Feedback__get_type_description_sources(NULL),
      "rosidl_generator_tests/action/Fibonacci_Feedback"))
  {
    return 1;
  }
  // Implicit service of an action
  if (0 != test_basic(
      rosidl_generator_tests__action__Fibonacci_SendGoal__get_type_description(NULL),
      rosidl_generator_tests__action__Fibonacci_SendGoal__get_type_description_sources(NULL),
      "rosidl_generator_tests/action/Fibonacci_SendGoal"))
  {
    return 1;
  }
  // Implicit message of implicit service of an action
  if (0 != test_basic(
      rosidl_generator_tests__action__Fibonacci_GetResult_Request__get_type_description(NULL),
      rosidl_generator_tests__action__Fibonacci_GetResult_Request__get_type_description_sources(
        NULL),
      "rosidl_generator_tests/action/Fibonacci_GetResult_Request"))
  {
    return 1;
  }

  return 0;
}
