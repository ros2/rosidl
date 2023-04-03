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
#include "rosidl_runtime_c/type_description/field__functions.h"
#include "rosidl_runtime_c/type_description/field_type__functions.h"
#include "rosidl_runtime_c/type_description/individual_type_description__functions.h"
#include "rosidl_runtime_c/type_description/key_value__functions.h"
#include "rosidl_runtime_c/type_description/type_description__functions.h"
#include "rosidl_runtime_c/type_description/type_description__struct.h"
#include "rosidl_runtime_c/type_description/type_source__functions.h"
#include "rosidl_runtime_c/type_description/type_source__struct.h"

#include "rosidl_generator_tests/action/fibonacci.h"
#include "rosidl_generator_tests/msg/defaults.h"
#include "rosidl_generator_tests/srv/empty.h"

#include "type_description_interfaces/msg/field.h"
#include "type_description_interfaces/msg/field_type.h"
#include "type_description_interfaces/msg/individual_type_description.h"
#include "type_description_interfaces/msg/key_value.h"
#include "type_description_interfaces/msg/type_description.h"
#include "type_description_interfaces/msg/type_source.h"


int description_namecheck(
  const rosidl_runtime_c__type_description__TypeDescription * description,
  const char * expected_name)
{
  const rosidl_runtime_c__String * typename = &description->type_description.type_name;
  if (0 != memcmp(typename->data, expected_name, typename->size)) {
    fprintf(stderr, "Typename incorrect, expected %s\n", expected_name);
    return 1;
  }
  return 0;
}

int test_description_linkage();
int test_copied_type_description_struct_hashes();
int test_source_defined();

int main(void)
{
  int rc = 0;
  printf("Testing rosidl_generator_tests description linkage...\n");
  if (test_description_linkage()) {
    fprintf(stderr, "test_description_linkage() FAILED\n");
    rc++;
  }
  printf("Testing rosidl_generator_tests copied type description struct hashes...\n");
  if (test_copied_type_description_struct_hashes()) {
    fprintf(stderr, "test_copied_type_description_struct_hashes() FAILED\n");
    rc++;
  }
  printf("Testing rosidl_generator_tests embedded raw sources...\n");
  if (test_source_defined()) {
    fprintf(stderr, "test_source_defined() FAILED\n");
    rc++;
  }

  if (rc != 0) {
    fprintf(stderr, "Some tests failed!\n");
  } else {
    printf("All tests were good!\n");
  }
  return rc != 0;
}

int test_description_linkage()
{
  // Smoke test linkage and basic values for msg, srv, action
  // Message
  if (0 != description_namecheck(
      rosidl_generator_tests__msg__Defaults__get_type_description(NULL),
      "rosidl_generator_tests/msg/Defaults"))
  {
    return 1;
  }

  // Service
  if (0 != description_namecheck(
      rosidl_generator_tests__srv__Empty__get_type_description(NULL),
      "rosidl_generator_tests/srv/Empty"))
  {
    return 1;
  }
  // Implicit message of a service
  if (0 != description_namecheck(
      rosidl_generator_tests__srv__Empty_Request__get_type_description(NULL),
      "rosidl_generator_tests/srv/Empty_Request"))
  {
    return 1;
  }

  // Action
  if (0 != description_namecheck(
      rosidl_generator_tests__action__Fibonacci__get_type_description(NULL),
      "rosidl_generator_tests/action/Fibonacci"))
  {
    return 1;
  }
  // Implicit message of an action
  if (0 != description_namecheck(
      rosidl_generator_tests__action__Fibonacci_Feedback__get_type_description(NULL),
      "rosidl_generator_tests/action/Fibonacci_Feedback"))
  {
    return 1;
  }
  // Implicit service of an action
  if (0 != description_namecheck(
      rosidl_generator_tests__action__Fibonacci_SendGoal__get_type_description(NULL),
      "rosidl_generator_tests/action/Fibonacci_SendGoal"))
  {
    return 1;
  }
  // Implicit message of implicit service of an action
  if (0 != description_namecheck(
      rosidl_generator_tests__action__Fibonacci_GetResult_Request__get_type_description(NULL),
      "rosidl_generator_tests/action/Fibonacci_GetResult_Request"))
  {
    return 1;
  }

  return 0;
}

int test_source_defined()
{
  // Smoke test that definitions are present for raw type sources
  // Message
  if (!rosidl_generator_tests__msg__Defaults__get_type_description_sources(NULL)) {
    return 1;
  }

  // Service
  if (!rosidl_generator_tests__srv__Empty__get_type_description_sources(NULL)) {
    return 1;
  }
  // Implicit message of a service
  if (!rosidl_generator_tests__srv__Empty_Response__get_type_description_sources(NULL)) {
    return 1;
  }

  // Action
  if (!rosidl_generator_tests__action__Fibonacci__get_type_description_sources(NULL)) {
    return 1;
  }
  // Implicit message of an action
  if (!rosidl_generator_tests__action__Fibonacci_Goal__get_type_description_sources(NULL)) {
    return 1;
  }
  // Implicit service of an action
  if (!rosidl_generator_tests__action__Fibonacci_GetResult__get_type_description_sources(NULL)) {
    return 1;
  }
  // Implicit message of implicit service of an action
  if (!rosidl_generator_tests__action__Fibonacci_SendGoal_Response__get_type_description_sources(NULL)) {
    return 1;
  }
  return 0;
}

int test_copied_type_description_struct_hashes()
{
  #define runtimehash(x) rosidl_runtime_c__type_description__ ## x ## __get_type_hash(NULL)
  #define msghash(x) type_description_interfaces__msg__ ## x ## __get_type_hash(NULL)
  #define hashcompare(x) memcmp(runtimehash(x), msghash(x), sizeof(rosidl_type_hash_t))
  int rc = 0;
  if (hashcompare(Field)) {
    fprintf(stderr, "Field hash NO MATCH\n");
    rc++;
  }
  if (hashcompare(FieldType)) {
    fprintf(stderr, "FieldType hash NO MATCH\n");
    rc++;
  }
  if (hashcompare(IndividualTypeDescription)) {
    fprintf(stderr, "IndividualTypeDescription hash NO MATCH\n");
    rc++;
  }
  if (hashcompare(KeyValue)) {
    fprintf(stderr, "KeyValue hash NO MATCH\n");
    rc++;
  }
  if (hashcompare(TypeDescription)) {
    fprintf(stderr, "TypeDescription hash NO MATCH\n");
    rc++;
  }
  if (hashcompare(TypeSource)) {
    fprintf(stderr, "TypeSource hash NO MATCH\n");
    rc++;
  }
  #undef hashcompare
  #undef msghash
  #undef runtimehash
  return rc;
}
