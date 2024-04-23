// Copyright 2023 Open Source Robotics Foundation, Inc.
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
#include "rosidl_generator_tests/msg/basic_idl.h"
#include "rosidl_generator_tests/msg/defaults.h"
#include "rosidl_generator_tests/msg/empty.h"
#include "rosidl_generator_tests/srv/empty.h"

#include "type_description_interfaces/msg/field.h"
#include "type_description_interfaces/msg/field_type.h"
#include "type_description_interfaces/msg/individual_type_description.h"
#include "type_description_interfaces/msg/key_value.h"
#include "type_description_interfaces/msg/type_description.h"
#include "type_description_interfaces/msg/type_source.h"


bool description_namecheck(
  const rosidl_runtime_c__type_description__TypeDescription * description,
  const char * expected_name)
{
  const rosidl_runtime_c__String * typename = &description->type_description.type_name;
  if (0 != memcmp(typename->data, expected_name, typename->size)) {
    fprintf(stderr, "Typename incorrect, expected %s\n", expected_name);
    return false;
  }
  return true;
}

bool string_char_equal(const rosidl_runtime_c__String * lhs, const char * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  size_t rhs_len = strlen(rhs);
  if (lhs->size != rhs_len) {
    return false;
  }
  return 0 == memcmp(lhs->data, rhs, rhs_len);
}

int test_description_linkage(void);
int test_copied_type_description_struct_hashes(void);
int test_source_defined(void);
int test_same_name_types(void);

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
  printf("Testing same named types...\n");
  if (test_same_name_types()) {
    fprintf(stderr, "test_same_name_types() FAILED\n");
    rc++;
  }

  if (rc != 0) {
    fprintf(stderr, "Some tests failed!\n");
  } else {
    printf("All tests were good!\n");
  }
  return rc != 0;
}

int test_description_linkage(void)
{
  // Smoke test linkage and basic values for msg, srv, action
  // Message
  if (!description_namecheck(
      rosidl_generator_tests__msg__Defaults__get_type_description(NULL),
      "rosidl_generator_tests/msg/Defaults"))
  {
    fprintf(stderr, "Defaults.msg description name mismatch\n");
    return 1;
  }

  // Service
  if (!description_namecheck(
      rosidl_generator_tests__srv__Empty__get_type_description(NULL),
      "rosidl_generator_tests/srv/Empty"))
  {
    fprintf(stderr, "Empty.srv description name mismatch\n");
    return 1;
  }
  // Implicit message of a service
  if (!description_namecheck(
      rosidl_generator_tests__srv__Empty_Request__get_type_description(NULL),
      "rosidl_generator_tests/srv/Empty_Request"))
  {
    fprintf(stderr, "Empty.srv Request description name mismatch\n");
    return 1;
  }

  // Action
  if (!description_namecheck(
      rosidl_generator_tests__action__Fibonacci__get_type_description(NULL),
      "rosidl_generator_tests/action/Fibonacci"))
  {
    fprintf(stderr, "Fibonacci.action description name mismatch\n");
    return 1;
  }
  // Implicit message of an action
  if (!description_namecheck(
      rosidl_generator_tests__action__Fibonacci_Feedback__get_type_description(NULL),
      "rosidl_generator_tests/action/Fibonacci_Feedback"))
  {
    fprintf(stderr, "Fibonacci.action Feedback msg description name mismatch\n");
    return 1;
  }
  // Implicit service of an action
  if (!description_namecheck(
      rosidl_generator_tests__action__Fibonacci_SendGoal__get_type_description(NULL),
      "rosidl_generator_tests/action/Fibonacci_SendGoal"))
  {
    fprintf(stderr, "Fibonacci.action SendGoal srv description name mismatch\n");
    return 1;
  }
  // Implicit message of implicit service of an action
  if (!description_namecheck(
      rosidl_generator_tests__action__Fibonacci_GetResult_Request__get_type_description(NULL),
      "rosidl_generator_tests/action/Fibonacci_GetResult_Request"))
  {
    fprintf(stderr, "Fibonacci.action GetResult srv Request description name mismatch\n");
    return 1;
  }

  return 0;
}

int test_source_defined(void)
{
  // Smoke test that definitions are present for raw type sources
  // Message
  const rosidl_runtime_c__type_description__TypeSource__Sequence * defaults_sources =
    rosidl_generator_tests__msg__Defaults__get_type_description_sources(NULL);
  if (!defaults_sources) {
    fprintf(stderr, "Defaults.msg raw sources undefined\n");
    return 1;
  }
  if (defaults_sources->size != 1) {
    fprintf(stderr, "Defaults.msg expected exactly 1 raw sources\n");
    return 1;
  }
  const rosidl_runtime_c__type_description__TypeSource * check_src = defaults_sources->data;
  if (!string_char_equal(&check_src->type_name, "rosidl_generator_tests/msg/Defaults")) {
    fprintf(stderr, "Defaults.msg source name not as expected\n");
    return 1;
  }
  if (!string_char_equal(&check_src->encoding, "msg")) {
    fprintf(stderr, "Defaults.msg source not encoded as msg\n");
    return 1;
  }
  if (0 == check_src->raw_file_contents.size) {
    fprintf(stderr, "Defaults.msg raw contents empty\n");
    return 1;
  }

  // Service
  // Hardcoding a bit of knowledge about an empty srv - it has 5 referenced type descriptions:
  // builtin_interfaces/msg/Time
  // rosidl_generator_tests/srv/Empty_Event
  // rosidl_generator_tests/srv/Empty_Request
  // rosidl_generator_tests/srv/Empty_Response
  // service_msgs/msg/ServiceEventInfo
  const rosidl_runtime_c__type_description__TypeSource__Sequence * empty_srv_sources =
    rosidl_generator_tests__srv__Empty__get_type_description_sources(NULL);
  if (!empty_srv_sources) {
    fprintf(stderr, "Empty.srv raw sources undefined\n");
    return 1;
  }
  if (empty_srv_sources->size != 6) {
    fprintf(stderr, "Empty.srv expected exactly 6 raw sources\n");
    return 1;
  }
  check_src = empty_srv_sources->data;
  if (!string_char_equal(&check_src->type_name, "rosidl_generator_tests/srv/Empty")) {
    fprintf(stderr, "Empty.srv source name '%s' not as expected\n", check_src->type_name.data);
    return 1;
  }
  if (!string_char_equal(&check_src->encoding, "srv")) {
    fprintf(stderr, "Empty.srv source encoding '%s' is not 'srv'\n", check_src->encoding.data);
    return 1;
  }
  if (0 == check_src->raw_file_contents.size) {
    fprintf(stderr, "Empty.srv raw contents empty\n");
    return 1;
  }
  check_src = &empty_srv_sources->data[2];  // Looking past Time to the implicit Event
  if (!string_char_equal(&check_src->type_name, "rosidl_generator_tests/srv/Empty_Event")) {
    fprintf(stderr, "Empty.srv Event src name '%s' not as expected\n", check_src->type_name.data);
    return 1;
  }
  if (!string_char_equal(&check_src->encoding, "implicit")) {
    fprintf(stderr, "Defaults first source not encoded as implicit\n");
    return 1;
  }
  if (0 != check_src->raw_file_contents.size) {
    fprintf(stderr, "Empty.srv Request implicit source not empty.\n");
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
  if (!rosidl_generator_tests__action__Fibonacci_SendGoal_Response__get_type_description_sources(
      NULL))
  {
    return 1;
  }

  // IDL
  check_src = rosidl_generator_tests__msg__BasicIdl__get_individual_type_description_source(NULL);
  if (!check_src) {
    fprintf(stderr, "BasicIdl.idl sources not available.\n");
    return 1;
  }
  if (!string_char_equal(&check_src->encoding, "idl")) {
    fprintf(stderr, "BasicIdl.idl source encoding '%s' is not 'idl'\n", check_src->encoding.data);
    return 1;
  }

  return 0;
}

int test_copied_type_description_struct_hashes(void)
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

int test_same_name_types(void)
{
  // Msg and srv with same name in this package
  // Regression check case, this was receiving "srv" encoding with Empty.srv sources
  const rosidl_runtime_c__type_description__TypeSource * empty_msg_src =
    rosidl_generator_tests__msg__Empty__get_individual_type_description_source(NULL);
  if (!string_char_equal(&empty_msg_src->type_name, "rosidl_generator_tests/msg/Empty")) {
    fprintf(stderr, "Empty.msg source name not as expected\n");
    return 1;
  }
  if (!string_char_equal(&empty_msg_src->encoding, "msg")) {
    fprintf(stderr, "Empty.msg source not encoded as msg\n");
    return 1;
  }

  const rosidl_runtime_c__type_description__TypeSource * empty_srv_src =
    rosidl_generator_tests__srv__Empty__get_individual_type_description_source(NULL);
  if (!string_char_equal(&empty_srv_src->type_name, "rosidl_generator_tests/srv/Empty")) {
    fprintf(stderr, "Empty.srv source name not as expected\n");
    return 1;
  }
  if (!string_char_equal(&empty_srv_src->encoding, "srv")) {
    fprintf(stderr, "Empty.srv source not encoded as srv\n");
    return 1;
  }
  return 0;
}
