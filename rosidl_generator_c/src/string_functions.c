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

#include "rosidl_generator_c/string_functions.h"

#include <assert.h>
#include <stdlib.h>
#include <string.h>

void
rosidl_generator_c__String__init(rosidl_generator_c__String * str)
{
  if (!str) {
    return;
  }
  str->data = malloc(1);
  str->data[0] = '\0';
  str->size = 0;
  str->capacity = 1;
}

void
rosidl_generator_c__String__fini(rosidl_generator_c__String * str)
{
  if (!str) {
    return;
  }
  if (str->data) {
    /* ensure that data and capacity values are consistent */
    assert(str->capacity > 0);
    free(str->data);
    str->data = NULL;
    str->size = 0;
    str->capacity = 0;
  } else {
    /* ensure that data, size, and capacity values are consistent */
    assert(0 == str->size);
    assert(0 == str->capacity);
  }
}

bool
rosidl_generator_c__String__assignn(
  rosidl_generator_c__String * str, const char * value, size_t n)
{
  if (!str) {
    return false;
  }
  // a NULL value is not valid
  if (!value) {
    return false;
  }
  // since n + 1 bytes are being allocated n can't be the maximum value
  if (n == SIZE_MAX) {
    return false;
  }
  char * data = (char *)malloc(n + 1);
  if (!data) {
    return false;
  }
  rosidl_generator_c__String__fini(str);
  memcpy(data, value, n);
  data[n] = '\0';
  str->data = data;
  str->size = n;
  str->capacity = n + 1;
  return true;
}

bool
rosidl_generator_c__String__assign(
  rosidl_generator_c__String * str, const char * value)
{
  if (!str) {
    return false;
  }
  // a NULL value is not valid
  if (!value) {
    return false;
  }
  return rosidl_generator_c__String__assignn(
    str, value, strlen(value));
}

bool
rosidl_generator_c__String__Array__init(
  rosidl_generator_c__String__Array * array, size_t size)
{
  if (!array) {
    return false;
  }
  rosidl_generator_c__String * data = NULL;
  if (size) {
    data = (rosidl_generator_c__String *)malloc(sizeof(rosidl_generator_c__String) * size);
    if (!data) {
      return false;
    }
    // initialize all array elements
    for (size_t i = 0; i < size; ++i) {
      rosidl_generator_c__String__init(&data[i]);
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
rosidl_generator_c__String__Array__fini(
  rosidl_generator_c__String__Array * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      rosidl_generator_c__String__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

rosidl_generator_c__String__Array *
rosidl_generator_c__String__Array__create(size_t size)
{
  rosidl_generator_c__String__Array * array =
    (rosidl_generator_c__String__Array *)malloc(sizeof(rosidl_generator_c__String__Array));
  if (!array) {
    return NULL;
  }
  bool success = rosidl_generator_c__String__Array__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
rosidl_generator_c__String__Array__destroy(
  rosidl_generator_c__String__Array * array)
{
  if (array) {
    rosidl_generator_c__String__Array__fini(array);
  }
  free(array);
}
