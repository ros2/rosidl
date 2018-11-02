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
#include <stdio.h>

bool
rosidl_generator_c__String__init(rosidl_generator_c__String * str)
{
  if (!str) {
    return false;
  }
  str->data = malloc(1);
  if (!str->data) {
    return false;
  }
  str->data[0] = '\0';
  str->size = 0;
  str->capacity = 1;
  return true;
}

void
rosidl_generator_c__String__fini(rosidl_generator_c__String * str)
{
  if (!str) {
    return;
  }
  if (str->data) {
    /* ensure that data and capacity values are consistent */
    if (str->capacity <= 0) {
      fprintf(stderr, "Unexpected condition: string capacity was zero for allocated data! "
        "Exiting.\n");
      exit(-1);
    }
    free(str->data);
    str->data = NULL;
    str->size = 0;
    str->capacity = 0;
  } else {
    /* ensure that data, size, and capacity values are consistent */
    if (0 != str->size) {
      fprintf(stderr, "Unexpected condition: string size was non-zero for deallocated data! "
        "Exiting.\n");
      exit(-1);
    }
    if (0 != str->capacity) {
      fprintf(stderr, "Unexpected behavior: string capacity was non-zero for deallocated data! "
        "Exiting.\n");
      exit(-1);
    }
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
  char * data = realloc(str->data, n + 1);
  if (!data) {
    return false;
  }
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
  return rosidl_generator_c__String__assignn(
    str, value, strlen(value));
}

bool
rosidl_generator_c__String__Sequence__init(
  rosidl_generator_c__String__Sequence * sequence, size_t size)
{
  if (!sequence) {
    return false;
  }
  rosidl_generator_c__String * data = NULL;
  if (size) {
    data = (rosidl_generator_c__String *)calloc(size, sizeof(rosidl_generator_c__String));
    if (!data) {
      return false;
    }
    // initialize all sequence elements
    for (size_t i = 0; i < size; ++i) {
      if (!rosidl_generator_c__String__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > 0; ) {
          rosidl_generator_c__String__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
  }
  sequence->data = data;
  sequence->size = size;
  sequence->capacity = size;
  return true;
}

void
rosidl_generator_c__String__Sequence__fini(
  rosidl_generator_c__String__Sequence * sequence)
{
  if (!sequence) {
    return;
  }
  if (sequence->data) {
    // ensure that data and capacity values are consistent
    assert(sequence->capacity > 0);
    // finalize all sequence elements
    for (size_t i = 0; i < sequence->capacity; ++i) {
      rosidl_generator_c__String__fini(&sequence->data[i]);
    }
    free(sequence->data);
    sequence->data = NULL;
    sequence->size = 0;
    sequence->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == sequence->size);
    assert(0 == sequence->capacity);
  }
}

rosidl_generator_c__String__Sequence *
rosidl_generator_c__String__Sequence__create(size_t size)
{
  rosidl_generator_c__String__Sequence * sequence =
    (rosidl_generator_c__String__Sequence *)malloc(sizeof(rosidl_generator_c__String__Sequence));
  if (!sequence) {
    return NULL;
  }
  bool success = rosidl_generator_c__String__Sequence__init(sequence, size);
  if (!success) {
    free(sequence);
    return NULL;
  }
  return sequence;
}

void
rosidl_generator_c__String__Sequence__destroy(
  rosidl_generator_c__String__Sequence * sequence)
{
  if (sequence) {
    rosidl_generator_c__String__Sequence__fini(sequence);
  }
  free(sequence);
}
