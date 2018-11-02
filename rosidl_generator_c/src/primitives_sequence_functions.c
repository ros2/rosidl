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
#include <stdint.h>
#include <stdlib.h>

#include "rosidl_generator_c/primitives_sequence_functions.h"

#define ROSIDL_GENERATOR_C__DEFINE_PRIMITIVE_SEQUENCE_FUNCTIONS(STRUCT_NAME, TYPE_NAME) \
  bool rosidl_generator_c__ ## STRUCT_NAME ## __Sequence__init( \
    rosidl_generator_c__ ## STRUCT_NAME ## __Sequence * sequence, size_t size) \
  { \
    if (!sequence) { \
      return false; \
    } \
    TYPE_NAME * data = NULL; \
    if (size) { \
      data = malloc(sizeof(TYPE_NAME) * size); \
      if (!data) { \
        return false; \
      } \
    } \
    sequence->data = data; \
    sequence->size = size; \
    sequence->capacity = size; \
    return true; \
  } \
 \
  void rosidl_generator_c__ ## STRUCT_NAME ## __Sequence__fini( \
    rosidl_generator_c__ ## STRUCT_NAME ## __Sequence * sequence) \
  { \
    if (!sequence) { \
      return; \
    } \
    if (sequence->data) { \
      /* ensure that data and capacity values are consistent */ \
      assert(sequence->capacity > 0); \
      free(sequence->data); \
      sequence->data = NULL; \
      sequence->size = 0; \
      sequence->capacity = 0; \
    } else { \
      /* ensure that data, size, and capacity values are consistent */ \
      assert(0 == sequence->size); \
      assert(0 == sequence->capacity); \
    } \
  }

// sequence functions for all primitive types
ROSIDL_GENERATOR_C__DEFINE_PRIMITIVE_SEQUENCE_FUNCTIONS(bool, bool)
ROSIDL_GENERATOR_C__DEFINE_PRIMITIVE_SEQUENCE_FUNCTIONS(byte, uint8_t)
ROSIDL_GENERATOR_C__DEFINE_PRIMITIVE_SEQUENCE_FUNCTIONS(char, signed char)
ROSIDL_GENERATOR_C__DEFINE_PRIMITIVE_SEQUENCE_FUNCTIONS(float32, float)
ROSIDL_GENERATOR_C__DEFINE_PRIMITIVE_SEQUENCE_FUNCTIONS(float64, double)
ROSIDL_GENERATOR_C__DEFINE_PRIMITIVE_SEQUENCE_FUNCTIONS(int8, int8_t)
ROSIDL_GENERATOR_C__DEFINE_PRIMITIVE_SEQUENCE_FUNCTIONS(uint8, uint8_t)
ROSIDL_GENERATOR_C__DEFINE_PRIMITIVE_SEQUENCE_FUNCTIONS(int16, int16_t)
ROSIDL_GENERATOR_C__DEFINE_PRIMITIVE_SEQUENCE_FUNCTIONS(uint16, uint16_t)
ROSIDL_GENERATOR_C__DEFINE_PRIMITIVE_SEQUENCE_FUNCTIONS(int32, int32_t)
ROSIDL_GENERATOR_C__DEFINE_PRIMITIVE_SEQUENCE_FUNCTIONS(uint32, uint32_t)
ROSIDL_GENERATOR_C__DEFINE_PRIMITIVE_SEQUENCE_FUNCTIONS(int64, int64_t)
ROSIDL_GENERATOR_C__DEFINE_PRIMITIVE_SEQUENCE_FUNCTIONS(uint64, uint64_t)
