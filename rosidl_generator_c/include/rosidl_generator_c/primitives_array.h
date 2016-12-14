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

#ifndef ROSIDL_GENERATOR_C__PRIMITIVES_ARRAY_H_
#define ROSIDL_GENERATOR_C__PRIMITIVES_ARRAY_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define ROSIDL_GENERATOR_C__PRIMITIVE_ARRAY(STRUCT_NAME, TYPE_NAME) \
  typedef struct rosidl_generator_c__ ## STRUCT_NAME ## __Array \
  { \
    TYPE_NAME * data; \
    size_t size; /*!< The number of valid items in data */ \
    size_t capacity; /*!< The number of allocated items in data */ \
  } rosidl_generator_c__ ## STRUCT_NAME ## __Array;

// array types for all primitive types
ROSIDL_GENERATOR_C__PRIMITIVE_ARRAY(bool, bool)
ROSIDL_GENERATOR_C__PRIMITIVE_ARRAY(byte, uint8_t)
ROSIDL_GENERATOR_C__PRIMITIVE_ARRAY(char, char)
ROSIDL_GENERATOR_C__PRIMITIVE_ARRAY(float32, float)
ROSIDL_GENERATOR_C__PRIMITIVE_ARRAY(float64, double)
ROSIDL_GENERATOR_C__PRIMITIVE_ARRAY(int8, int8_t)
ROSIDL_GENERATOR_C__PRIMITIVE_ARRAY(uint8, uint8_t)
ROSIDL_GENERATOR_C__PRIMITIVE_ARRAY(int16, int16_t)
ROSIDL_GENERATOR_C__PRIMITIVE_ARRAY(uint16, uint16_t)
ROSIDL_GENERATOR_C__PRIMITIVE_ARRAY(int32, int32_t)
ROSIDL_GENERATOR_C__PRIMITIVE_ARRAY(uint32, uint32_t)
ROSIDL_GENERATOR_C__PRIMITIVE_ARRAY(int64, int64_t)
ROSIDL_GENERATOR_C__PRIMITIVE_ARRAY(uint64, uint64_t)

#endif  // ROSIDL_GENERATOR_C__PRIMITIVES_ARRAY_H_
