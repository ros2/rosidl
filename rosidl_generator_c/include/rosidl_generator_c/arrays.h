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

#ifndef ROSIDL_GENERATOR_C__ARRAYS_H_
#define ROSIDL_GENERATOR_C__ARRAYS_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#ifdef __cplusplus
#define ROSIDL_ALLOC(Type) static_cast<Type *>(malloc(sizeof(Type)))
#define ROSIDL_ALLOC_ARRAY(Type, Size) \
  static_cast<Type *>(malloc(sizeof(Type) * Size))
#else
#define ROSIDL_ALLOC(Type) malloc(sizeof(Type))
#define ROSIDL_ALLOC_ARRAY(Type, Size) malloc(sizeof(Type) * Size)
#endif

#define ROSIDL_GENERATE_STATIC_ARRAY_CREATE(ContainingMsg, Field, Size) \
  ROSIDL_Array__ ## ContainingMsg ## __ ## Field * \
  ROSIDL_Array__ ## ContainingMsg ## __ ## Field ## __create() \
  { \
    /* Initialize const parts on the stack */ \
    ROSIDL_Array__ ## ContainingMsg ## __ ## Field init = { \
      .size = Size, .capacity = Size, .is_fixed = true}; \
    /* Allocate space for the returned pointer */ \
    ROSIDL_Array__ ## ContainingMsg ## __ ## Field * pointer = \
      ROSIDL_ALLOC(ROSIDL_Array__ ## ContainingMsg ## __ ## Field); \
    /* Copy from the stack to the heap */ \
    memcpy(pointer, &init, sizeof(ROSIDL_Array__ ## ContainingMsg ## __ ## Field)); \
    return pointer; \
  }

#define ROSIDL_GENERATE_STATIC_ARRAY_DESTROY(Name, Size) \
  void ROSIDL_Array__ ## Name ## __destroy(ROSIDL_Array__ ## Name * array) \
  { \
    free(array); \
  }

/* Used to generate struct and functions/macros for fixed size arrays.
 *
 * Fixed sized arrays are generated on demand for specific fields of messages.
 */

// TODO(wjwwood): Error check malloc.
#define ROSIDL_GENERATE_STATIC_ARRAY(ContainingMsg, Field, Type, Size) \
  typedef Type ContainingMsg ## __ ## Field ## _t[Size]; \
  typedef struct ROSIDL_Array__ ## ContainingMsg ## __ ## Field \
  { \
    ContainingMsg ## __ ## Field ## _t data; \
    const size_t size; \
    const size_t capacity; \
    const bool is_fixed; \
  } ROSIDL_Array__ ## ContainingMsg ## __ ## Field; \
  ROSIDL_GENERATE_STATIC_ARRAY_CREATE(ContainingMsg, Field, Size) \
  ROSIDL_GENERATE_STATIC_ARRAY_DESTROY(ContainingMsg, Field, Size)

/* Used to create a struct and create and destroy functions for arrays.
 *
 * This macro is used below to generate dynamic arrays for primitive types.
 * It is also always used in the generated headers for messages, such that
 * any message type also always have struct and functions for dynamic arrays
 * of that type too.
 */
#define ROSIDL_GENERATE_ARRAY(Name) ROSIDL_GENERATE_PRIMITIVE_ARRAY(Name, Name)

// TODO(wjwwood): Do malloc error checking, consider how to do error checking
//                and allocation without reproducing stuff in rmw.
#define ROSIDL_GENERATE_PRIMITIVE_ARRAY(Name, Type) \
  /* Structure for containing dynamic length array of given type */ \
  typedef struct ROSIDL_Array__ ## Name \
  { \
    Type * data; \
    size_t size; \
    size_t capacity; \
    const bool is_fixed; \
  } ROSIDL_Array__ ## Name; \
  /* Construction function for dynamic array structure */ \
  ROSIDL_Array__ ## Name * ROSIDL_Array__ ## Name ## __create(size_t size) \
  { \
    ROSIDL_Array__ ## Name * p = ROSIDL_ALLOC_ARRAY(ROSIDL_Array__ ## Name, size); \
    return p; \
  } \
  /* Destructor function for dynamic array structure */ \
  void ROSIDL_Array__ ## Name ## __destroy(ROSIDL_Array__ ## Name * array) \
  { \
    free(array); \
  }

/* Generate Dynamic Length Arrays for Primitives */

ROSIDL_GENERATE_PRIMITIVE_ARRAY(bool, bool);
ROSIDL_GENERATE_PRIMITIVE_ARRAY(byte, uint8_t);
ROSIDL_GENERATE_PRIMITIVE_ARRAY(char, char);
ROSIDL_GENERATE_PRIMITIVE_ARRAY(float32, float);
ROSIDL_GENERATE_PRIMITIVE_ARRAY(float64, double);
ROSIDL_GENERATE_PRIMITIVE_ARRAY(uint8, uint8_t);
ROSIDL_GENERATE_PRIMITIVE_ARRAY(int8, int8_t);
ROSIDL_GENERATE_PRIMITIVE_ARRAY(uint16, uint16_t);
ROSIDL_GENERATE_PRIMITIVE_ARRAY(int16, int16_t);
ROSIDL_GENERATE_PRIMITIVE_ARRAY(uint32, uint32_t);
ROSIDL_GENERATE_PRIMITIVE_ARRAY(int32, int32_t);
ROSIDL_GENERATE_PRIMITIVE_ARRAY(uint64, uint64_t);
ROSIDL_GENERATE_PRIMITIVE_ARRAY(int64, int64_t);
// NOLINTNEXTLINE(build/include_what_you_use)
ROSIDL_GENERATE_PRIMITIVE_ARRAY(string, char *);

#endif  // ROSIDL_GENERATOR_C__ARRAYS_H_
