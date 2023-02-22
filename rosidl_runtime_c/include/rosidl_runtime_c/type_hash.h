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

#ifndef ROSIDL_RUNTIME_C__TYPE_HASH_H_
#define ROSIDL_RUNTIME_C__TYPE_HASH_H_

#include <stdint.h>

#include "rcutils/sha256.h"

#include "rosidl_runtime_c/visibility_control.h"

#define ROSIDL_TYPE_HASH_VERSION_UNSET 0
#define ROSIDL_TYPE_HASH_SIZE RCUTILS_SHA256_BLOCK_SIZE

typedef struct rosidl_type_hash_s
{
  uint8_t version;
  uint8_t value[ROSIDL_TYPE_HASH_SIZE];
} rosidl_type_hash_t;

ROSIDL_GENERATOR_C_PUBLIC
rosidl_type_hash_t get_zero_initialized_type_hash(void);

#endif  // ROSIDL_RUNTIME_C__TYPE_HASH_H_
