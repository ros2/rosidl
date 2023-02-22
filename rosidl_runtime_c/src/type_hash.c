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

#include "rosidl_runtime_c/type_hash.h"

#include "rcutils/error_handling.h"
#include "rcutils/format_string.h"

rosidl_type_hash_t
rosidl_get_zero_initialized_type_hash(void)
{
  rosidl_type_hash_t zero_initialized_type_hash = {0};
  return zero_initialized_type_hash;
}

rcutils_ret_t
rosidl_stringify_type_hash(
  const rosidl_type_hash_t * type_hash,
  rcutils_allocator_t allocator,
  char ** output_string)
{
  if (!type_hash) {
    RCUTILS_SET_ERROR_MSG("Null type_hash argument.");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }
  if (!rcutils_allocator_is_valid(&allocator)) {
    RCUTILS_SET_ERROR_MSG("Invalid allocator");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }
  if (!output_string) {
    RCUTILS_SET_ERROR_MSG("Null string destination pointer.");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  // Hash representation will be simple hex string, two characters per byte
  const char * fmt = "RIHS%d_%064d";
  char * local_output = rcutils_format_string(allocator, fmt, type_hash->version, 0);
  if (!local_output) {
    *output_string = NULL;
    RCUTILS_SET_ERROR_MSG("Unable to allocate space for stringified type hash.");
    return RCUTILS_RET_BAD_ALLOC;
  }
  for (size_t i = 0; i < RCUTILS_SHA256_BLOCK_SIZE; i++) {
    sprintf(local_output + (i * 2), "%02x", type_hash->value[i]);
  }
  fprintf(stderr, "%s", local_output);

  *output_string = local_output;
  return RCUTILS_RET_OK;
}
