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

#include <math.h>

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
  const char * fmt = "RIHS%d_%64d";
  const size_t prefix_len = strlen("RIHS1_");
  char * local_output = rcutils_format_string(allocator, fmt, type_hash->version, 0);
  if (!local_output) {
    *output_string = NULL;
    RCUTILS_SET_ERROR_MSG("Unable to allocate space for stringified type hash.");
    return RCUTILS_RET_BAD_ALLOC;
  }
  for (size_t i = 0; i < ROSIDL_TYPE_HASH_SIZE; i++) {
    sprintf(local_output + prefix_len + (i * 2), "%02x", type_hash->value[i]);
  }

  *output_string = local_output;
  return RCUTILS_RET_OK;
}

rcutils_ret_t
rosidl_parse_type_hash_string(
  const char * type_hash_string,
  rosidl_type_hash_t * hash_out)
{
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(type_hash_string, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(hash_out, RCUTILS_RET_INVALID_ARGUMENT);
  static const size_t value_length = 64;  // 32 bytes * 2 digit characters
  char hash_value_str[value_length + 1];
  hash_value_str[value_length] = '\0';
  int res = sscanf(type_hash_string, "RIHS%hhu_%64s", &hash_out->version, hash_value_str);
  if (res != 2) {
    RCUTILS_SET_ERROR_MSG("Type hash data did not match expected format.");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }
  size_t version_digits = log10(hash_out->version) + 1;
  size_t prefix_fixed_len = strlen("RIHS_");
  if (strlen(type_hash_string) > value_length + prefix_fixed_len + version_digits) {
    RCUTILS_SET_ERROR_MSG("Hash value too long.");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }
  for (size_t i = 0; i < ROSIDL_TYPE_HASH_SIZE; i += 1) {
    if (sscanf(hash_value_str + (i * 2), "%2hhx", &hash_out->value[i]) != 1) {
      RCUTILS_SET_ERROR_MSG("Couldn't parse hex string of type hash value.");
      return RCUTILS_RET_INVALID_ARGUMENT;
    }
  }
  return RCUTILS_RET_OK;
}
