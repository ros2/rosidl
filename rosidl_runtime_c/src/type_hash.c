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

  // Hash representation is hex string, two characters per byte
  const char * fmt = "RIHS%02d_%64d";
  const size_t prefix_len = strlen("RIHS01_");
  char * local_output = rcutils_format_string(allocator, fmt, type_hash->version, 0);
  if (!local_output) {
    *output_string = NULL;
    RCUTILS_SET_ERROR_MSG("Unable to allocate space for stringified type hash.");
    return RCUTILS_RET_BAD_ALLOC;
  }
  for (size_t i = 0; i < ROSIDL_TYPE_HASH_SIZE; i++) {
    snprintf(local_output + prefix_len + (i * 2), 3, "%02x", type_hash->value[i]);  // NOLINT
  }

  *output_string = local_output;
  return RCUTILS_RET_OK;
}

static int _xatoi(char c)
{
  if (c >= '0' && c <= '9') {
    return c - '0';
  }
  if (c >= 'A' && c <= 'F') {
    return c - 'A' + 0xa;
  }
  if (c >= 'a' && c <= 'f') {
    return c - 'a' + 0xa;
  }
  return -1;
}

static int _str_to_byte(const char * str)
{
  return (_xatoi(str[0]) << 4) + _xatoi(str[1]);
}

rcutils_ret_t
rosidl_parse_type_hash_string(
  const char * type_hash_string,
  rosidl_type_hash_t * hash_out)
{
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(type_hash_string, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(hash_out, RCUTILS_RET_INVALID_ARGUMENT);
  static const size_t kprefix_len = sizeof("RIHS01_") - 1;
  static const size_t kvalue_len = 64;
  hash_out->version = 0;

  if (strlen(type_hash_string) != (kprefix_len + kvalue_len)) {
    RCUTILS_SET_ERROR_MSG("Hash string incorrect size.");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }
  if (0 != strncmp(type_hash_string, "RIHS01_", 7)) {
    RCUTILS_SET_ERROR_MSG("Type hash string is not prefixed RIHS01_");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }
  hash_out->version = 1;
  const char * value_str = type_hash_string + kprefix_len;
  for (size_t i = 0; i < 32; i++) {
    int byte_val = _str_to_byte(value_str + (i * 2));
    if (byte_val < 0) {
      RCUTILS_SET_ERROR_MSG("Type hash string value did not contain only hex digits.");
      return RCUTILS_RET_INVALID_ARGUMENT;
    }
    hash_out->value[i] = (char)byte_val;
  }
  return RCUTILS_RET_OK;
}
