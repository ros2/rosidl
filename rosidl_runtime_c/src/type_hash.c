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

#include <string.h>

#include "rosidl_runtime_c/type_hash.h"

#include "rcutils/error_handling.h"

static const char RIHS01_PREFIX[] = "RIHS01_";
// Hash representation is hex string, two characters per byte
static const size_t RIHS_VERSION_IDX = 4;
static const size_t RIHS_PREFIX_LEN = 7;
static const size_t RIHS01_STRING_LEN = 71;  // RIHS_PREFIX_LEN + (ROSIDL_TYPE_HASH_SIZE * 2);

static bool _ishexdigit(char c)
{
  return (c >= '0' && c <= '9') || (c >= 'A' && c <= 'F') || (c >= 'a' && c <= 'f');
}

/// Translate a single character hex digit to a nibble
static uint8_t _xatoi(char c)
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

/// Tranlate a byte value to two hex characters
static void _xitoa(uint8_t val, char * dest)
{
  uint8_t nibble = 0;
  for (size_t n = 0; n < 2; n++) {
    nibble = (val >> (4 * n)) & 0x0f;
    if (nibble < 0xa) {
      dest[n] = '0' + nibble;
    } else {  // 0xa <= nibble < 0x10
      dest[n] = 'a' + (nibble - 0xa);
    }
  }
}

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
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(type_hash, RCUTILS_RET_INVALID_ARGUMENT);
  if (!rcutils_allocator_is_valid(&allocator)) {
    RCUTILS_SET_ERROR_MSG("Invalid allocator");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(output_string, RCUTILS_RET_INVALID_ARGUMENT);

  char * local_output = allocator.allocate(RIHS01_STRING_LEN + 1, allocator.state);
  if (!local_output) {
    *output_string = NULL;
    RCUTILS_SET_ERROR_MSG("Unable to allocate space for type hash string.");
    return RCUTILS_RET_BAD_ALLOC;
  }
  local_output[RIHS01_STRING_LEN] = '\0';
  memcpy(local_output, RIHS01_PREFIX, RIHS_PREFIX_LEN);
  for (size_t i = 0; i < ROSIDL_TYPE_HASH_SIZE; i++) {
    _xitoa(type_hash->value[i], local_output + RIHS_PREFIX_LEN + (i * 2));
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
  hash_out->version = 0;
  size_t input_len = strlen(type_hash_string);

  // Check prefix
  if (input_len < RIHS_PREFIX_LEN) {
    RCUTILS_SET_ERROR_MSG("Hash string not long enough to contain RIHS prefix.");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }
  if (0 != strncmp(type_hash_string, RIHS01_PREFIX, RIHS_VERSION_IDX)) {
    RCUTILS_SET_ERROR_MSG("Hash string doesn't start with RIHS.");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  // Parse version
  char version_top = type_hash_string[RIHS_VERSION_IDX];
  char version_bot = type_hash_string[RIHS_VERSION_IDX + 1];
  if (!(_ishexdigit(version_top) && _ishexdigit(version_bot))) {
    RCUTILS_SET_ERROR_MSG("RIHS version is not a 2-digit hex string.");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }
  hash_out->version = (_xatoi(version_top) << 4) + _xatoi(version_bot);

  if (hash_out->version != 1) {
    RCUTILS_SET_ERROR_MSG("Do not know how to parse RIHS version.");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }
  if (input_len != RIHS01_STRING_LEN) {
    RCUTILS_SET_ERROR_MSG("RIHS string is the incorrect size to contain a RIHS01 value.");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }
  const char * value_str = type_hash_string + RIHS_PREFIX_LEN;
  for (size_t i = 0; i < ROSIDL_TYPE_HASH_SIZE; i++) {
    if (!_ishexdigit(value_str[i * 2])) {
      RCUTILS_SET_ERROR_MSG("Type hash string value contained non-hex-digit character.");
      return RCUTILS_RET_INVALID_ARGUMENT;
    }
  }
  for (size_t i = 0; i < ROSIDL_TYPE_HASH_SIZE; i++) {
    // No error checking on byte values because the whole string was checked in prior loop
    uint8_t byte_val = (_xatoi(value_str[i * 2]) << 4) + _xatoi(value_str[i * 2 + 1]);
    if (byte_val < 0) {
      RCUTILS_SET_ERROR_MSG("Type hash string value did not contain only hex digits.");
      return RCUTILS_RET_INVALID_ARGUMENT;
    }
    hash_out->value[i] = (char)byte_val;
  }
  return RCUTILS_RET_OK;
}
