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
static const uint8_t INVALID_NIBBLE = 0xff;

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
  return INVALID_NIBBLE;
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

  uint8_t nibble = 0;
  char * dest = NULL;
  for (size_t i = 0; i < ROSIDL_TYPE_HASH_SIZE; i++) {
    // Translate byte into two hex characters
    dest = local_output + RIHS_PREFIX_LEN + (i * 2);
    // First character is top half of byte
    nibble = (type_hash->value[i] >> 4) & 0x0f;
    if (nibble < 0xa) {
      dest[0] = '0' + nibble;
    } else {
      dest[0] = 'a' + (nibble - 0xa);
    }
    // Second character is bottom half of byte
    nibble = (type_hash->value[i] >> 0) & 0x0f;
    if (nibble < 0xa) {
      dest[1] = '0' + nibble;
    } else {
      dest[1] = 'a' + (nibble - 0xa);
    }
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
  uint8_t hexbyte_top_nibble;
  uint8_t hexbyte_bot_nibble;

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
  hexbyte_top_nibble = _xatoi(type_hash_string[RIHS_VERSION_IDX]);
  hexbyte_bot_nibble = _xatoi(type_hash_string[RIHS_VERSION_IDX + 1]);
  if (hexbyte_top_nibble == INVALID_NIBBLE || hexbyte_bot_nibble == INVALID_NIBBLE) {
    RCUTILS_SET_ERROR_MSG("RIHS version is not a 2-digit hex string.");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }
  hash_out->version = (hexbyte_top_nibble << 4) + hexbyte_bot_nibble;

  if (hash_out->version != 1) {
    RCUTILS_SET_ERROR_MSG("Do not know how to parse RIHS version.");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  // Check total length
  if (input_len != RIHS01_STRING_LEN) {
    RCUTILS_SET_ERROR_MSG("RIHS string is the incorrect size to contain a RIHS01 value.");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  // Parse hash value
  const char * value_str = type_hash_string + RIHS_PREFIX_LEN;
  for (size_t i = 0; i < ROSIDL_TYPE_HASH_SIZE; i++) {
    hexbyte_top_nibble = _xatoi(value_str[i * 2]);
    hexbyte_bot_nibble = _xatoi(value_str[i * 2 + 1]);
    if (hexbyte_top_nibble == INVALID_NIBBLE || hexbyte_bot_nibble == INVALID_NIBBLE) {
      RCUTILS_SET_ERROR_MSG("Type hash string value contained non-hexdigit character.");
      return RCUTILS_RET_INVALID_ARGUMENT;
    }
    hash_out->value[i] = (hexbyte_top_nibble << 4) + hexbyte_bot_nibble;
  }
  return RCUTILS_RET_OK;
}
