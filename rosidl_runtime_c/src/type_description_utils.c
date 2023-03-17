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

// NOTE(methylDragon): Maybe this belongs inside the type_description_interfaces
//                     package instead?
//                     But as a C header (.h)???

// NOTE(methylDragon): I made it so that every instance of a non-message struct (e.g. hash map)
//                     borrows, whereas the message structs copy.
//                     So lifetime should be managed by the message structs.

#include "rosidl_runtime_c/type_description_utils.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <rcutils/logging_macros.h>
#include <rcutils/qsort.h>
#include <rcutils/types/rcutils_ret.h>
#include <rcutils/types/hash_map.h>
#include <rosidl_runtime_c/string_functions.h>

#include "rosidl_runtime_c/type_description/field__functions.h"
#include "rosidl_runtime_c/type_description/field__struct.h"
#include "rosidl_runtime_c/type_description/individual_type_description__functions.h"
#include "rosidl_runtime_c/type_description/individual_type_description__struct.h"
#include "rosidl_runtime_c/type_description/type_description__functions.h"
#include "rosidl_runtime_c/type_description/type_description__struct.h"


// Modified from https://stackoverflow.com/questions/4398711/round-to-the-nearest-power-of-two
int next_power_of_two(int v)
{
  v--;
  v |= v >> 1;
  v |= v >> 2;
  v |= v >> 4;
  v |= v >> 8;
  v |= v >> 16;
  v++;   // next power of 2

  return v > 1 ? v : 1;
}

// =================================================================================================
// GET BY NAME
// =================================================================================================

rcutils_ret_t
rosidl_runtime_c_type_description_utils_find_field(
  const rosidl_runtime_c__type_description__Field__Sequence * fields,
  const char * name,
  rosidl_runtime_c__type_description__Field ** field)
{
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(fields, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(name, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(field, RCUTILS_RET_INVALID_ARGUMENT);
  if (*field != NULL) {
    RCUTILS_LOG_ERROR("`field` output argument is not pointing to null");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  for (size_t i = 0; i < fields->size; i++) {
    if (strcmp(fields->data[i].name.data, name) == 0) {
      *field = &fields->data[i];
      return RCUTILS_RET_OK;
    }
  }

  RCUTILS_LOG_WARN("Could not find field: %s", name);
  return RCUTILS_RET_NOT_FOUND;
}


rcutils_ret_t
rosidl_runtime_c_type_description_utils_find_referenced_type_description(
  const rosidl_runtime_c__type_description__IndividualTypeDescription__Sequence * referenced_types,
  const char * type_name,
  rosidl_runtime_c__type_description__IndividualTypeDescription ** referenced_type)
{
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(referenced_types, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(type_name, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(referenced_type, RCUTILS_RET_INVALID_ARGUMENT);
  if (*referenced_type != NULL) {
    RCUTILS_LOG_ERROR("`referenced_type` output argument is not pointing to null");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  for (size_t i = 0; i < referenced_types->size; i++) {
    if (strcmp(referenced_types->data[i].type_name.data, type_name) == 0) {
      *referenced_type = &referenced_types->data[i];
      return RCUTILS_RET_OK;
    }
  }

  RCUTILS_LOG_WARN("Could not find referenced type description: %s", type_name);
  return RCUTILS_RET_NOT_FOUND;
}


// =================================================================================================
// HASH MAPS
// =================================================================================================

rcutils_ret_t
rosidl_runtime_c_type_description_utils_get_field_map(
  const rosidl_runtime_c__type_description__IndividualTypeDescription * individual_description,
  const rcutils_allocator_t * allocator,
  rcutils_hash_map_t ** hash_map)
{
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(individual_description, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(allocator, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(hash_map, RCUTILS_RET_INVALID_ARGUMENT);
  if (*hash_map != NULL) {
    RCUTILS_LOG_ERROR("`hash_map` output argument is not pointing to null");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  rcutils_hash_map_t * out = allocator->allocate(sizeof(rcutils_hash_map_t), allocator->state);
  if (out == NULL) {
    RCUTILS_LOG_ERROR("Could not allocate output hash map");
    return RCUTILS_RET_BAD_ALLOC;
  }
  *out = rcutils_get_zero_initialized_hash_map();

  rcutils_ret_t ret = RCUTILS_RET_ERROR;
  rcutils_ret_t fail_ret = RCUTILS_RET_ERROR;

  ret = rcutils_hash_map_init(
    out, next_power_of_two(individual_description->fields.size),
    sizeof(char *), sizeof(rosidl_runtime_c__type_description__Field *),
    rcutils_hash_map_string_hash_func, rcutils_hash_map_string_cmp_func,
    allocator);
  if (ret != RCUTILS_RET_OK) {
    allocator->deallocate(out, allocator->state);
    RCUTILS_LOG_ERROR("Could not init hash map");
    return ret;
  }

  for (size_t i = 0; i < individual_description->fields.size; i++) {
    rosidl_runtime_c__type_description__Field * tmp = &individual_description->fields.data[i];
    // Passing tmp is fine even if tmp goes out of scope later since it copies in the set method...
    ret = rcutils_hash_map_set(out, &individual_description->fields.data[i].name.data, &tmp);
    if (ret != RCUTILS_RET_OK) {
      RCUTILS_LOG_ERROR(
        "Could not set hash map entry for field: %s",
        individual_description->fields.data[i].name.data);
      fail_ret = ret;
      goto fail;
    }
  }

  *hash_map = out;
  return RCUTILS_RET_OK;

fail:
  {
    rcutils_ret_t fini_ret = rcutils_hash_map_fini(out);
    if (fini_ret != RCUTILS_RET_OK) {
      RCUTILS_LOG_ERROR("Failed to finalize hash map");
    }
    allocator->deallocate(out, allocator->state);
  }
  return fail_ret;
}


rcutils_ret_t
rosidl_runtime_c_type_description_utils_get_referenced_type_description_map(
  const rosidl_runtime_c__type_description__IndividualTypeDescription__Sequence * referenced_types,
  const rcutils_allocator_t * allocator,
  rcutils_hash_map_t ** hash_map)
{
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(referenced_types, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(allocator, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(hash_map, RCUTILS_RET_INVALID_ARGUMENT);
  if (*hash_map != NULL) {
    RCUTILS_LOG_ERROR("`hash_map` output argument is not pointing to null");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  rcutils_hash_map_t * out = allocator->allocate(sizeof(rcutils_hash_map_t), allocator->state);
  if (out == NULL) {
    RCUTILS_LOG_ERROR("Could not allocate output hash map");
    return RCUTILS_RET_BAD_ALLOC;
  }
  *out = rcutils_get_zero_initialized_hash_map();

  rcutils_ret_t ret = RCUTILS_RET_ERROR;
  rcutils_ret_t fail_ret = RCUTILS_RET_ERROR;

  ret = rcutils_hash_map_init(
    out, next_power_of_two(referenced_types->size),
    sizeof(char *), sizeof(rosidl_runtime_c__type_description__IndividualTypeDescription *),
    rcutils_hash_map_string_hash_func, rcutils_hash_map_string_cmp_func,
    allocator);
  if (ret != RCUTILS_RET_OK) {
    allocator->deallocate(out, allocator->state);
    RCUTILS_LOG_ERROR("Could not init hash map");
    return ret;
  }

  for (size_t i = 0; i < referenced_types->size; i++) {
    rosidl_runtime_c__type_description__IndividualTypeDescription * tmp =
      &referenced_types->data[i];

    // Check for duplicate referenced types
    if (rcutils_hash_map_key_exists(out, &referenced_types->data[i].type_name.data)) {
      rosidl_runtime_c__type_description__IndividualTypeDescription * stored_description;
      ret = rcutils_hash_map_get(
        out, &referenced_types->data[i].type_name.data, &stored_description);
      if (ret != RCUTILS_RET_OK) {
        RCUTILS_LOG_ERROR(
          "Could not get stored description: %s", referenced_types->data[i].type_name.data);
        fail_ret = ret;  // Most likely a RCUTILS_RET_NOT_FOUND
        goto fail;
      }

      if (!rosidl_runtime_c__type_description__IndividualTypeDescription__are_equal(
          &referenced_types->data[i], stored_description))
      {
        // Non-identical duplicate referenced types is invalid (it's ambiguous which one to use)
        RCUTILS_LOG_ERROR(
          "Passed referenced IndividualTypeDescriptions has non-identical duplicate types");
        fail_ret = RCUTILS_RET_INVALID_ARGUMENT;
        goto fail;
      }
    }

    // Passing tmp is fine even if tmp goes out of scope later since it copies in the set method...
    ret = rcutils_hash_map_set(out, &referenced_types->data[i].type_name.data, &tmp);
    if (ret != RCUTILS_RET_OK) {
      RCUTILS_LOG_ERROR(
        "Could not set hash map entry for referenced type: %s",
        referenced_types->data[i].type_name.data);
      fail_ret = ret;
      goto fail;
    }
  }

  size_t map_length;
  ret = rcutils_hash_map_get_size(out, &map_length);
  if (ret != RCUTILS_RET_OK) {
    RCUTILS_LOG_ERROR("Could not get size of hash map for validation");
    fail_ret = RCUTILS_RET_ERROR;
    goto fail;
  }

  *hash_map = out;
  return RCUTILS_RET_OK;

fail:
  {
    rcutils_ret_t fini_ret = rcutils_hash_map_fini(out);
    if (fini_ret != RCUTILS_RET_OK) {
      RCUTILS_LOG_ERROR("Failed to finalize hash map");
    }
    allocator->deallocate(out, allocator->state);
  }
  return fail_ret;
}


// =================================================================================================
// DESCRIPTION VALIDITY
// =================================================================================================

rcutils_ret_t
rosidl_runtime_c_type_description_utils_get_necessary_referenced_type_descriptions_map(
  const rosidl_runtime_c__type_description__IndividualTypeDescription * main_type_description,
  const rcutils_hash_map_t * referenced_types_map,
  const rcutils_allocator_t * allocator,
  rcutils_hash_map_t ** seen_map)
{
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(main_type_description, RCUTILS_RET_INVALID_ARGUMENT);
  HASH_MAP_VALIDATE_HASH_MAP(referenced_types_map);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(allocator, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(seen_map, RCUTILS_RET_INVALID_ARGUMENT);

  // Only true for the top level call, so we can determine when to finalize the map
  bool top_level_call = false;
  rcutils_ret_t ret = RCUTILS_RET_ERROR;
  rcutils_ret_t fail_ret = RCUTILS_RET_ERROR;

  // 1. Init new hash map only on the top level call
  if (!*seen_map) {
    top_level_call = true;

    *seen_map = allocator->allocate(sizeof(rcutils_hash_map_t), allocator->state);
    if (*seen_map == NULL) {
      RCUTILS_LOG_ERROR("Could not allocate hash map");
      return RCUTILS_RET_BAD_ALLOC;
    }
    **seen_map = rcutils_get_zero_initialized_hash_map();

    size_t referenced_types_map_size;
    ret = rcutils_hash_map_get_size(referenced_types_map, &referenced_types_map_size);
    if (ret != RCUTILS_RET_OK) {
      allocator->deallocate(*seen_map, allocator->state);
      RCUTILS_LOG_ERROR("Could not get size of referenced types hash map");
      *seen_map = NULL;
      return RCUTILS_RET_ERROR;
    }

    ret = rcutils_hash_map_init(
      *seen_map, next_power_of_two(referenced_types_map_size),
      sizeof(char *), sizeof(rosidl_runtime_c__type_description__IndividualTypeDescription *),
      rcutils_hash_map_string_hash_func, rcutils_hash_map_string_cmp_func,
      allocator);
    if (ret != RCUTILS_RET_OK) {
      allocator->deallocate(*seen_map, allocator->state);
      RCUTILS_LOG_ERROR("Could not init hash map");
      *seen_map = NULL;
      return RCUTILS_RET_BAD_ALLOC;
    }
  }

  // 2. Iterate through fields
  for (size_t i = 0; i < main_type_description->fields.size; i++) {
    rosidl_runtime_c__type_description__Field * field = &main_type_description->fields.data[i];
    // 3. Skip cases
    // continue if field is not nested type or nested type is in seen map:
    if ((field->type.type_id %
      ROSIDL_RUNTIME_C_TYPE_DESCRIPTION_UTILS_SEQUENCE_TYPE_ID_DELIMITER) != 1 ||
      rcutils_hash_map_key_exists(*seen_map, &field->type.nested_type_name.data))
    {
      continue;
    }

    // 4. Error cases
    // Referenced type does not exist
    if (!rcutils_hash_map_key_exists(referenced_types_map, &field->type.nested_type_name.data)) {
      RCUTILS_LOG_ERROR("Missing referenced type: %s", field->type.nested_type_name.data);
      fail_ret = RCUTILS_RET_NOT_FOUND;
      goto fail;
    }
    // Nested name empty
    if (field->type.nested_type_name.size == 0) {
      RCUTILS_LOG_ERROR("Missing referenced type name");
      fail_ret = RCUTILS_RET_INVALID_ARGUMENT;
      goto fail;
    }

    // 5. Add to seen map (we didn't skip and didn't error out)
    rosidl_runtime_c__type_description__IndividualTypeDescription * necessary_description;

    ret = rcutils_hash_map_get(
      referenced_types_map, &field->type.nested_type_name.data, &necessary_description);
    if (ret != RCUTILS_RET_OK) {
      RCUTILS_LOG_ERROR(
        "Could not get necessary referenced type: %s", field->type.nested_type_name.data);
      fail_ret = ret;  // Most likely a RCUTILS_RET_NOT_FOUND
      goto fail;
    }

    // Check for mismatched name
    if (strcmp(field->type.nested_type_name.data, necessary_description->type_name.data) != 0) {
      RCUTILS_LOG_ERROR(
        "Necessary referenced type name (%s) did not match field's nested type name (%s)",
        necessary_description->type_name.data,
        field->type.nested_type_name.data);
      fail_ret = RCUTILS_RET_INVALID_ARGUMENT;
      goto fail;
    }

    // Add to map (finally!!)
    ret = rcutils_hash_map_set(
      *seen_map, &field->type.nested_type_name.data, &necessary_description);
    if (ret != RCUTILS_RET_OK) {
      RCUTILS_LOG_ERROR("Failed to set hash map for key: %s", field->type.nested_type_name.data);
      fail_ret = ret;
      goto fail;
    }

    // Recurse on fields on necessary_description
    ret = rosidl_runtime_c_type_description_utils_get_necessary_referenced_type_descriptions_map(
      necessary_description, referenced_types_map, allocator, seen_map);
    if (ret != RCUTILS_RET_OK) {
      RCUTILS_LOG_ERROR("Recursion failed on: %s", necessary_description->type_name.data);
      fail_ret = ret;
      goto fail;
    }
  }  // End field iteration

  return RCUTILS_RET_OK;

fail:
  if (top_level_call) {
    rcutils_ret_t fini_ret = rcutils_hash_map_fini(*seen_map);
    if (fini_ret != RCUTILS_RET_OK) {
      RCUTILS_LOG_ERROR("Failed to finalize hash map");
    }
    allocator->deallocate(*seen_map, allocator->state);
    *seen_map = NULL;
  }
  return fail_ret;
}


rcutils_ret_t
rosidl_runtime_c_type_description_utils_copy_init_sequence_from_referenced_type_descriptions_map(
  const rcutils_hash_map_t * hash_map,
  rosidl_runtime_c__type_description__IndividualTypeDescription__Sequence ** sequence,
  bool sort)
{
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(hash_map, RCUTILS_RET_INVALID_ARGUMENT);
  if (*sequence != NULL) {
    RCUTILS_LOG_ERROR("`sequence` output argument is not pointing to null");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  size_t map_length;
  rcutils_ret_t ret = RCUTILS_RET_ERROR;
  ret = rcutils_hash_map_get_size(hash_map, &map_length);
  if (ret != RCUTILS_RET_OK) {
    RCUTILS_LOG_ERROR("Could not get size of hash map");
    return RCUTILS_RET_ERROR;
  }
  *sequence = rosidl_runtime_c__type_description__IndividualTypeDescription__Sequence__create(
    map_length);
  if (*sequence == NULL) {
    RCUTILS_LOG_ERROR("Could allocate sequence");
    return RCUTILS_RET_BAD_ALLOC;
  }

  size_t i = 0;
  char * key;
  rosidl_runtime_c__type_description__IndividualTypeDescription * data;
  rcutils_ret_t status = rcutils_hash_map_get_next_key_and_data(hash_map, NULL, &key, &data);
  while (RCUTILS_RET_OK == status) {
    if (strcmp(key, data->type_name.data) != 0) {
      RCUTILS_LOG_ERROR(
        "Necessary referenced type name (%s) did not match key (%s)", data->type_name.data, key);
      rosidl_runtime_c__type_description__IndividualTypeDescription__Sequence__destroy(*sequence);
      return RCUTILS_RET_INVALID_ARGUMENT;
    }

    // Deep copy
    if (!rosidl_runtime_c__type_description__IndividualTypeDescription__copy(
        data, &((*sequence)->data[i])))
    {
      RCUTILS_LOG_ERROR("Could not copy type %s to sequence", key);
      rosidl_runtime_c__type_description__IndividualTypeDescription__Sequence__destroy(*sequence);
      return RCUTILS_RET_BAD_ALLOC;
    }

    i += 1;
    status = rcutils_hash_map_get_next_key_and_data(hash_map, &key, &key, &data);
  }

  if (sort) {
    rcutils_ret_t ret =
      rosidl_runtime_c_type_description_utils_sort_referenced_type_descriptions_in_place(*sequence);
    if (ret != RCUTILS_RET_OK) {
      RCUTILS_LOG_WARN("Could not sort copy of referenced type descriptions for validation");
    }
  }

  return RCUTILS_RET_OK;
}


int
rosidl_runtime_c_type_description_utils_referenced_type_description_sequence_sort_compare(
  const void * lhs, const void * rhs)
{
  rosidl_runtime_c__type_description__IndividualTypeDescription * left =
    (rosidl_runtime_c__type_description__IndividualTypeDescription *)lhs;
  rosidl_runtime_c__type_description__IndividualTypeDescription * right =
    (rosidl_runtime_c__type_description__IndividualTypeDescription *)rhs;
  if (left == NULL) {
    return right == NULL ? 0 : 1;
  } else if (right == NULL) {
    return -1;
  }
  return strcmp(left->type_name.data, right->type_name.data);
}


rcutils_ret_t
rosidl_runtime_c_type_description_utils_sort_referenced_type_descriptions_in_place(
  rosidl_runtime_c__type_description__IndividualTypeDescription__Sequence * sequence)
{
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(sequence, RCUTILS_RET_INVALID_ARGUMENT);
  return rcutils_qsort(
    sequence->data,
    sequence->size,
    sizeof(rosidl_runtime_c__type_description__IndividualTypeDescription),
    rosidl_runtime_c_type_description_utils_referenced_type_description_sequence_sort_compare);
}


rcutils_ret_t
rosidl_runtime_c_type_description_utils_prune_referenced_type_descriptions_in_place(
  const rosidl_runtime_c__type_description__IndividualTypeDescription * main_type_description,
  rosidl_runtime_c__type_description__IndividualTypeDescription__Sequence * referenced_types)
{
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(main_type_description, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(referenced_types, RCUTILS_RET_INVALID_ARGUMENT);

  rcutils_ret_t ret = RCUTILS_RET_ERROR;
  rcutils_ret_t end_ret = RCUTILS_RET_ERROR;
  rcutils_ret_t fini_ret = RCUTILS_RET_ERROR;

  rcutils_hash_map_t * referenced_types_map = NULL;
  rcutils_hash_map_t * necessary_map = NULL;
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  ret = rosidl_runtime_c_type_description_utils_get_referenced_type_description_map(
    referenced_types, &allocator, &referenced_types_map);
  if (ret != RCUTILS_RET_OK) {
    RCUTILS_LOG_ERROR("Could not construct referenced type description map");
    return ret;
  }

  ret = rosidl_runtime_c_type_description_utils_get_necessary_referenced_type_descriptions_map(
    main_type_description, referenced_types_map, &allocator, &necessary_map);
  if (ret != RCUTILS_RET_OK) {
    RCUTILS_LOG_ERROR("Could not construct necessary referenced type description map");
    end_ret = ret;
    goto end_ref;
  }

  size_t map_length;
  ret = rcutils_hash_map_get_size(necessary_map, &map_length);
  if (ret != RCUTILS_RET_OK) {
    RCUTILS_LOG_ERROR("Could not get size of hash map for validation");
    end_ret = RCUTILS_RET_ERROR;
    goto end_necessary;
  }
  // End early if pruning was not needed
  if (referenced_types->size == map_length) {
    end_ret = RCUTILS_RET_OK;
    goto end_necessary;
  }

  size_t append_count = 0;
  char * key;
  rosidl_runtime_c__type_description__IndividualTypeDescription * data = NULL;
  rcutils_ret_t status = rcutils_hash_map_get_next_key_and_data(necessary_map, NULL, &key, &data);
  while (RCUTILS_RET_OK == status) {
    if (strcmp(key, data->type_name.data) != 0) {
      RCUTILS_LOG_ERROR(
        "Necessary referenced type name (%s) did not match key (%s)", data->type_name.data, key);
      end_ret = RCUTILS_RET_ERROR;
      goto end_necessary;
    }

    // Deep copy if necessary
    if (!rosidl_runtime_c__type_description__IndividualTypeDescription__are_equal(
        data, &referenced_types->data[append_count]))
    {
      if (!rosidl_runtime_c__type_description__IndividualTypeDescription__copy(
          data, &referenced_types->data[append_count++]))
      {
        RCUTILS_LOG_ERROR(
          "Could not copy necessary referenced type description %s to rearrange", key);
        end_ret = RCUTILS_RET_ERROR;
        goto end_necessary;
      }
    } else {
      append_count++;
    }
    status = rcutils_hash_map_get_next_key_and_data(necessary_map, &key, &key, &data);
  }

  // Finalize entries after the section of necessary referenced types, and shrink the input sequence
  for (size_t i = append_count; i < referenced_types->size; i++) {
    rosidl_runtime_c__type_description__IndividualTypeDescription__fini(
      &referenced_types->data[i]);
  }
  size_t allocation_size =
    append_count * sizeof(rosidl_runtime_c__type_description__IndividualTypeDescription);

  rosidl_runtime_c__type_description__IndividualTypeDescription * next_ptr =
    allocator.reallocate(referenced_types->data, allocation_size, allocator.state);
  if (next_ptr == NULL && allocation_size != 0) {
    RCUTILS_LOG_ERROR(
      "Could not shrink the necessary referenced type descriptions sequence during rearrangement! "
      "Beware! The referenced type descriptions was likely already partially modified in place!");
    end_ret = RCUTILS_RET_BAD_ALLOC;
    goto end_necessary;
  }
  referenced_types->data = next_ptr;
  referenced_types->size = append_count;
  referenced_types->capacity = append_count;

  end_ret = RCUTILS_RET_OK;

end_necessary:
  {
    fini_ret = rcutils_hash_map_fini(necessary_map);
    if (fini_ret != RCUTILS_RET_OK) {
      RCUTILS_LOG_ERROR("Failed to finalize hash map");
    }
    allocator.deallocate(necessary_map, allocator.state);
  }

end_ref:
  {
    fini_ret = rcutils_hash_map_fini(referenced_types_map);
    if (fini_ret != RCUTILS_RET_OK) {
      RCUTILS_LOG_ERROR("Failed to finalize hash map");
    }
    allocator.deallocate(referenced_types_map, allocator.state);
  }
  return end_ret;
}


bool
rosidl_runtime_c_type_description_utils_field_is_valid(
  const rosidl_runtime_c__type_description__Field * field)
{
  if (field == NULL) {
    RCUTILS_LOG_WARN("Field is invalid: Pointer is null");
    return false;
  }
  if (field->name.size == 0) {
    RCUTILS_LOG_WARN("Field is invalid: Empty name");
    return false;
  }
  if ((field->type.type_id % ROSIDL_RUNTIME_C_TYPE_DESCRIPTION_UTILS_SEQUENCE_TYPE_ID_DELIMITER) ==
    0)
  {
    RCUTILS_LOG_WARN("Field `%s` is invalid: Unset type ID", field->name.data);
    return false;
  }
  if ((field->type.type_id % ROSIDL_RUNTIME_C_TYPE_DESCRIPTION_UTILS_SEQUENCE_TYPE_ID_DELIMITER) ==
    1 && field->type.nested_type_name.size == 0)
  {
    RCUTILS_LOG_WARN(
      "Field `%s` is invalid: Field is nested but with empty nested type name", field->name.data);
    return false;
  }
  return true;
}


bool
rosidl_runtime_c_type_description_utils_individual_type_description_is_valid(
  const rosidl_runtime_c__type_description__IndividualTypeDescription * description)
{
  if (description == NULL) {
    RCUTILS_LOG_WARN("Individual type description is invalid: Pointer is null");
    return false;
  }
  if (description->type_name.size == 0) {
    RCUTILS_LOG_WARN("Individual type description is invalid: Empty name");
    return false;
  }

  for (size_t i = 0; i < description->fields.size; i++) {
    if (!rosidl_runtime_c_type_description_utils_field_is_valid(&description->fields.data[i])) {
      RCUTILS_LOG_WARN(
        "Individual type description `%s` is invalid: Invalid field", description->type_name.data);
      return false;
    }
  }

  bool end_ret = false;
  rcutils_hash_map_t * hash_map = NULL;
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  rcutils_ret_t ret = rosidl_runtime_c_type_description_utils_get_field_map(
    description, &allocator, &hash_map);
  if (ret != RCUTILS_RET_OK) {
    RCUTILS_LOG_ERROR("Could not construct field map for validation");
    return false;
  }

  size_t map_length;
  ret = rcutils_hash_map_get_size(hash_map, &map_length);
  if (ret != RCUTILS_RET_OK) {
    RCUTILS_LOG_ERROR("Could not get size of field map for validation");
    goto end;
  }

  if (description->fields.size != map_length) {
    RCUTILS_LOG_WARN(
      "Individual type description `%s` is invalid: Duplicate fields", description->type_name.data);
    goto end;
  }

  end_ret = true;

end:
  {
    rcutils_ret_t fini_ret = rcutils_hash_map_fini(hash_map);
    if (fini_ret != RCUTILS_RET_OK) {
      RCUTILS_LOG_ERROR("Failed to finalize hash map");
    }
    allocator.deallocate(hash_map, allocator.state);
    return end_ret;
  }
}


bool
rosidl_runtime_c_type_description_utils_type_description_is_valid(
  const rosidl_runtime_c__type_description__TypeDescription * description)
{
  if (description == NULL) {
    RCUTILS_LOG_WARN("Type description is invalid: Pointer is null");
    return false;
  }

  if (!rosidl_runtime_c_type_description_utils_individual_type_description_is_valid(
      &description->type_description))
  {
    if (description->type_description.type_name.size != 0) {
      RCUTILS_LOG_WARN(
        "Type description `%s` is invalid: Main type description is invalid",
        description->type_description.type_name.data);
    } else {
      RCUTILS_LOG_WARN("Type description is invalid: Main type description is invalid");
    }
    return false;
  }

  bool end_ret = false;
  rcutils_hash_map_t * referenced_types_map = NULL;
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  rcutils_ret_t ret = rosidl_runtime_c_type_description_utils_get_referenced_type_description_map(
    &description->referenced_type_descriptions, &allocator, &referenced_types_map);
  if (ret != RCUTILS_RET_OK) {
    RCUTILS_LOG_ERROR("Could not construct referenced type description map");
    return false;
  }

  size_t map_length;

  // Check no duplicated ref types
  ret = rcutils_hash_map_get_size(referenced_types_map, &map_length);
  if (ret != RCUTILS_RET_OK) {
    RCUTILS_LOG_ERROR("Could not get size of referenced type description map for validation");
    goto end_ref;
  }
  if (description->referenced_type_descriptions.size != map_length) {
    RCUTILS_LOG_WARN(
      "Type description `%s` is invalid: Duplicate referenced type descriptions",
      description->type_description.type_name.data);
    goto end_ref;
  }

  // Check no missing necessary ref types
  rcutils_hash_map_t * necessary_types_map = NULL;
  ret = rosidl_runtime_c_type_description_utils_get_necessary_referenced_type_descriptions_map(
    &description->type_description, referenced_types_map, &allocator, &necessary_types_map);
  if (ret != RCUTILS_RET_OK) {
    RCUTILS_LOG_ERROR("Could not construct necessary referenced type description map");
    goto end_ref;
  }

  // Check no unnecessary ref types
  ret = rcutils_hash_map_get_size(necessary_types_map, &map_length);
  if (ret != RCUTILS_RET_OK) {
    RCUTILS_LOG_ERROR(
      "Could not get size of necessary referenced type description map for validation");
    goto end_necessary;
  }

  if (description->referenced_type_descriptions.size != map_length) {
    RCUTILS_LOG_WARN(
      "Type description `%s` is invalid: Unnecessary referenced type descriptions",
      description->type_description.type_name.data);
    goto end_necessary;
  }

  // Check all referenced types valid (the prior checks ensure all of them are necessary)
  for (size_t i = 0; i < description->referenced_type_descriptions.size; i++) {
    if (!rosidl_runtime_c_type_description_utils_individual_type_description_is_valid(
        &description->referenced_type_descriptions.data[i]))
    {
      RCUTILS_LOG_WARN(
        "Type description `%s` is invalid: Invalid referenced type description",
        description->type_description.type_name.data);
      goto end_necessary;
    }
  }

  // Check referenced types sorted
  rosidl_runtime_c__type_description__IndividualTypeDescription__Sequence * sorted_sequence =
    rosidl_runtime_c__type_description__IndividualTypeDescription__Sequence__create(map_length);
  if (sorted_sequence == NULL) {
    RCUTILS_LOG_ERROR("Could allocate sequence for copy of referenced type descriptions");
    goto end_necessary;
  }
  if (!rosidl_runtime_c__type_description__IndividualTypeDescription__Sequence__copy(
      &description->referenced_type_descriptions, sorted_sequence))
  {
    RCUTILS_LOG_ERROR("Could not copy referenced type descriptions for validation");
    goto end_sequence;
  }
  ret = rosidl_runtime_c_type_description_utils_sort_referenced_type_descriptions_in_place(
    sorted_sequence);
  if (ret != RCUTILS_RET_OK) {
    RCUTILS_LOG_ERROR("Could not sort copy of referenced type descriptions for validation");
    goto end_sequence;
  }

  if (!rosidl_runtime_c__type_description__IndividualTypeDescription__Sequence__are_equal(
      &description->referenced_type_descriptions, sorted_sequence))
  {
    RCUTILS_LOG_WARN(
      "Type description `%s` is invalid: Referenced type descriptions not sorted",
      description->type_description.type_name.data);
    goto end_sequence;
  }

  end_ret = true;

end_sequence:
  rosidl_runtime_c__type_description__IndividualTypeDescription__Sequence__destroy(sorted_sequence);

end_necessary:
  ret = rcutils_hash_map_fini(necessary_types_map);
  if (ret != RCUTILS_RET_OK) {
    RCUTILS_LOG_ERROR("Failed to finalize referenced types map");
  }
  allocator.deallocate(necessary_types_map, allocator.state);

end_ref:
  ret = rcutils_hash_map_fini(referenced_types_map);
  if (ret != RCUTILS_RET_OK) {
    RCUTILS_LOG_ERROR("Failed to finalize referenced types map");
  }
  allocator.deallocate(referenced_types_map, allocator.state);

  return end_ret;
}


rcutils_ret_t
rosidl_runtime_c_type_description_utils_coerce_to_valid_type_description_in_place(
  rosidl_runtime_c__type_description__TypeDescription * type_description)
{
  if (!rosidl_runtime_c_type_description_utils_individual_type_description_is_valid(
      &type_description->type_description))
  {
    RCUTILS_LOG_ERROR("Could not make type description valid: Invalid main type description");
    return RCUTILS_RET_ERROR;
  }

  rcutils_ret_t ret;
  ret = rosidl_runtime_c_type_description_utils_prune_referenced_type_descriptions_in_place(
    &type_description->type_description, &type_description->referenced_type_descriptions);
  if (ret != RCUTILS_RET_OK) {
    RCUTILS_LOG_WARN(
      "Could not make type description valid: Could not prune referenced_type_descriptions");
    return ret;
  }

  ret = rosidl_runtime_c_type_description_utils_sort_referenced_type_descriptions_in_place(
    &type_description->referenced_type_descriptions);
  if (ret != RCUTILS_RET_OK) {
    RCUTILS_LOG_WARN(
      "Could not make type description valid: Could not sort referenced_type_descriptions");
    return ret;
  }

  return RCUTILS_RET_OK;
}


// =================================================================================================
// DESCRIPTION CONSTRUCTION
// =================================================================================================

rcutils_ret_t
rosidl_runtime_c_type_description_utils_create_field(
  const char * name, size_t name_length,
  uint8_t type_id, uint64_t capacity, uint64_t string_capacity,
  const char * nested_type_name, size_t nested_type_name_length,
  const char * default_value, size_t default_value_length,
  rosidl_runtime_c__type_description__Field ** field)
{
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(name, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(nested_type_name, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(default_value, RCUTILS_RET_INVALID_ARGUMENT);
  if (*field != NULL) {
    RCUTILS_LOG_ERROR("`field` output argument is not pointing to null");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  *field = rosidl_runtime_c__type_description__Field__create();
  if (*field == NULL) {
    RCUTILS_LOG_ERROR("Could not create field");
    return RCUTILS_RET_BAD_ALLOC;
  }

  // Field
  if (!rosidl_runtime_c__String__assignn(&(*field)->name, name, name_length)) {
    RCUTILS_LOG_ERROR("Could not assign field name");
    rosidl_runtime_c__type_description__Field__destroy(*field);
    *field = NULL;
    return RCUTILS_RET_BAD_ALLOC;
  }
  if (!rosidl_runtime_c__String__assignn(
      &(*field)->default_value, default_value, default_value_length))
  {
    RCUTILS_LOG_ERROR("Could not assign field default value");
    rosidl_runtime_c__type_description__Field__destroy(*field);
    *field = NULL;
    return RCUTILS_RET_BAD_ALLOC;
  }

  // FieldType
  (*field)->type.type_id = type_id;
  (*field)->type.capacity = capacity;
  (*field)->type.string_capacity = string_capacity;

  if (!rosidl_runtime_c__String__assignn(
      &(*field)->type.nested_type_name, nested_type_name, nested_type_name_length))
  {
    RCUTILS_LOG_ERROR("Could not assign field nested type name");
    rosidl_runtime_c__type_description__Field__destroy(*field);
    *field = NULL;
    return RCUTILS_RET_BAD_ALLOC;
  }

  return RCUTILS_RET_OK;
}


rcutils_ret_t
rosidl_runtime_c_type_description_utils_create_individual_type_description(
  const char * type_name, size_t type_name_length,
  rosidl_runtime_c__type_description__IndividualTypeDescription ** individual_description)
{
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(type_name, RCUTILS_RET_INVALID_ARGUMENT);
  if (*individual_description != NULL) {
    RCUTILS_LOG_ERROR("`individual_description` output argument is not pointing to null");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  *individual_description = rosidl_runtime_c__type_description__IndividualTypeDescription__create();
  if (*individual_description == NULL) {
    RCUTILS_LOG_ERROR("Could not create individual description");
    return RCUTILS_RET_BAD_ALLOC;
  }

  if (!rosidl_runtime_c__String__assignn(
      &(*individual_description)->type_name, type_name, type_name_length))
  {
    RCUTILS_LOG_ERROR("Could not assign individual description type name");
    rosidl_runtime_c__type_description__IndividualTypeDescription__destroy(*individual_description);
    *individual_description = NULL;
    return RCUTILS_RET_BAD_ALLOC;
  }
  return RCUTILS_RET_OK;
}


rcutils_ret_t
rosidl_runtime_c_type_description_utils_create_type_description(
  const char * type_name, size_t type_name_length,
  rosidl_runtime_c__type_description__TypeDescription ** type_description)
{
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(type_name, RCUTILS_RET_INVALID_ARGUMENT);
  if (*type_description != NULL) {
    RCUTILS_LOG_ERROR("`type_description` output argument is not pointing to null");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  *type_description = rosidl_runtime_c__type_description__TypeDescription__create();
  if (*type_description == NULL) {
    RCUTILS_LOG_ERROR("Could not create type description");
    return RCUTILS_RET_BAD_ALLOC;
  }

  if (!rosidl_runtime_c__String__assignn(
      &(*type_description)->type_description.type_name, type_name, type_name_length))
  {
    RCUTILS_LOG_ERROR("Could not assign main individual description type name");
    rosidl_runtime_c__type_description__TypeDescription__destroy(*type_description);
    *type_description = NULL;
    return RCUTILS_RET_BAD_ALLOC;
  }
  return RCUTILS_RET_OK;
}


rcutils_ret_t
rosidl_runtime_c_type_description_utils_append_field(
  rosidl_runtime_c__type_description__IndividualTypeDescription * individual_type_description,
  rosidl_runtime_c__type_description__Field * field)
{
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(individual_type_description, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(field, RCUTILS_RET_INVALID_ARGUMENT);

  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rcutils_ret_t fini_ret = RCUTILS_RET_ERROR;

  size_t allocation_size = (
    (individual_type_description->fields.size + 1) *
    sizeof(rosidl_runtime_c__type_description__Field)
  );
  size_t last_index = individual_type_description->fields.size;
  rosidl_runtime_c__type_description__Field * next_ptr = allocator.reallocate(
    individual_type_description->fields.data, allocation_size, allocator.state);
  if (next_ptr == NULL && allocation_size != 0) {
    RCUTILS_LOG_ERROR("Could not realloc individual type description fields sequence");
    return RCUTILS_RET_BAD_ALLOC;
  }
  individual_type_description->fields.data = next_ptr;
  individual_type_description->fields.size += 1;
  individual_type_description->fields.capacity += 1;

  if (!rosidl_runtime_c__type_description__Field__init(&next_ptr[last_index])) {
    RCUTILS_LOG_ERROR("Could not init new individual type description field element");
    fini_ret = RCUTILS_RET_BAD_ALLOC;
    goto fail;
  }

  if (!rosidl_runtime_c__type_description__Field__copy(field, &next_ptr[last_index])) {
    RCUTILS_LOG_ERROR("Could not copy into new individual type description field element");
    rosidl_runtime_c__type_description__Field__fini(&next_ptr[last_index]);
    fini_ret = RCUTILS_RET_ERROR;
    goto fail;
  }

  return RCUTILS_RET_OK;

fail:
  // Attempt to undo on failure
  next_ptr = allocator.reallocate(
    individual_type_description->fields.data,
    individual_type_description->fields.size * sizeof(rosidl_runtime_c__type_description__Field),
    allocator.state);
  if (next_ptr == NULL && individual_type_description->fields.size != 0) {
    RCUTILS_LOG_ERROR(
      "Could not shorten individual type description fields sequence. "
      "Excess memory will be UNINITIALIZED!");
  } else {
    individual_type_description->fields.data = next_ptr;
    individual_type_description->fields.size -= 1;
    individual_type_description->fields.capacity -= 1;
  }
  return fini_ret;
}


rcutils_ret_t
rosidl_runtime_c_type_description_utils_append_referenced_individual_type_description(
  rosidl_runtime_c__type_description__TypeDescription * type_description,
  rosidl_runtime_c__type_description__IndividualTypeDescription * referenced_type_description,
  bool sort)
{
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(type_description, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(referenced_type_description, RCUTILS_RET_INVALID_ARGUMENT);

  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rcutils_ret_t fini_ret = RCUTILS_RET_ERROR;

  size_t allocation_size = (
    (type_description->referenced_type_descriptions.size + 1) *
    sizeof(rosidl_runtime_c__type_description__IndividualTypeDescription)
  );
  size_t last_index = type_description->referenced_type_descriptions.size;

  rosidl_runtime_c__type_description__IndividualTypeDescription * next_ptr = allocator.reallocate(
    type_description->referenced_type_descriptions.data, allocation_size, allocator.state);
  if (next_ptr == NULL && allocation_size != 0) {
    RCUTILS_LOG_ERROR("Could not realloc type description referenced type descriptions sequence");
    return RCUTILS_RET_BAD_ALLOC;
  }
  type_description->referenced_type_descriptions.data = next_ptr;
  type_description->referenced_type_descriptions.size += 1;
  type_description->referenced_type_descriptions.capacity += 1;

  if (!rosidl_runtime_c__type_description__IndividualTypeDescription__init(&next_ptr[last_index])) {
    RCUTILS_LOG_ERROR("Could not init new type description referenced type descriptions element");
    fini_ret = RCUTILS_RET_BAD_ALLOC;
    goto fail;
  }

  if (!rosidl_runtime_c__type_description__IndividualTypeDescription__copy(
      referenced_type_description, &next_ptr[last_index]))
  {
    // Attempt to undo changes on failure
    RCUTILS_LOG_ERROR(
      "Could not copy into new type description referenced type descriptions element");
    rosidl_runtime_c__type_description__IndividualTypeDescription__fini(&next_ptr[last_index]);
    fini_ret = RCUTILS_RET_ERROR;
    goto fail;
  }

  if (sort) {
    rcutils_ret_t ret =
      rosidl_runtime_c_type_description_utils_sort_referenced_type_descriptions_in_place(
      &type_description->referenced_type_descriptions);
    if (ret != RCUTILS_RET_OK) {
      RCUTILS_LOG_WARN("Could not sort copy of referenced type descriptions for validation");
    }
  }
  return RCUTILS_RET_OK;

fail:
  // Attempt to undo on failure
  next_ptr = allocator.reallocate(
    type_description->referenced_type_descriptions.data,
    (
      type_description->referenced_type_descriptions.size *
      sizeof(rosidl_runtime_c__type_description__IndividualTypeDescription)
    ),
    allocator.state);
  if (next_ptr == NULL && type_description->referenced_type_descriptions.size != 0) {
    RCUTILS_LOG_ERROR(
      "Could not shorten type description referenced type descriptions sequence. "
      "Excess memory will be UNINITIALIZED!");
  } else {
    type_description->referenced_type_descriptions.data = next_ptr;
    type_description->referenced_type_descriptions.size -= 1;
    type_description->referenced_type_descriptions.capacity -= 1;
  }
  return fini_ret;
}


rcutils_ret_t
rosidl_runtime_c_type_description_utils_append_referenced_type_description(
  rosidl_runtime_c__type_description__TypeDescription * type_description,
  rosidl_runtime_c__type_description__TypeDescription * type_description_to_append,
  bool coerce_to_valid)
{
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(type_description, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(type_description_to_append, RCUTILS_RET_INVALID_ARGUMENT);

  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rcutils_ret_t fini_ret = RCUTILS_RET_ERROR;

  // +1 for the type_description_to_append's main type description
  size_t extend_count = type_description_to_append->referenced_type_descriptions.size + 1;
  size_t allocation_size = (
    (type_description->referenced_type_descriptions.size + extend_count) *
    sizeof(rosidl_runtime_c__type_description__IndividualTypeDescription)
  );
  rosidl_runtime_c__type_description__IndividualTypeDescription * next_ptr = allocator.reallocate(
    type_description->referenced_type_descriptions.data, allocation_size, allocator.state);
  if (next_ptr == NULL && allocation_size != 0) {
    RCUTILS_LOG_ERROR("Could not realloc type description referenced type descriptions sequence");
    return RCUTILS_RET_BAD_ALLOC;
  }

  size_t init_reset_size = 0;
  size_t last_index = type_description->referenced_type_descriptions.size;
  for (size_t i = last_index; i < last_index + extend_count; i++) {
    if (!rosidl_runtime_c__type_description__IndividualTypeDescription__init(&next_ptr[i])) {
      RCUTILS_LOG_ERROR("Could not init new type description referenced type descriptions element");
      fini_ret = RCUTILS_RET_BAD_ALLOC;
      goto fail;
    }
    init_reset_size += 1;
  }

  // Copy type_description_to_append's main type description
  if (!rosidl_runtime_c__type_description__IndividualTypeDescription__copy(
      &type_description_to_append->type_description, &next_ptr[last_index]))
  {
    RCUTILS_LOG_ERROR(
      "Could not copy into new type description referenced type descriptions element");
    fini_ret = RCUTILS_RET_ERROR;
    goto fail;
  }

  // Copy type_description_to_append's referenced type descriptions
  // There are (extend_count - 1) referenced type descriptions to copy
  for (size_t i = last_index + 1; i < last_index + extend_count; i++) {
    if (!rosidl_runtime_c__type_description__IndividualTypeDescription__copy(
        &type_description_to_append->referenced_type_descriptions.data[i - 1 - last_index],
        &next_ptr[i]))
    {
      RCUTILS_LOG_ERROR("Could not copy new type description referenced type descriptions element");
      fini_ret = RCUTILS_RET_BAD_ALLOC;
      goto fail;
    }
    init_reset_size += 1;
  }

  type_description->referenced_type_descriptions.data = next_ptr;
  type_description->referenced_type_descriptions.size += extend_count;
  type_description->referenced_type_descriptions.capacity += extend_count;

  if (coerce_to_valid) {
    rcutils_ret_t ret =
      rosidl_runtime_c_type_description_utils_coerce_to_valid_type_description_in_place(
      type_description);
    if (ret != RCUTILS_RET_OK) {
      RCUTILS_LOG_WARN("Could not coerce type description to valid!");
      return RCUTILS_RET_WARN;
    }
  }

  return RCUTILS_RET_OK;

fail:
  // Attempt to undo changes on failure
  for (size_t j = last_index; j < last_index + init_reset_size; j++) {
    rosidl_runtime_c__type_description__IndividualTypeDescription__fini(&next_ptr[j]);
  }
  next_ptr = allocator.reallocate(
    type_description->referenced_type_descriptions.data,
    (
      type_description->referenced_type_descriptions.size *
      sizeof(rosidl_runtime_c__type_description__IndividualTypeDescription)
    ),
    allocator.state);
  if (next_ptr == NULL && type_description->referenced_type_descriptions.size != 0) {
    RCUTILS_LOG_ERROR(
      "Could not shorten type description referenced type descriptions sequence. "
      "Excess memory will be UNINITIALIZED!");
    type_description->referenced_type_descriptions.size += extend_count;
    type_description->referenced_type_descriptions.capacity += extend_count;
  }
  return fini_ret;
}


rcutils_ret_t
rosidl_runtime_c_type_description_utils_get_referenced_type_description_as_type_description(
  const rosidl_runtime_c__type_description__IndividualTypeDescription__Sequence *
    referenced_descriptions,
  const rosidl_runtime_c__type_description__IndividualTypeDescription * referenced_description,
  rosidl_runtime_c__type_description__TypeDescription ** output_description,
  bool coerce_to_valid)
{
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(referenced_descriptions, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(referenced_description, RCUTILS_RET_INVALID_ARGUMENT);
  if (*output_description != NULL) {
    RCUTILS_LOG_ERROR("`output_description` output argument is not pointing to null");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  *output_description = rosidl_runtime_c__type_description__TypeDescription__create();
  if (*output_description == NULL) {
    RCUTILS_LOG_ERROR("Could not create output type description");
    return RCUTILS_RET_BAD_ALLOC;
  }

  if (!rosidl_runtime_c__type_description__IndividualTypeDescription__copy(
      referenced_description, &(*output_description)->type_description))
  {
    RCUTILS_LOG_ERROR("Could not copy referenced type description into main description");
    rosidl_runtime_c__type_description__TypeDescription__destroy(*output_description);
    *output_description = NULL;
    return RCUTILS_RET_ERROR;
  }

  if (!rosidl_runtime_c__type_description__IndividualTypeDescription__Sequence__copy(
      referenced_descriptions, &(*output_description)->referenced_type_descriptions))
  {
    RCUTILS_LOG_ERROR("Could not copy referenced type descriptions");
    rosidl_runtime_c__type_description__TypeDescription__destroy(*output_description);
    *output_description = NULL;
    return RCUTILS_RET_ERROR;
  }

  if (coerce_to_valid) {
    rcutils_ret_t ret =
      rosidl_runtime_c_type_description_utils_coerce_to_valid_type_description_in_place(
        *output_description);
    if (ret != RCUTILS_RET_OK) {
      RCUTILS_LOG_ERROR("Could not coerce output type description to valid");
      rosidl_runtime_c__type_description__TypeDescription__destroy(*output_description);
      *output_description = NULL;
      return ret;
    }
  }
  return RCUTILS_RET_OK;
}


// =================================================================================================
// DESCRIPTION PRINTING
// =================================================================================================

void
rosidl_runtime_c_type_description_utils_print_field_type(
  const rosidl_runtime_c__type_description__FieldType * field_type)
{
  printf(
    "  [FIELD TYPE]\n"
    "    type_id: %d\n"
    "    capacity: %ld\n"
    "    string_capacity: %ld\n",
    field_type->type_id, field_type->capacity, field_type->string_capacity);

  if (field_type->nested_type_name.data == NULL) {
    printf("    nested_type_name: %s\n", field_type->nested_type_name.data);
  } else {
    printf("    nested_type_name: \"%s\"\n", field_type->nested_type_name.data);
  }
}


void
rosidl_runtime_c_type_description_utils_print_field(
  const rosidl_runtime_c__type_description__Field * field)
{
  printf("[FIELD]\n");

  if (field->name.data == NULL) {
    printf("  name: %s\n", field->name.data);
  } else {
    printf("  name: \"%s\"\n", field->name.data);
  }

  if (field->default_value.data == NULL) {
    printf("  default_value: %s\n", field->default_value.data);
  } else {
    printf("  default_value: \"%s\"\n", field->default_value.data);
  }

  rosidl_runtime_c_type_description_utils_print_field_type(&field->type);
}


void
rosidl_runtime_c_type_description_utils_print_individual_type_description(
  const rosidl_runtime_c__type_description__IndividualTypeDescription * individual_type_description)
{
  printf(
    "\n[INDIVIDUAL TYPE DESCRIPTION] (Fields: %ld)\n", individual_type_description->fields.size);

  if (individual_type_description->type_name.data == NULL) {
    printf("  type_name: %s\n", individual_type_description->type_name.data);
  } else {
    printf("  type_name: \"%s\"\n", individual_type_description->type_name.data);
  }

  for (size_t i = 0; i < individual_type_description->fields.size; i++) {
    rosidl_runtime_c_type_description_utils_print_field(
      &individual_type_description->fields.data[i]);
  }
}


void rosidl_runtime_c_type_description_utils_print_type_description(
  const rosidl_runtime_c__type_description__TypeDescription * type_description)
{
  printf("\n\n---\n\n");

  printf(
    "= [PRINTING TYPE DESCRIPTION] = (Referenced descriptions: %ld)\n",
    type_description->referenced_type_descriptions.size);

  printf("\n== [MAIN DESCRIPTION] ==\n");
  rosidl_runtime_c_type_description_utils_print_individual_type_description(
    &type_description->type_description);

  printf("\n== [REFERENCED DESCRIPTIONS] ==\n");
  for (size_t i = 0; i < type_description->referenced_type_descriptions.size; i++) {
    rosidl_runtime_c_type_description_utils_print_individual_type_description(
      &type_description->referenced_type_descriptions.data[i]);
  }

  printf("\n---\n\n");
}


void
rosidl_runtime_c_type_description_utils_print_field_map(const rcutils_hash_map_t * hash_map)
{
  char * key;
  rosidl_runtime_c__type_description__Field * data = NULL;
  rcutils_ret_t status = rcutils_hash_map_get_next_key_and_data(hash_map, NULL, &key, &data);
  while (RCUTILS_RET_OK == status) {
    if (key == NULL) {
      printf("\n== KEY: %s ==\n", key);
    } else {
      printf("\n== KEY: \"%s\" ==\n", key);
    }
    rosidl_runtime_c_type_description_utils_print_field(data);
    status = rcutils_hash_map_get_next_key_and_data(hash_map, &key, &key, &data);
  }
}


void
rosidl_runtime_c_type_description_utils_print_referenced_type_description_map(
  const rcutils_hash_map_t * hash_map)
{
  char * key;
  rosidl_runtime_c__type_description__IndividualTypeDescription * data;
  rcutils_ret_t status = rcutils_hash_map_get_next_key_and_data(hash_map, NULL, &key, &data);
  while (RCUTILS_RET_OK == status) {
    if (key == NULL) {
      printf("\n== KEY: %s ==\n", key);
    } else {
      printf("\n== KEY: \"%s\" ==\n", key);
    }
    rosidl_runtime_c_type_description_utils_print_individual_type_description(data);
    status = rcutils_hash_map_get_next_key_and_data(hash_map, &key, &key, &data);
  }
}
