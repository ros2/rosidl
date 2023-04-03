// DO NOT EDIT MANUALLY - this copied file managed by copy_type_description_generated_sources.bash
// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from type_description_interfaces:msg/FieldType.idl
// generated code does not contain a copyright notice

#include "rosidl_runtime_c/type_description/field_type__functions.h"

ROSIDL_GENERATOR_C_PUBLIC
const rosidl_type_hash_t *
rosidl_runtime_c__type_description__FieldType__get_type_hash(const rosidl_message_type_support_t *)
{
  static rosidl_type_hash_t hash = {1, {
      0xa7, 0x0b, 0x6d, 0xd9, 0x19, 0x64, 0x5a, 0x03,
      0xa3, 0x58, 0x6f, 0x7f, 0x82, 0x1d, 0xef, 0xbc,
      0x88, 0x6e, 0xa3, 0xe5, 0x31, 0xa1, 0xd9, 0x5c,
      0xc0, 0xf3, 0x80, 0xa3, 0x97, 0x3c, 0xca, 0xa6,
    }};
  return &hash;
}


#include <assert.h>
#include <string.h>
// Include directives for referenced types

// Expected hashes for externally referenced types

// Names for all types
static char rosidl_runtime_c__type_description__FieldType__TYPE_NAME[] = "type_description_interfaces/msg/FieldType";

// Define type names, field names, and default values
static char rosidl_runtime_c__type_description__FieldType__FIELD_NAME__type_id[] = "type_id";
static char rosidl_runtime_c__type_description__FieldType__DEFAULT_VALUE__type_id[] = "0";
static char rosidl_runtime_c__type_description__FieldType__FIELD_NAME__capacity[] = "capacity";
static char rosidl_runtime_c__type_description__FieldType__FIELD_NAME__string_capacity[] = "string_capacity";
static char rosidl_runtime_c__type_description__FieldType__FIELD_NAME__nested_type_name[] = "nested_type_name";

/// Define arrays of Fields
static rosidl_runtime_c__type_description__Field rosidl_runtime_c__type_description__FieldType__FIELDS[] = {
  {
    {rosidl_runtime_c__type_description__FieldType__FIELD_NAME__type_id, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {rosidl_runtime_c__type_description__FieldType__DEFAULT_VALUE__type_id, 1, 1},
  },
  {
    {rosidl_runtime_c__type_description__FieldType__FIELD_NAME__capacity, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {rosidl_runtime_c__type_description__FieldType__FIELD_NAME__string_capacity, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {rosidl_runtime_c__type_description__FieldType__FIELD_NAME__nested_type_name, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOUNDED_STRING,
      0,
      255,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

/// Define exported TypeDescription and TypeSources

const rosidl_runtime_c__type_description__TypeDescription *
rosidl_runtime_c__type_description__FieldType__get_type_description(const rosidl_message_type_support_t *)
{
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {rosidl_runtime_c__type_description__FieldType__TYPE_NAME, 41, 41},
      {rosidl_runtime_c__type_description__FieldType__FIELDS, 4, 4},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
rosidl_runtime_c__type_description__FieldType__get_type_description_sources(const rosidl_message_type_support_t *)
{
  static const rosidl_runtime_c__type_description__TypeSource__Sequence sources = {NULL, 0, 0};
  return &sources;
}
