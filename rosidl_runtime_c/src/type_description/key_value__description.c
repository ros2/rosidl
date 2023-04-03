// DO NOT EDIT MANUALLY - this copied file managed by copy_type_description_generated_sources.bash
// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from type_description_interfaces:msg/KeyValue.idl
// generated code does not contain a copyright notice

#include "rosidl_runtime_c/type_description/key_value__functions.h"

ROSIDL_GENERATOR_C_PUBLIC
const rosidl_type_hash_t *
rosidl_runtime_c__type_description__KeyValue__get_type_hash(const rosidl_message_type_support_t *)
{
  static rosidl_type_hash_t hash = {1, {
      0x27, 0x4f, 0xe5, 0x6b, 0xf1, 0x4f, 0x33, 0xc7,
      0x51, 0x2e, 0x34, 0xc6, 0x46, 0xa3, 0x75, 0x79,
      0xee, 0x36, 0x77, 0x9f, 0x74, 0x5f, 0x04, 0x9a,
      0x97, 0x60, 0x76, 0x3e, 0x81, 0x7f, 0x0c, 0x42,
    }};
  return &hash;
}


#include <assert.h>
#include <string.h>
// Include directives for referenced types

// Expected hashes for externally referenced types

// Names for all types
static char rosidl_runtime_c__type_description__KeyValue__TYPE_NAME[] = "type_description_interfaces/msg/KeyValue";

// Define type names, field names, and default values
static char rosidl_runtime_c__type_description__KeyValue__FIELD_NAME__key[] = "key";
static char rosidl_runtime_c__type_description__KeyValue__FIELD_NAME__value[] = "value";

/// Define arrays of Fields
static rosidl_runtime_c__type_description__Field rosidl_runtime_c__type_description__KeyValue__FIELDS[] = {
  {
    {rosidl_runtime_c__type_description__KeyValue__FIELD_NAME__key, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {rosidl_runtime_c__type_description__KeyValue__FIELD_NAME__value, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

/// Define exported TypeDescription and TypeSources

const rosidl_runtime_c__type_description__TypeDescription *
rosidl_runtime_c__type_description__KeyValue__get_type_description(const rosidl_message_type_support_t *)
{
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {rosidl_runtime_c__type_description__KeyValue__TYPE_NAME, 40, 40},
      {rosidl_runtime_c__type_description__KeyValue__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
rosidl_runtime_c__type_description__KeyValue__get_type_description_sources(const rosidl_message_type_support_t *)
{
  static const rosidl_runtime_c__type_description__TypeSource__Sequence sources = {NULL, 0, 0};
  return &sources;
}
