// DO NOT EDIT MANUALLY - this copied file managed by copy_type_description_generated_sources.bash
// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from type_description_interfaces:msg/TypeSource.idl
// generated code does not contain a copyright notice

#include "rosidl_runtime_c/type_description/type_source__functions.h"

ROSIDL_GENERATOR_C_PUBLIC
const rosidl_type_hash_t *
rosidl_runtime_c__type_description__TypeSource__get_type_hash(const rosidl_message_type_support_t *)
{
  static rosidl_type_hash_t hash = {1, {
      0xfa, 0xea, 0xec, 0x75, 0x96, 0xc0, 0x4e, 0xcf,
      0x5b, 0x6e, 0x99, 0xad, 0x22, 0x5e, 0x4c, 0x7c,
      0xbb, 0x99, 0x7a, 0xd5, 0x43, 0x5f, 0x79, 0x35,
      0x26, 0xfb, 0x39, 0x84, 0xd0, 0x11, 0xaa, 0xe5,
    }};
  return &hash;
}


#include <assert.h>
#include <string.h>
// Include directives for referenced types

// Expected hashes for externally referenced types

// Names for all types
static char rosidl_runtime_c__type_description__TypeSource__TYPE_NAME[] = "type_description_interfaces/msg/TypeSource";

// Define type names, field names, and default values
static char rosidl_runtime_c__type_description__TypeSource__FIELD_NAME__type_name[] = "type_name";
static char rosidl_runtime_c__type_description__TypeSource__FIELD_NAME__encoding[] = "encoding";
static char rosidl_runtime_c__type_description__TypeSource__FIELD_NAME__raw_file_contents[] = "raw_file_contents";

/// Define arrays of Fields
static rosidl_runtime_c__type_description__Field rosidl_runtime_c__type_description__TypeSource__FIELDS[] = {
  {
    {rosidl_runtime_c__type_description__TypeSource__FIELD_NAME__type_name, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {rosidl_runtime_c__type_description__TypeSource__FIELD_NAME__encoding, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {rosidl_runtime_c__type_description__TypeSource__FIELD_NAME__raw_file_contents, 17, 17},
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
rosidl_runtime_c__type_description__TypeSource__get_type_description(const rosidl_message_type_support_t *)
{
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {rosidl_runtime_c__type_description__TypeSource__TYPE_NAME, 42, 42},
      {rosidl_runtime_c__type_description__TypeSource__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
rosidl_runtime_c__type_description__TypeSource__get_type_description_sources(const rosidl_message_type_support_t *)
{
  static const rosidl_runtime_c__type_description__TypeSource__Sequence sources = {NULL, 0, 0};
  return &sources;
}
