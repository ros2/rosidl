// DO NOT EDIT MANUALLY - this copied file managed by copy_type_description_generated_sources.bash
// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from type_description_interfaces:msg/TypeDescription.idl
// generated code does not contain a copyright notice

#include "rosidl_runtime_c/type_description/type_description__functions.h"

ROSIDL_GENERATOR_C_PUBLIC
const rosidl_type_hash_t *
rosidl_runtime_c__type_description__TypeDescription__get_type_hash(const rosidl_message_type_support_t *)
{
  static rosidl_type_hash_t hash = {1, {
      0x73, 0x9f, 0x25, 0x08, 0xc9, 0xfa, 0x3a, 0x6f,
      0x33, 0x09, 0x13, 0xff, 0x5b, 0x9d, 0x25, 0xfb,
      0x74, 0x15, 0x9a, 0x07, 0x7d, 0xa7, 0x1e, 0x10,
      0x87, 0xf5, 0x1a, 0x60, 0xc1, 0x2a, 0x08, 0x0b,
    }};
  return &hash;
}


#include <assert.h>
#include <string.h>
// Include directives for referenced types
#include "rosidl_runtime_c/type_description/individual_type_description__functions.h"
#include "rosidl_runtime_c/type_description/field_type__functions.h"
#include "rosidl_runtime_c/type_description/field__functions.h"

// Expected hashes for externally referenced types
static const rosidl_type_hash_t rosidl_runtime_c__type_description__Field__EXPECTED_HASH = {1, {
    0xc0, 0xb0, 0x13, 0x79, 0xcd, 0x42, 0x26, 0x28,
    0x12, 0x85, 0xcc, 0xaf, 0x6b, 0xe4, 0x66, 0x53,
    0x96, 0x8f, 0x85, 0x5f, 0x7c, 0x5e, 0x41, 0x61,
    0x4f, 0xf5, 0xd7, 0xa8, 0x54, 0xef, 0xef, 0x7c,
  }};
static const rosidl_type_hash_t rosidl_runtime_c__type_description__FieldType__EXPECTED_HASH = {1, {
    0xa7, 0x0b, 0x6d, 0xd9, 0x19, 0x64, 0x5a, 0x03,
    0xa3, 0x58, 0x6f, 0x7f, 0x82, 0x1d, 0xef, 0xbc,
    0x88, 0x6e, 0xa3, 0xe5, 0x31, 0xa1, 0xd9, 0x5c,
    0xc0, 0xf3, 0x80, 0xa3, 0x97, 0x3c, 0xca, 0xa6,
  }};
static const rosidl_type_hash_t rosidl_runtime_c__type_description__IndividualTypeDescription__EXPECTED_HASH = {1, {
    0x55, 0xc8, 0x27, 0xd8, 0x6c, 0x3c, 0x14, 0x1b,
    0xdd, 0x31, 0x8f, 0xe6, 0xc2, 0x2e, 0x11, 0x19,
    0x0e, 0x4d, 0x3b, 0x37, 0xc8, 0xf4, 0xf9, 0x75,
    0x1a, 0x08, 0x4a, 0xa0, 0x5c, 0xe9, 0x65, 0x60,
  }};

// Names for all types
static char rosidl_runtime_c__type_description__TypeDescription__TYPE_NAME[] = "type_description_interfaces/msg/TypeDescription";
static char rosidl_runtime_c__type_description__Field__TYPE_NAME[] = "type_description_interfaces/msg/Field";
static char rosidl_runtime_c__type_description__FieldType__TYPE_NAME[] = "type_description_interfaces/msg/FieldType";
static char rosidl_runtime_c__type_description__IndividualTypeDescription__TYPE_NAME[] = "type_description_interfaces/msg/IndividualTypeDescription";

// Define type names, field names, and default values
static char rosidl_runtime_c__type_description__TypeDescription__FIELD_NAME__type_description[] = "type_description";
static char rosidl_runtime_c__type_description__TypeDescription__FIELD_NAME__referenced_type_descriptions[] = "referenced_type_descriptions";

/// Define arrays of Fields
static rosidl_runtime_c__type_description__Field rosidl_runtime_c__type_description__TypeDescription__FIELDS[] = {
  {
    {rosidl_runtime_c__type_description__TypeDescription__FIELD_NAME__type_description, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {rosidl_runtime_c__type_description__IndividualTypeDescription__TYPE_NAME, 57, 57},
    },
    {NULL, 0, 0},
  },
  {
    {rosidl_runtime_c__type_description__TypeDescription__FIELD_NAME__referenced_type_descriptions, 28, 28},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {rosidl_runtime_c__type_description__IndividualTypeDescription__TYPE_NAME, 57, 57},
    },
    {NULL, 0, 0},
  },
};

/// Define exported TypeDescription and TypeSources
static rosidl_runtime_c__type_description__IndividualTypeDescription rosidl_runtime_c__type_description__TypeDescription__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {rosidl_runtime_c__type_description__Field__TYPE_NAME, 37, 37},
    {NULL, 0, 0},
  },
  {
    {rosidl_runtime_c__type_description__FieldType__TYPE_NAME, 41, 41},
    {NULL, 0, 0},
  },
  {
    {rosidl_runtime_c__type_description__IndividualTypeDescription__TYPE_NAME, 57, 57},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
rosidl_runtime_c__type_description__TypeDescription__get_type_description(const rosidl_message_type_support_t *)
{
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {rosidl_runtime_c__type_description__TypeDescription__TYPE_NAME, 47, 47},
      {rosidl_runtime_c__type_description__TypeDescription__FIELDS, 2, 2},
    },
    {rosidl_runtime_c__type_description__TypeDescription__REFERENCED_TYPE_DESCRIPTIONS, 3, 3},
  };
  if (!constructed) {
    {
      assert(0 == memcmp(&rosidl_runtime_c__type_description__Field__EXPECTED_HASH, rosidl_runtime_c__type_description__Field__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
      const rosidl_runtime_c__type_description__TypeDescription * ref_desc = rosidl_runtime_c__type_description__Field__get_type_description(NULL);
      description.referenced_type_descriptions.data[0].fields.data = ref_desc->type_description.fields.data;
      description.referenced_type_descriptions.data[0].fields.size = ref_desc->type_description.fields.size;
      description.referenced_type_descriptions.data[0].fields.capacity = ref_desc->type_description.fields.capacity;
    }
    {
      assert(0 == memcmp(&rosidl_runtime_c__type_description__FieldType__EXPECTED_HASH, rosidl_runtime_c__type_description__FieldType__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
      const rosidl_runtime_c__type_description__TypeDescription * ref_desc = rosidl_runtime_c__type_description__FieldType__get_type_description(NULL);
      description.referenced_type_descriptions.data[1].fields.data = ref_desc->type_description.fields.data;
      description.referenced_type_descriptions.data[1].fields.size = ref_desc->type_description.fields.size;
      description.referenced_type_descriptions.data[1].fields.capacity = ref_desc->type_description.fields.capacity;
    }
    {
      assert(0 == memcmp(&rosidl_runtime_c__type_description__IndividualTypeDescription__EXPECTED_HASH, rosidl_runtime_c__type_description__IndividualTypeDescription__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
      const rosidl_runtime_c__type_description__TypeDescription * ref_desc = rosidl_runtime_c__type_description__IndividualTypeDescription__get_type_description(NULL);
      description.referenced_type_descriptions.data[2].fields.data = ref_desc->type_description.fields.data;
      description.referenced_type_descriptions.data[2].fields.size = ref_desc->type_description.fields.size;
      description.referenced_type_descriptions.data[2].fields.capacity = ref_desc->type_description.fields.capacity;
    }
    constructed = true;
  }
  return &description;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
rosidl_runtime_c__type_description__TypeDescription__get_type_description_sources(const rosidl_message_type_support_t *)
{
  static const rosidl_runtime_c__type_description__TypeSource__Sequence sources = {NULL, 0, 0};
  return &sources;
}
