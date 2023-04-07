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

#include <stdlib.h>
#include <gtest/gtest.h>

#include <rcutils/error_handling.h>
#include <rcutils/types/rcutils_ret.h>
#include <rcutils/types/hash_map.h>
#include <rosidl_runtime_c/string_functions.h>

#include <cstring>

#include "rosidl_runtime_c/type_description/field__functions.h"
#include "rosidl_runtime_c/type_description/field__struct.h"
#include "rosidl_runtime_c/type_description/individual_type_description__functions.h"
#include "rosidl_runtime_c/type_description/individual_type_description__struct.h"
#include "rosidl_runtime_c/type_description/type_description__functions.h"
#include "rosidl_runtime_c/type_description/type_description__struct.h"
#include "rosidl_runtime_c/type_description_utils.h"


TEST(TestUtils, test_basic_construction)
{
  const std::string test_name = "test_name";

  // TypeDescription
  {
    rosidl_runtime_c__type_description__TypeDescription * desc =
      rosidl_runtime_c__type_description__TypeDescription__create();
    ASSERT_TRUE(desc);
    ASSERT_TRUE(
      rosidl_runtime_c__String__assignn(
        &desc->type_description.type_name, test_name.c_str(), test_name.size()));

    rosidl_runtime_c__type_description__TypeDescription * utils_desc = NULL;
    EXPECT_EQ(
      rosidl_runtime_c_type_description_utils_create_type_description(
        test_name.c_str(),
        test_name.size(), &utils_desc),
      RCUTILS_RET_OK);
    ASSERT_TRUE(utils_desc);

    EXPECT_FALSE(utils_desc->type_description.fields.data);
    EXPECT_EQ(utils_desc->type_description.fields.size, 0);
    EXPECT_EQ(utils_desc->type_description.fields.capacity, 0);

    EXPECT_FALSE(utils_desc->referenced_type_descriptions.data);
    EXPECT_EQ(utils_desc->referenced_type_descriptions.size, 0);
    EXPECT_EQ(utils_desc->referenced_type_descriptions.capacity, 0);

    EXPECT_TRUE(rosidl_runtime_c__type_description__TypeDescription__are_equal(desc, utils_desc));

    rosidl_runtime_c__type_description__TypeDescription__destroy(desc);
    rosidl_runtime_c__type_description__TypeDescription__destroy(utils_desc);
  }

  // IndividualTypeDescription
  {
    rosidl_runtime_c__type_description__IndividualTypeDescription * individual_desc =
      rosidl_runtime_c__type_description__IndividualTypeDescription__create();
    ASSERT_TRUE(individual_desc);
    ASSERT_TRUE(
      rosidl_runtime_c__String__assignn(
        &individual_desc->type_name, test_name.c_str(), test_name.size()));

    rosidl_runtime_c__type_description__IndividualTypeDescription * individual_utils_desc = NULL;
    EXPECT_EQ(
      rosidl_runtime_c_type_description_utils_create_individual_type_description(
        test_name.c_str(), test_name.size(), &individual_utils_desc),
      RCUTILS_RET_OK);

    ASSERT_TRUE(individual_utils_desc);

    EXPECT_FALSE(individual_utils_desc->fields.data);
    EXPECT_EQ(individual_utils_desc->fields.size, 0);
    EXPECT_EQ(individual_utils_desc->fields.capacity, 0);

    EXPECT_TRUE(
      rosidl_runtime_c__type_description__IndividualTypeDescription__are_equal(
        individual_desc, individual_utils_desc));

    rosidl_runtime_c__type_description__IndividualTypeDescription__destroy(individual_desc);
    rosidl_runtime_c__type_description__IndividualTypeDescription__destroy(individual_utils_desc);
  }

  // Field
  {
    rosidl_runtime_c__type_description__Field * field =
      rosidl_runtime_c__type_description__Field__create();
    ASSERT_TRUE(field);
    ASSERT_TRUE(
      rosidl_runtime_c__String__assignn(
        &field->name, test_name.c_str(), test_name.size()));

    rosidl_runtime_c__type_description__Field * utils_field = NULL;
    EXPECT_EQ(
      rosidl_runtime_c_type_description_utils_create_field(
        test_name.c_str(), test_name.size(), 0,  // Name, Name Length, Type ID
        0, 0,                                    // Capacity, String Capacity
        "", 0,                                   // Nested Type Name, Nested Type Name Length
        "", 0,                                   // Default Value, Default Value Length
        &utils_field),
      RCUTILS_RET_OK);
    ASSERT_TRUE(utils_field);

    EXPECT_TRUE(utils_field->default_value.data);

    EXPECT_EQ(utils_field->default_value.size, 0);
    EXPECT_EQ(utils_field->default_value.capacity, 1);  // Null terminator

    EXPECT_TRUE(rosidl_runtime_c__type_description__Field__are_equal(field, utils_field));

    rosidl_runtime_c__type_description__Field__destroy(field);
    rosidl_runtime_c__type_description__Field__destroy(utils_field);
  }
}


class TestUtilsFixture : public ::testing::Test
{
public:
  void SetUp()
  {
    type_description_1 = NULL;
    rosidl_runtime_c_type_description_utils_create_type_description("t1", 2, &type_description_1);
    ASSERT_TRUE(type_description_1);

    type_description_2 = NULL;
    rosidl_runtime_c_type_description_utils_create_type_description("t2", 2, &type_description_2);
    ASSERT_TRUE(type_description_2);

    type_description_3 = NULL;
    rosidl_runtime_c_type_description_utils_create_type_description("t3", 2, &type_description_3);
    ASSERT_TRUE(type_description_3);


    individual_desc_1 = NULL;
    rosidl_runtime_c_type_description_utils_create_individual_type_description(
      "i1", 2, &individual_desc_1);
    ASSERT_TRUE(individual_desc_1);

    individual_desc_2 = NULL;
    rosidl_runtime_c_type_description_utils_create_individual_type_description(
      "i2", 2, &individual_desc_2);
    ASSERT_TRUE(individual_desc_2);

    individual_desc_3 = NULL;
    rosidl_runtime_c_type_description_utils_create_individual_type_description(
      "i3", 2, &individual_desc_3);
    ASSERT_TRUE(individual_desc_3);

    empty_individual_desc = NULL;
    rosidl_runtime_c_type_description_utils_create_individual_type_description(
      "empty", 5, &empty_individual_desc);
    ASSERT_TRUE(empty_individual_desc);


    field_1 = NULL;  // Nested
    rosidl_runtime_c_type_description_utils_create_field(
      "f1", 2, 1,                 // Name, Name Length, Type ID
      0, 0,                       // Capacity, String Capacity
      "empty", 5,                 // Nested Type Name, Nested Type Name Length
      "", 0,                      // Default Value, Default Value Length
      &field_1);
    ASSERT_TRUE(field_1);

    field_2 = NULL;
    rosidl_runtime_c_type_description_utils_create_field(
      "f2", 2, 2,  // Name, Name Length, Type ID
      0, 0, "", 0, "", 0, &field_2);
    ASSERT_TRUE(field_2);

    field_3 = NULL;
    rosidl_runtime_c_type_description_utils_create_field(
      "f3", 2, 3,  // Name, Name Length, Type ID
      0, 0, "", 0, "", 0, &field_3);
    ASSERT_TRUE(field_3);
  }

  void TearDown()
  {
    rosidl_runtime_c__type_description__TypeDescription__destroy(type_description_1);
    rosidl_runtime_c__type_description__TypeDescription__destroy(type_description_2);
    rosidl_runtime_c__type_description__TypeDescription__destroy(type_description_3);
    rosidl_runtime_c__type_description__IndividualTypeDescription__destroy(individual_desc_1);
    rosidl_runtime_c__type_description__IndividualTypeDescription__destroy(individual_desc_2);
    rosidl_runtime_c__type_description__IndividualTypeDescription__destroy(individual_desc_3);
    rosidl_runtime_c__type_description__IndividualTypeDescription__destroy(empty_individual_desc);
    rosidl_runtime_c__type_description__Field__destroy(field_1);
    rosidl_runtime_c__type_description__Field__destroy(field_2);
    rosidl_runtime_c__type_description__Field__destroy(field_3);
  }

  rosidl_runtime_c__type_description__TypeDescription * type_description_1;
  rosidl_runtime_c__type_description__TypeDescription * type_description_2;
  rosidl_runtime_c__type_description__TypeDescription * type_description_3;

  rosidl_runtime_c__type_description__IndividualTypeDescription * individual_desc_1;
  rosidl_runtime_c__type_description__IndividualTypeDescription * individual_desc_2;
  rosidl_runtime_c__type_description__IndividualTypeDescription * individual_desc_3;
  rosidl_runtime_c__type_description__IndividualTypeDescription * empty_individual_desc;

  rosidl_runtime_c__type_description__Field * field_1;
  rosidl_runtime_c__type_description__Field * field_2;
  rosidl_runtime_c__type_description__Field * field_3;

  rcutils_ret_t ret = RCUTILS_RET_ERROR;
};


TEST_F(TestUtilsFixture, test_appends_and_advanced_construction)
{
  // FIELD APPEND
  EXPECT_EQ(individual_desc_1->fields.size, 0);
  EXPECT_EQ(individual_desc_1->fields.capacity, 0);
  ret = rosidl_runtime_c_type_description_utils_append_field(individual_desc_1, field_1);
  EXPECT_EQ(ret, RCUTILS_RET_OK);
  EXPECT_EQ(individual_desc_1->fields.size, 1);
  EXPECT_EQ(individual_desc_1->fields.capacity, 1);

  ret = rosidl_runtime_c_type_description_utils_append_field(individual_desc_1, field_2);
  EXPECT_EQ(ret, RCUTILS_RET_OK);
  EXPECT_EQ(individual_desc_1->fields.size, 2);
  EXPECT_EQ(individual_desc_1->fields.capacity, 2);

  ret = rosidl_runtime_c_type_description_utils_append_field(individual_desc_1, field_3);
  EXPECT_EQ(ret, RCUTILS_RET_OK);
  EXPECT_EQ(individual_desc_1->fields.size, 3);
  EXPECT_EQ(individual_desc_1->fields.capacity, 3);

  EXPECT_TRUE(
    rosidl_runtime_c__type_description__Field__are_equal(
      &individual_desc_1->fields.data[0], field_1));
  EXPECT_TRUE(
    rosidl_runtime_c__type_description__Field__are_equal(
      &individual_desc_1->fields.data[1], field_2));
  EXPECT_TRUE(
    rosidl_runtime_c__type_description__Field__are_equal(
      &individual_desc_1->fields.data[2], field_3));


  // REFERENCED INDIVIDUAL TYPE DESCRIPTION APPEND
  // Append out of order
  EXPECT_EQ(type_description_1->referenced_type_descriptions.size, 0);
  EXPECT_EQ(type_description_1->referenced_type_descriptions.capacity, 0);
  ret = rosidl_runtime_c_type_description_utils_append_referenced_individual_type_description(
    type_description_1, individual_desc_1, false);
  ASSERT_EQ(ret, RCUTILS_RET_OK);
  EXPECT_EQ(type_description_1->referenced_type_descriptions.size, 1);
  EXPECT_EQ(type_description_1->referenced_type_descriptions.capacity, 1);

  ret = rosidl_runtime_c_type_description_utils_append_referenced_individual_type_description(
    type_description_1, individual_desc_3, false);
  ASSERT_EQ(ret, RCUTILS_RET_OK);
  EXPECT_EQ(type_description_1->referenced_type_descriptions.size, 2);
  EXPECT_EQ(type_description_1->referenced_type_descriptions.capacity, 2);

  EXPECT_TRUE(
    rosidl_runtime_c__type_description__IndividualTypeDescription__are_equal(
      &type_description_1->referenced_type_descriptions.data[0], individual_desc_1));

  EXPECT_TRUE(
    rosidl_runtime_c__type_description__IndividualTypeDescription__are_equal(
      &type_description_1->referenced_type_descriptions.data[1], individual_desc_3));


  // Append and Sort
  ret = rosidl_runtime_c_type_description_utils_append_referenced_individual_type_description(
    type_description_1, individual_desc_2, true);
  ASSERT_EQ(ret, RCUTILS_RET_OK);
  EXPECT_EQ(type_description_1->referenced_type_descriptions.size, 3);
  EXPECT_EQ(type_description_1->referenced_type_descriptions.capacity, 3);

  EXPECT_TRUE(
    rosidl_runtime_c__type_description__IndividualTypeDescription__are_equal(
      &type_description_1->referenced_type_descriptions.data[0], individual_desc_1));

  EXPECT_TRUE(
    rosidl_runtime_c__type_description__IndividualTypeDescription__are_equal(
      &type_description_1->referenced_type_descriptions.data[1], individual_desc_2));

  EXPECT_TRUE(
    rosidl_runtime_c__type_description__IndividualTypeDescription__are_equal(
      &type_description_1->referenced_type_descriptions.data[2], individual_desc_3));


  // REFERENCED TYPE DESCRIPTION APPEND
  // Naive recursive append
  EXPECT_EQ(type_description_2->referenced_type_descriptions.size, 0);
  EXPECT_EQ(type_description_2->referenced_type_descriptions.capacity, 0);
  ret = rosidl_runtime_c_type_description_utils_append_referenced_type_description(
    type_description_2, type_description_1, false);
  ASSERT_EQ(ret, RCUTILS_RET_OK);

  EXPECT_EQ(type_description_2->referenced_type_descriptions.size, 4);
  EXPECT_EQ(type_description_2->referenced_type_descriptions.capacity, 4);


  // Recursive append with coercion to valid
  EXPECT_EQ(
    rosidl_runtime_c_type_description_utils_append_field(
      &type_description_1->type_description, field_1),
    RCUTILS_RET_OK);

  EXPECT_EQ(
    rosidl_runtime_c_type_description_utils_append_field(
      &type_description_3->type_description, field_1),
    RCUTILS_RET_OK);

  // Deliberately invalid
  EXPECT_EQ(type_description_3->referenced_type_descriptions.size, 0);
  EXPECT_EQ(type_description_3->referenced_type_descriptions.capacity, 0);
  ASSERT_EQ(
    rosidl_runtime_c_type_description_utils_append_referenced_type_description(
      type_description_3, type_description_1, true),
    RCUTILS_RET_WARN);
  rcutils_reset_error();
  EXPECT_EQ(type_description_3->referenced_type_descriptions.size, 4);
  EXPECT_EQ(type_description_3->referenced_type_descriptions.capacity, 4);

  // With the append of the empty ref description, it should work now
  ret = rosidl_runtime_c_type_description_utils_append_referenced_individual_type_description(
    type_description_1, empty_individual_desc, true);
  ASSERT_EQ(ret, RCUTILS_RET_OK);

  ASSERT_EQ(
    rosidl_runtime_c_type_description_utils_append_referenced_type_description(
      type_description_3, type_description_1, true),
    RCUTILS_RET_OK);
  EXPECT_EQ(type_description_3->referenced_type_descriptions.size, 1);
  EXPECT_EQ(type_description_3->referenced_type_descriptions.capacity, 1);
  EXPECT_TRUE(
    rosidl_runtime_c__type_description__IndividualTypeDescription__are_equal(
      &type_description_3->referenced_type_descriptions.data[0], empty_individual_desc));


  // Get referenced type as its own full description
  // Naively (naive append of ref types)
  rosidl_runtime_c__type_description__TypeDescription * subset_desc = NULL;
  rosidl_runtime_c_type_description_utils_get_referenced_type_description_as_type_description(
    &type_description_1->referenced_type_descriptions,  // Refs: i1, i2, i3, empty
    individual_desc_1,                                  // Depends on ref: "empty"
    &subset_desc,
    false);                                             // No coercion to valid
  EXPECT_EQ(subset_desc->referenced_type_descriptions.size, 4);
  EXPECT_EQ(subset_desc->referenced_type_descriptions.capacity, 4);
  EXPECT_FALSE(
    rosidl_runtime_c__type_description__TypeDescription__are_equal(
      type_description_1, subset_desc));
  rosidl_runtime_c__type_description__TypeDescription__destroy(subset_desc);

  rosidl_runtime_c__type_description__TypeDescription * subset_desc_2 = NULL;
  rosidl_runtime_c_type_description_utils_get_referenced_type_description_as_type_description(
    &type_description_1->referenced_type_descriptions,  // Refs: i1, i2, i3, empty
    individual_desc_1,                                  // Depends on ref: "empty"
    &subset_desc_2,
    true);                                              // Coercion to valid
  EXPECT_EQ(
    rosidl_runtime_c_type_description_utils_type_description_is_valid(subset_desc_2),
    RCUTILS_RET_OK);
  EXPECT_EQ(subset_desc_2->referenced_type_descriptions.size, 1);
  EXPECT_EQ(subset_desc_2->referenced_type_descriptions.capacity, 1);
  EXPECT_FALSE(
    rosidl_runtime_c__type_description__TypeDescription__are_equal(
      type_description_1, subset_desc_2));
  EXPECT_TRUE(
    rosidl_runtime_c__type_description__IndividualTypeDescription__are_equal(
      &subset_desc_2->referenced_type_descriptions.data[0], empty_individual_desc));

  // Test type name string replacements
  ret = rosidl_runtime_c_type_description_utils_repl_all_type_description_type_names_in_place(
    subset_desc_2, "mpty", "ligibility"  // Expect eligibility
  );
  EXPECT_EQ(ret, RCUTILS_RET_OK);
  EXPECT_FALSE(
    rosidl_runtime_c__type_description__IndividualTypeDescription__are_equal(
      &subset_desc_2->referenced_type_descriptions.data[0], empty_individual_desc));
  rosidl_runtime_c__type_description__TypeDescription__destroy(subset_desc_2);
}


TEST_F(TestUtilsFixture, test_find)
{
  // Find Field
  ret = rosidl_runtime_c_type_description_utils_append_field(individual_desc_1, field_1);
  ASSERT_EQ(ret, RCUTILS_RET_OK);
  ret = rosidl_runtime_c_type_description_utils_append_field(individual_desc_1, field_2);
  ASSERT_EQ(ret, RCUTILS_RET_OK);
  ret = rosidl_runtime_c_type_description_utils_append_field(individual_desc_1, field_3);
  ASSERT_EQ(ret, RCUTILS_RET_OK);

  rosidl_runtime_c__type_description__Field * find_field = NULL;
  ret = rosidl_runtime_c_type_description_utils_find_field(
    &individual_desc_1->fields, "a",
    &find_field);
  EXPECT_EQ(ret, RCUTILS_RET_NOT_FOUND);
  rcutils_reset_error();
  EXPECT_FALSE(find_field);

  ret = rosidl_runtime_c_type_description_utils_find_field(
    &individual_desc_1->fields, "f3",
    &find_field);
  EXPECT_EQ(ret, RCUTILS_RET_OK);
  EXPECT_TRUE(rosidl_runtime_c__type_description__Field__are_equal(find_field, field_3));

  // Find Referenced Type Description
  ret = rosidl_runtime_c_type_description_utils_append_referenced_individual_type_description(
    type_description_1, individual_desc_1, false);
  ASSERT_EQ(ret, RCUTILS_RET_OK);
  ret = rosidl_runtime_c_type_description_utils_append_referenced_individual_type_description(
    type_description_1, individual_desc_2, false);
  ASSERT_EQ(ret, RCUTILS_RET_OK);
  ret = rosidl_runtime_c_type_description_utils_append_referenced_individual_type_description(
    type_description_1, individual_desc_3, true);
  ASSERT_EQ(ret, RCUTILS_RET_OK);

  rosidl_runtime_c__type_description__IndividualTypeDescription * find_desc = NULL;
  ret = rosidl_runtime_c_type_description_utils_find_referenced_type_description(
    &type_description_1->referenced_type_descriptions, "a", &find_desc);
  EXPECT_EQ(ret, RCUTILS_RET_NOT_FOUND);
  rcutils_reset_error();
  EXPECT_FALSE(find_desc);

  ret = rosidl_runtime_c_type_description_utils_find_referenced_type_description(
    &type_description_1->referenced_type_descriptions, "i3", &find_desc);
  EXPECT_EQ(ret, RCUTILS_RET_OK);
  EXPECT_TRUE(
    rosidl_runtime_c__type_description__IndividualTypeDescription__are_equal(
      find_desc, individual_desc_3));
}


TEST_F(TestUtilsFixture, test_maps)
{
  rcutils_hash_map_t * hash_map = NULL;
  rcutils_hash_map_t * ref_types_hash_map = NULL;
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  size_t map_length;

  // Field map when empty
  ret = rosidl_runtime_c_type_description_utils_get_field_map(
    individual_desc_1, &allocator,
    &hash_map);
  ASSERT_EQ(ret, RCUTILS_RET_OK);

  ASSERT_EQ(rcutils_hash_map_get_size(hash_map, &map_length), RCUTILS_RET_OK);
  ASSERT_EQ(map_length, 0);
  ASSERT_EQ(rcutils_hash_map_fini(hash_map), RCUTILS_RET_OK);
  allocator.deallocate(hash_map, allocator.state);
  hash_map = NULL;

  // Field map when populated
  ret = rosidl_runtime_c_type_description_utils_append_field(individual_desc_1, field_1);
  ASSERT_EQ(ret, RCUTILS_RET_OK);
  ret = rosidl_runtime_c_type_description_utils_append_field(individual_desc_1, field_2);
  ASSERT_EQ(ret, RCUTILS_RET_OK);
  ret = rosidl_runtime_c_type_description_utils_append_field(individual_desc_1, field_3);
  ASSERT_EQ(ret, RCUTILS_RET_OK);

  ret = rosidl_runtime_c_type_description_utils_get_field_map(
    individual_desc_1, &allocator,
    &hash_map);
  ASSERT_EQ(ret, RCUTILS_RET_OK);

  ASSERT_EQ(rcutils_hash_map_get_size(hash_map, &map_length), RCUTILS_RET_OK);
  ASSERT_EQ(map_length, 3);
  ASSERT_EQ(rcutils_hash_map_fini(hash_map), RCUTILS_RET_OK);
  allocator.deallocate(hash_map, allocator.state);
  hash_map = NULL;


  // Ref type map when empty
  ret = rosidl_runtime_c_type_description_utils_get_referenced_type_description_map(
    &type_description_1->referenced_type_descriptions, &allocator, &hash_map);
  ASSERT_EQ(ret, RCUTILS_RET_OK);

  ASSERT_EQ(rcutils_hash_map_get_size(hash_map, &map_length), RCUTILS_RET_OK);
  ASSERT_EQ(map_length, 0);
  ASSERT_EQ(rcutils_hash_map_fini(hash_map), RCUTILS_RET_OK);
  allocator.deallocate(hash_map, allocator.state);
  hash_map = NULL;

  // Ref type map when populated
  // Also we set up the next test block (for testing the necessary map)
  ret = rosidl_runtime_c_type_description_utils_append_referenced_individual_type_description(
    type_description_1, individual_desc_1, false);
  ASSERT_EQ(ret, RCUTILS_RET_OK);
  ret = rosidl_runtime_c_type_description_utils_append_referenced_individual_type_description(
    type_description_1, individual_desc_2, false);
  ASSERT_EQ(ret, RCUTILS_RET_OK);
  ret = rosidl_runtime_c_type_description_utils_append_referenced_individual_type_description(
    type_description_1, individual_desc_3, false);
  ASSERT_EQ(ret, RCUTILS_RET_OK);
  ret = rosidl_runtime_c_type_description_utils_append_referenced_individual_type_description(
    type_description_1, empty_individual_desc, true);
  ASSERT_EQ(ret, RCUTILS_RET_OK);

  ret = rosidl_runtime_c_type_description_utils_get_referenced_type_description_map(
    &type_description_1->referenced_type_descriptions, &allocator, &ref_types_hash_map);
  ASSERT_EQ(ret, RCUTILS_RET_OK);

  ASSERT_EQ(rcutils_hash_map_get_size(ref_types_hash_map, &map_length), RCUTILS_RET_OK);
  ASSERT_EQ(map_length, 4);


  // Necessary ref type map when empty
  ret = rosidl_runtime_c_type_description_utils_get_necessary_referenced_type_descriptions_map(
    &type_description_1->type_description,
    ref_types_hash_map,
    &allocator,
    &hash_map);
  ASSERT_EQ(ret, RCUTILS_RET_OK);

  ASSERT_EQ(rcutils_hash_map_get_size(hash_map, &map_length), RCUTILS_RET_OK);
  ASSERT_EQ(map_length, 0);
  ASSERT_EQ(rcutils_hash_map_fini(hash_map), RCUTILS_RET_OK);
  allocator.deallocate(hash_map, allocator.state);
  hash_map = NULL;

  // Necessary ref type map when populated
  ret = rosidl_runtime_c_type_description_utils_append_field(
    &type_description_1->type_description, field_1);
  ASSERT_EQ(ret, RCUTILS_RET_OK);

  // Expect failure since empty ref type not added yet
  ret = rosidl_runtime_c_type_description_utils_get_necessary_referenced_type_descriptions_map(
    &type_description_1->type_description,
    ref_types_hash_map,
    &allocator,
    &hash_map);
  ASSERT_EQ(ret, RCUTILS_RET_OK);

  ASSERT_EQ(rcutils_hash_map_get_size(hash_map, &map_length), RCUTILS_RET_OK);
  ASSERT_EQ(map_length, 1);
  ASSERT_EQ(rcutils_hash_map_fini(hash_map), RCUTILS_RET_OK);
  allocator.deallocate(hash_map, allocator.state);
  hash_map = NULL;


  // Ref type map to sequence
  rosidl_runtime_c__type_description__IndividualTypeDescription__Sequence * seq = NULL;
  /* *INDENT-OFF* */
  ret = rosidl_runtime_c_type_description_utils_copy_init_sequence_from_referenced_type_descriptions_map(  // NOLINT
    ref_types_hash_map, &seq, true);
  /* *INDENT-ON* */
  EXPECT_EQ(ret, RCUTILS_RET_OK);
  EXPECT_TRUE(
    rosidl_runtime_c__type_description__IndividualTypeDescription__Sequence__are_equal(
      &type_description_1->referenced_type_descriptions, seq));


  rosidl_runtime_c__type_description__IndividualTypeDescription__Sequence__destroy(seq);

  ASSERT_EQ(rcutils_hash_map_fini(ref_types_hash_map), RCUTILS_RET_OK);
  allocator.deallocate(ref_types_hash_map, allocator.state);
  ref_types_hash_map = NULL;
}

// TODO(methylDragon)
// TEST(Utils, test_validity)  <-- This is technically implicitly tested with appends with coercion
