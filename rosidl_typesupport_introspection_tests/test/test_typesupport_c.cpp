// Copyright 2021 Open Source Robotics Foundation, Inc.
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

#include <gtest/gtest.h>

#include <dlfcn.h>

#include <cstdio>
#include <string>
#include <unordered_map>
#include <vector>

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

#include "test_msgs/msg/arrays.h"
#include "test_msgs/msg/basic_types.h"
#include "test_msgs/msg/bounded_plain_sequences.h"
#include "test_msgs/msg/bounded_sequences.h"
#include "test_msgs/msg/builtins.h"
#include "test_msgs/msg/constants.h"
#include "test_msgs/msg/defaults.h"
#include "test_msgs/msg/empty.h"
#include "test_msgs/msg/multi_nested.h"
#include "test_msgs/msg/nested.h"
#include "test_msgs/msg/strings.h"
#include "test_msgs/msg/unbounded_sequences.h"
#include "test_msgs/msg/w_strings.h"

#include "test_msgs/srv/arrays.h"
#include "test_msgs/srv/basic_types.h"
#include "test_msgs/srv/empty.h"

#include "test_msgs/action/fibonacci.h"
#include "test_msgs/action/nested_message.h"

TEST(library_management, can_load_typesupport_library)
{
  auto typesupport_lib_name = "libtest_msgs__rosidl_typesupport_introspection_c.so";
  void * typesupport_lib = dlopen(typesupport_lib_name, RTLD_LAZY);
  char * error = dlerror();
  ASSERT_NE(typesupport_lib, nullptr) << "Error loading library '" << typesupport_lib_name <<
    "': " << error;
}

class TypeSupportLibraryTest: public ::testing::Test
{
protected:
  void SetUp() override {
    all_messages_member_counts.insert(
      interfaces_message_member_counts.begin(),
      interfaces_message_member_counts.end());
    for (const auto & service_type: services) {
      service_messages.push_back(service_type + "_Request");
      service_messages.push_back(service_type + "_Response");
      all_services.push_back(service_type);

      all_messages_member_counts[service_type + "_Request"] =
        services_message_member_counts[service_type + "_Request"];
      all_messages_member_counts[service_type + "_Response"] =
        services_message_member_counts[service_type + "_Response"];
    }
    for (const auto & action_type: actions) {
      action_messages.push_back(action_type + "_Feedback");
      action_messages.push_back(action_type + "_FeedbackMessage");
      action_messages.push_back(action_type + "_GetResult_Request");
      action_messages.push_back(action_type + "_GetResult_Response");
      action_messages.push_back(action_type + "_Goal");
      action_messages.push_back(action_type + "_Result");
      action_messages.push_back(action_type + "_SendGoal_Request");
      action_messages.push_back(action_type + "_SendGoal_Response");

      action_services.push_back(action_type + "_GetResult");
      action_services.push_back(action_type + "_SendGoal");

      all_messages_member_counts[action_type + "_Feedback"] =
        action_message_member_counts[action_type + "_Feedback"];
      all_messages_member_counts[action_type + "_FeedbackMessage"] =
        action_message_member_counts[action_type + "_FeedbackMessage"];
      all_messages_member_counts[action_type + "_GetResult_Request"] =
        action_message_member_counts[action_type + "_GetResult_Request"];
      all_messages_member_counts[action_type + "_GetResult_Response"] =
        action_message_member_counts[action_type + "_GetResult_Response"];
      all_messages_member_counts[action_type + "_Goal"] =
        action_message_member_counts[action_type + "_Goal"];
      all_messages_member_counts[action_type + "_Result"] =
        action_message_member_counts[action_type + "_Result"];
      all_messages_member_counts[action_type + "_SendGoal_Request"] =
        action_message_member_counts[action_type + "_SendGoal_Request"];
      all_messages_member_counts[action_type + "_SendGoal_Response"] =
        action_message_member_counts[action_type + "_SendGoal_Response"];
    }
    all_messages = interfaces;
    all_messages.insert(
      all_messages.end(),
      service_messages.begin(),
      service_messages.end());
    all_messages.insert(
      all_messages.end(),
      action_messages.begin(),
      action_messages.end());
    all_services.insert(
      all_services.end(),
      action_services.begin(),
      action_services.end());

    auto typesupport_lib_name = "lib" + ns + "__rosidl_typesupport_introspection_c.so";
    ts_lib = dlopen(typesupport_lib_name.c_str(), RTLD_LAZY);
    ASSERT_NE(ts_lib, nullptr) << "Error loading library '" << typesupport_lib_name.c_str() <<
      "': " << dlerror();
  }

  void TearDown() override {
    if (dlclose(ts_lib) != 0) {
      printf("Error closing library: %s\n", dlerror());
    }
  }

  void * ts_lib = nullptr;

  std::string ns = "test_msgs";
  const std::vector<std::string> interfaces = {
    "Arrays",
    "BasicTypes",
    "BoundedPlainSequences",
    "BoundedSequences",
    "Builtins",
    "Constants",
    "Defaults",
    "Empty",
    "MultiNested",
    "Nested",
    "Strings",
    "UnboundedSequences",
    "WStrings",
  };
  const std::unordered_map<std::string, uint32_t> interfaces_message_member_counts = {
    {"Arrays", 32},
    {"BasicTypes", 13},
    {"BoundedPlainSequences", 30},
    {"BoundedSequences", 32},
    {"Builtins", 2},
    {"Constants", 1},  // Not zero because a structure needs at least one member
    {"Defaults", 13},
    {"Empty", 1},  // Not zero because a structure needs at least one member
    {"MultiNested", 9},
    {"Nested", 1},
    {"Strings", 12},
    {"UnboundedSequences", 32},
    {"WStrings", 7},
  };
  const std::vector<std::string> services = {
    "Arrays",
    "BasicTypes",
    "Empty",
  };
  // TODO(gbiggs): This should be const
  std::unordered_map<std::string, uint32_t> services_message_member_counts = {
    {"Arrays_Request", 31},
    {"Arrays_Response", 31},
    {"BasicTypes_Request", 14},
    {"BasicTypes_Response", 14},
    {"Empty_Request", 1},  // Not zero because a structure needs at least one member
    {"Empty_Response", 1},  // Not zero because a structure needs at least one member
  };
  const std::vector<std::string> actions = {
    "Fibonacci",
    "NestedMessage",
  };
  // TODO(gbiggs): This should be const
  std::unordered_map<std::string, uint32_t> action_message_member_counts = {
    {"Fibonacci_Feedback", 1},
    {"Fibonacci_FeedbackMessage", 2},
    {"Fibonacci_GetResult_Request", 1},
    {"Fibonacci_GetResult_Response", 2},
    {"Fibonacci_Goal", 1},
    {"Fibonacci_Result", 1},
    {"Fibonacci_SendGoal_Request", 2},
    {"Fibonacci_SendGoal_Response", 2},
    {"NestedMessage_Feedback", 3},
    {"NestedMessage_FeedbackMessage", 2},
    {"NestedMessage_GetResult_Request", 1},
    {"NestedMessage_GetResult_Response", 2},
    {"NestedMessage_Goal", 3},
    {"NestedMessage_Result", 3},
    {"NestedMessage_SendGoal_Request", 2},
    {"NestedMessage_SendGoal_Response", 2},
  };

  std::vector<std::string> service_messages;
  std::vector<std::string> action_messages;
  std::vector<std::string> action_services;
  std::vector<std::string> all_messages;
  std::vector<std::string> all_services;

  std::unordered_map<std::string, uint32_t> all_messages_member_counts;
};

void get_symbol_from_library_with_checks(
  void * library,  // in
  const std::string & symbol_name,  // in
  void ** symbol)  // out if not nullptr
{
  // Clear any previous error
  dlerror();

  // Look up the symbol's address
  void * tmp_symbol = dlsym(library, symbol_name.c_str());
  EXPECT_NE(tmp_symbol, nullptr);
  if (symbol) {
    *symbol = tmp_symbol;
  }

  // The correct way to determine if the symbol was found is to confirm that there was no error
  // string set, i.e. dlerror() returns nullptr. This should be fatal to prevent a calling test
  // from continuing with a null symbol, but we can't use ASSERT_* in a function that doesn't
  // return void.
  char * error_str = dlerror();
  ASSERT_EQ(error_str, nullptr) << "Error finding symbol '" << symbol_name << "': " << error_str;
}

typedef const rosidl_message_type_support_t * (*get_message_ts_function)();
typedef const rosidl_service_type_support_t * (*get_service_ts_function)();

std::string make_message_typesupport_function_name(
  std::string ns,
  std::string type,
  std::string interface)
{
  return "rosidl_typesupport_introspection_c__get_message_type_support_handle__" +
    ns +
    "__" +
    type +
    "__" +
    interface;
}

std::string make_service_typesupport_function_name(
  std::string ns,
  std::string type,
  std::string interface)
{
  return "rosidl_typesupport_introspection_c__get_service_type_support_handle__" +
    ns +
    "__" +
    type +
    "__" +
    interface;
}

TEST_F(TypeSupportLibraryTest, library_contains_expected_interface_symbols)
{
  for(const auto & message: interfaces) {
    get_symbol_from_library_with_checks(
      ts_lib,
      make_message_typesupport_function_name(ns, "msg", message),
      nullptr);
  }
}

TEST_F(TypeSupportLibraryTest, library_contains_expected_service_symbols)
{
  for(const auto & message: service_messages) {
    get_symbol_from_library_with_checks(
      ts_lib,
      make_message_typesupport_function_name(ns, "srv", message),
      nullptr);
  }

  for(const auto & service: services) {
    get_symbol_from_library_with_checks(
      ts_lib,
      make_service_typesupport_function_name(ns, "srv", service),
      nullptr);
  }
}

TEST_F(TypeSupportLibraryTest, library_contains_expected_action_symbols)
{
  for(const auto & message: action_messages) {
    get_symbol_from_library_with_checks(
      ts_lib,
      make_message_typesupport_function_name(ns, "action", message),
      nullptr);
  }

  for(const auto & service: action_services) {
    get_symbol_from_library_with_checks(
      ts_lib,
      make_service_typesupport_function_name(ns, "action", service),
      nullptr);
  }
}

TEST_F(TypeSupportLibraryTest, can_get_introspection_structs_for_messages)
{
  for (const auto & message: interfaces) {
    void * get_ts_func_addr = nullptr;
    get_symbol_from_library_with_checks(
      ts_lib,
      make_message_typesupport_function_name(ns, "msg", message).c_str(),
      &get_ts_func_addr);
    if (!get_ts_func_addr) {
      // In case the symbol was not found, skip the rest of the tests for this symbol (allows us to
      // test the other symbols)
      continue;
    }

    get_message_ts_function get_ts_func =
      reinterpret_cast<get_message_ts_function>(get_ts_func_addr);
    const rosidl_message_type_support_t * ts = get_ts_func();

    ASSERT_NE(ts, nullptr);
    ASSERT_STREQ(ts->typesupport_identifier, "rosidl_typesupport_introspection_c");
    ASSERT_NE(ts->data, nullptr);
    // TODO(gbiggs): This should probably check it's the correct function
    ASSERT_NE(ts->func, nullptr);
  }
}

TEST_F(TypeSupportLibraryTest, can_get_introspection_structs_for_services)
{
  for (const auto & message: service_messages) {
    void * get_ts_func_addr = nullptr;
    get_symbol_from_library_with_checks(
      ts_lib,
      make_message_typesupport_function_name(ns, "srv", message).c_str(),
      &get_ts_func_addr);
    if (!get_ts_func_addr) {
      // In case the symbol was not found, skip the rest of the tests for this symbol (allows us to
      // test the other symbols)
      continue;
    }

    get_message_ts_function get_ts_func =
      reinterpret_cast<get_message_ts_function>(get_ts_func_addr);
    const rosidl_message_type_support_t * ts = get_ts_func();

    ASSERT_NE(ts, nullptr);
    ASSERT_STREQ(ts->typesupport_identifier, "rosidl_typesupport_introspection_c");
    ASSERT_NE(ts->data, nullptr);
    // TODO(gbiggs): This should probably check it's the correct function
    ASSERT_NE(ts->func, nullptr);
  }

  for (const auto & service: services) {
    void * get_ts_func_addr = nullptr;
    get_symbol_from_library_with_checks(
      ts_lib,
      make_service_typesupport_function_name(ns, "srv", service).c_str(),
      &get_ts_func_addr);
    if (!get_ts_func_addr) {
      // In case the symbol was not found, skip the rest of the tests for this symbol (allows us to
      // test the other symbols)
      continue;
    }

    get_service_ts_function get_ts_func =
      reinterpret_cast<get_service_ts_function>(get_ts_func_addr);
    const rosidl_service_type_support_t * ts = get_ts_func();

    ASSERT_NE(ts, nullptr);
    ASSERT_STREQ(ts->typesupport_identifier, "rosidl_typesupport_introspection_c");
    ASSERT_NE(ts->data, nullptr);
    // TODO(gbiggs): This should probably check it's the correct function
    ASSERT_NE(ts->func, nullptr);
  }
}

TEST_F(TypeSupportLibraryTest, can_get_introspection_structs_for_actions)
{
  for (const auto & message: action_messages) {
    void * get_ts_func_addr = nullptr;
    get_symbol_from_library_with_checks(
      ts_lib,
      make_message_typesupport_function_name(ns, "action", message).c_str(),
      &get_ts_func_addr);
    if (!get_ts_func_addr) {
      // In case the symbol was not found, skip the rest of the tests for this symbol (allows us to
      // test the other symbols)
      continue;
    }

    get_message_ts_function get_ts_func =
      reinterpret_cast<get_message_ts_function>(get_ts_func_addr);
    const rosidl_message_type_support_t * ts = get_ts_func();

    ASSERT_NE(ts, nullptr);
    ASSERT_STREQ(ts->typesupport_identifier, "rosidl_typesupport_introspection_c");
    ASSERT_NE(ts->data, nullptr);
    // TODO(gbiggs): This should probably check it's the correct function
    ASSERT_NE(ts->func, nullptr);
  }

  for (const auto & service: action_services) {
    void * get_ts_func_addr = nullptr;
    get_symbol_from_library_with_checks(
      ts_lib,
      make_service_typesupport_function_name(ns, "action", service).c_str(),
      &get_ts_func_addr);
    if (!get_ts_func_addr) {
      // In case the symbol was not found, skip the rest of the tests for this symbol (allows us to
      // test the other symbols)
      continue;
    }

    get_service_ts_function get_ts_func =
      reinterpret_cast<get_service_ts_function>(get_ts_func_addr);
    const rosidl_service_type_support_t * ts = get_ts_func();

    ASSERT_NE(ts, nullptr);
    ASSERT_STREQ(ts->typesupport_identifier, "rosidl_typesupport_introspection_c");
    ASSERT_NE(ts->data, nullptr);
    // TODO(gbiggs): This should probably check it's the correct function
    ASSERT_NE(ts->func, nullptr);
  }
}

class TypeSupportMembersStructuresTest: public TypeSupportLibraryTest
{
protected:
  void SetUp() override
  {
    TypeSupportLibraryTest::SetUp();

    for (const auto & message: interfaces) {
      void * get_ts_func_addr = nullptr;
      get_symbol_from_library_with_checks(
        ts_lib,
        make_message_typesupport_function_name(ns, "msg", message).c_str(),
        &get_ts_func_addr);
      ASSERT_NE(get_ts_func_addr, nullptr);

      get_message_ts_function get_ts_func =
        reinterpret_cast<get_message_ts_function>(get_ts_func_addr);
      const rosidl_message_type_support_t * ts = get_ts_func();
      const rosidl_typesupport_introspection_c__MessageMembers * members =
        reinterpret_cast<const rosidl_typesupport_introspection_c__MessageMembers *>(ts->data);
      interface_message_members_structs[message] = members;
      all_message_members_structs[message] = members;
    }

    for (const auto & message: service_messages) {
      void * get_ts_func_addr = nullptr;
      get_symbol_from_library_with_checks(
        ts_lib,
        make_message_typesupport_function_name(ns, "srv", message).c_str(),
        &get_ts_func_addr);
      ASSERT_NE(get_ts_func_addr, nullptr);

      get_message_ts_function get_ts_func =
        reinterpret_cast<get_message_ts_function>(get_ts_func_addr);
      const rosidl_message_type_support_t * ts = get_ts_func();
      const rosidl_typesupport_introspection_c__MessageMembers * members =
        reinterpret_cast<const rosidl_typesupport_introspection_c__MessageMembers *>(ts->data);
      service_message_members_structs[message] = members;
      all_message_members_structs[message] = members;
    }

    for (const auto & service: services) {
      void * get_ts_func_addr = nullptr;
      get_symbol_from_library_with_checks(
        ts_lib,
        make_service_typesupport_function_name(ns, "srv", service).c_str(),
        &get_ts_func_addr);
      ASSERT_NE(get_ts_func_addr, nullptr);

      get_service_ts_function get_ts_func =
        reinterpret_cast<get_service_ts_function>(get_ts_func_addr);
      const rosidl_service_type_support_t * ts = get_ts_func();
      const rosidl_typesupport_introspection_c__ServiceMembers * members =
        reinterpret_cast<const rosidl_typesupport_introspection_c__ServiceMembers *>(ts->data);
      service_service_members_structs[service] = members;
      all_service_members_structs[service] = members;
    }

    for (const auto & message: action_messages) {
      void * get_ts_func_addr = nullptr;
      get_symbol_from_library_with_checks(
        ts_lib,
        make_message_typesupport_function_name(ns, "action", message).c_str(),
        &get_ts_func_addr);
      ASSERT_NE(get_ts_func_addr, nullptr);

      get_message_ts_function get_ts_func =
        reinterpret_cast<get_message_ts_function>(get_ts_func_addr);
      const rosidl_message_type_support_t * ts = get_ts_func();
      const rosidl_typesupport_introspection_c__MessageMembers * members =
        reinterpret_cast<const rosidl_typesupport_introspection_c__MessageMembers *>(ts->data);
      action_message_members_structs[message] = members;
      all_message_members_structs[message] = members;
    }

    for (const auto & service: action_services) {
      void * get_ts_func_addr = nullptr;
      get_symbol_from_library_with_checks(
        ts_lib,
        make_service_typesupport_function_name(ns, "action", service).c_str(),
        &get_ts_func_addr);
      ASSERT_NE(get_ts_func_addr, nullptr);

      get_service_ts_function get_ts_func =
        reinterpret_cast<get_service_ts_function>(get_ts_func_addr);
      const rosidl_service_type_support_t * ts = get_ts_func();
      const rosidl_typesupport_introspection_c__ServiceMembers * members =
        reinterpret_cast<const rosidl_typesupport_introspection_c__ServiceMembers *>(ts->data);
      action_service_members_structs[service] = members;
      all_service_members_structs[service] = members;
    }

    populate_constructors();
  }

  std::unordered_map<std::string, const rosidl_typesupport_introspection_c__MessageMembers *>
    interface_message_members_structs;
  std::unordered_map<std::string, const rosidl_typesupport_introspection_c__MessageMembers *>
    service_message_members_structs;
  std::unordered_map<std::string, const rosidl_typesupport_introspection_c__ServiceMembers *>
    service_service_members_structs;
  std::unordered_map<std::string, const rosidl_typesupport_introspection_c__MessageMembers *>
    action_message_members_structs;
  std::unordered_map<std::string, const rosidl_typesupport_introspection_c__ServiceMembers *>
    action_service_members_structs;
  std::unordered_map<std::string, const rosidl_typesupport_introspection_c__MessageMembers *>
    all_message_members_structs;
  std::unordered_map<std::string, const rosidl_typesupport_introspection_c__ServiceMembers *>
    all_service_members_structs;

  std::unordered_map<std::string, std::pair<void *, void *>> interfaces_message_constructors = {
    {"Arrays", {nullptr, nullptr}},
    {"BasicTypes", {nullptr, nullptr}},
    {"BoundedPlainSequences", {nullptr, nullptr}},
    {"BoundedSequences", {nullptr, nullptr}},
    {"Builtins", {nullptr, nullptr}},
    {"Constants", {nullptr, nullptr}},
    {"Defaults", {nullptr, nullptr}},
    {"Empty", {nullptr, nullptr}},
    {"MultiNested", {nullptr, nullptr}},
    {"Nested", {nullptr, nullptr}},
    {"Strings", {nullptr, nullptr}},
    {"UnboundedSequences", {nullptr, nullptr}},
    {"WStrings", {nullptr, nullptr}},
  };
  std::unordered_map<std::string, std::pair<void *, void *>> service_message_constructors = {
    {"Arrays_Request", {nullptr, nullptr}},
    {"Arrays_Response", {nullptr, nullptr}},
    {"BasicTypes_Request", {nullptr, nullptr}},
    {"BasicTypes_Response", {nullptr, nullptr}},
    {"Empty_Request", {nullptr, nullptr}},
    {"Empty_Response", {nullptr, nullptr}},
  };
  std::unordered_map<std::string, std::pair<void *, void *>> action_message_constructors = {
    {"Fibonacci_Feedback", {nullptr, nullptr}},
    {"Fibonacci_FeedbackMessage", {nullptr, nullptr}},
    {"Fibonacci_GetResult_Request", {nullptr, nullptr}},
    {"Fibonacci_GetResult_Response", {nullptr, nullptr}},
    {"Fibonacci_Goal", {nullptr, nullptr}},
    {"Fibonacci_Result", {nullptr, nullptr}},
    {"Fibonacci_SendGoal_Request", {nullptr, nullptr}},
    {"Fibonacci_SendGoal_Response", {nullptr, nullptr}},
    {"NestedMessage_Feedback", {nullptr, nullptr}},
    {"NestedMessage_FeedbackMessage", {nullptr, nullptr}},
    {"NestedMessage_GetResult_Request", {nullptr, nullptr}},
    {"NestedMessage_GetResult_Response", {nullptr, nullptr}},
    {"NestedMessage_Goal", {nullptr, nullptr}},
    {"NestedMessage_Result", {nullptr, nullptr}},
    {"NestedMessage_SendGoal_Request", {nullptr, nullptr}},
    {"NestedMessage_SendGoal_Response", {nullptr, nullptr}},
  };
  std::unordered_map<std::string, std::pair<void *, void *>> all_message_constructors;

  void * types_lib = nullptr;

  std::string make_message_create_function_name(
    std::string ns,
    std::string type,
    std::string interface)
  {
    return ns + "__" + type + "__" + interface + "__destroy";
  }

  std::string make_message_destroy_function_name(
    std::string ns,
    std::string type,
    std::string interface)
  {
    return ns + "__" + type + "__" + interface + "__destroy";
  }

  void populate_constructors()
  {
    auto typesupport_lib_name = "lib" + ns + "__rosidl_generator_c.so";
    types_lib = dlopen(typesupport_lib_name.c_str(), RTLD_LAZY);
    ASSERT_NE(types_lib, nullptr) << "Error loading library '" << typesupport_lib_name.c_str() <<
      "': " << dlerror();

    for (const auto & [interface, _]: interfaces_message_constructors) {
      void * create_symbol = nullptr;
      get_symbol_from_library_with_checks(
        types_lib,
        make_message_create_function_name("test_msgs", "msg", interface),
        &create_symbol);

      void * destroy_symbol = nullptr;
      get_symbol_from_library_with_checks(
        types_lib,
        make_message_destroy_function_name("test_msgs", "msg", interface),
        &destroy_symbol);

      interfaces_message_constructors[interface] = {create_symbol, destroy_symbol};
      all_message_constructors[interface] = {create_symbol, destroy_symbol};
    }

    for (const auto & [interface, _]: service_message_constructors) {
      void * create_symbol = nullptr;
      get_symbol_from_library_with_checks(
        types_lib,
        make_message_create_function_name("test_msgs", "srv", interface),
        &create_symbol);

      void * destroy_symbol = nullptr;
      get_symbol_from_library_with_checks(
        types_lib,
        make_message_destroy_function_name("test_msgs", "srv", interface),
        &destroy_symbol);

      service_message_constructors[interface] = {create_symbol, destroy_symbol};
      all_message_constructors[interface] = {create_symbol, destroy_symbol};
    }

    for (const auto & [interface, _]: action_message_constructors) {
      void * create_symbol = nullptr;
      get_symbol_from_library_with_checks(
        types_lib,
        make_message_create_function_name("test_msgs", "action", interface),
        &create_symbol);

      void * destroy_symbol = nullptr;
      get_symbol_from_library_with_checks(
        types_lib,
        make_message_destroy_function_name("test_msgs", "action", interface),
        &destroy_symbol);

      action_message_constructors[interface] = {create_symbol, destroy_symbol};
      all_message_constructors[interface] = {create_symbol, destroy_symbol};
    }
  }
};

TEST_F(TypeSupportMembersStructuresTest, NamespaceIsFilledAndCorrect)
{
  for (const auto &[message, members_struct]: interface_message_members_structs) {
    ASSERT_STREQ(members_struct->message_namespace_, "test_msgs__msg") << message;
  }

  for (const auto &[message, members_struct]: service_message_members_structs) {
    ASSERT_STREQ(members_struct->message_namespace_, "test_msgs__srv") << message;
  }

  for (const auto &[service, members_struct]: service_service_members_structs) {
    ASSERT_STREQ(members_struct->service_namespace_, "test_msgs__srv") << service;
  }

  for (const auto &[message, members_struct]: action_message_members_structs) {
    ASSERT_STREQ(members_struct->message_namespace_, "test_msgs__action") << message;
  }

  for (const auto &[service, members_struct]: action_service_members_structs) {
    ASSERT_STREQ(members_struct->service_namespace_, "test_msgs__action") << service;
  }
}

TEST_F(TypeSupportMembersStructuresTest, NameIsFilledAndCorrect)
{
  for (const auto &[message, members_struct]: all_message_members_structs) {
    ASSERT_STREQ(members_struct->message_name_, message.c_str()) << message;
  }

  for (const auto &[service, members_struct]: all_service_members_structs) {
    ASSERT_STREQ(members_struct->service_name_, service.c_str()) << service;
  }
}

TEST_F(TypeSupportMembersStructuresTest, MemberCountIsFilledAndCorrect)
{
  for (const auto &[message, members_struct]: all_message_members_structs) {
    ASSERT_EQ(members_struct->member_count_, all_messages_member_counts[message]) << message;
  }
}

TEST_F(TypeSupportMembersStructuresTest, SizeOfIsFilledAndCorrect)
{
  std::unordered_map<std::string, size_t> message_struct_sizes = {
    {"Arrays", sizeof(test_msgs__msg__Arrays)},
    {"BasicTypes", sizeof(test_msgs__msg__BasicTypes)},
    {"BoundedPlainSequences", sizeof(test_msgs__msg__BoundedPlainSequences)},
    {"BoundedSequences", sizeof(test_msgs__msg__BoundedSequences)},
    {"Builtins", sizeof(test_msgs__msg__Builtins)},
    {"Constants", sizeof(test_msgs__msg__Constants)},
    {"Defaults", sizeof(test_msgs__msg__Defaults)},
    {"Empty", sizeof(test_msgs__msg__Empty)},
    {"MultiNested", sizeof(test_msgs__msg__MultiNested)},
    {"Nested", sizeof(test_msgs__msg__Nested)},
    {"Strings", sizeof(test_msgs__msg__Strings)},
    {"UnboundedSequences", sizeof(test_msgs__msg__UnboundedSequences)},
    {"WStrings", sizeof(test_msgs__msg__WStrings)},

    {"Arrays_Request", sizeof(test_msgs__srv__Arrays_Request)},
    {"Arrays_Response", sizeof(test_msgs__srv__Arrays_Response)},
    {"BasicTypes_Request", sizeof(test_msgs__srv__BasicTypes_Request)},
    {"BasicTypes_Response", sizeof(test_msgs__srv__BasicTypes_Response)},
    {"Empty_Request", sizeof(test_msgs__srv__Empty_Request)},
    {"Empty_Response", sizeof(test_msgs__srv__Empty_Response)},

    {"Fibonacci_Feedback", sizeof(test_msgs__action__Fibonacci_Feedback)},
    {"Fibonacci_FeedbackMessage", sizeof(test_msgs__action__Fibonacci_FeedbackMessage)},
    {"Fibonacci_GetResult_Request", sizeof(test_msgs__action__Fibonacci_GetResult_Request)},
    {"Fibonacci_GetResult_Response", sizeof(test_msgs__action__Fibonacci_GetResult_Response)},
    {"Fibonacci_Goal", sizeof(test_msgs__action__Fibonacci_Goal)},
    {"Fibonacci_Result", sizeof(test_msgs__action__Fibonacci_Result)},
    {"Fibonacci_SendGoal_Request", sizeof(test_msgs__action__Fibonacci_SendGoal_Request)},
    {"Fibonacci_SendGoal_Response", sizeof(test_msgs__action__Fibonacci_SendGoal_Response)},
    {"NestedMessage_Feedback", sizeof(test_msgs__action__NestedMessage_Feedback)},
    {"NestedMessage_FeedbackMessage", sizeof(test_msgs__action__NestedMessage_FeedbackMessage)},
    {"NestedMessage_GetResult_Request",
      sizeof(test_msgs__action__NestedMessage_GetResult_Request)},
    {"NestedMessage_GetResult_Response",
      sizeof(test_msgs__action__NestedMessage_GetResult_Response)},
    {"NestedMessage_Goal", sizeof(test_msgs__action__NestedMessage_Goal)},
    {"NestedMessage_Result", sizeof(test_msgs__action__NestedMessage_Result)},
    {"NestedMessage_SendGoal_Request", sizeof(test_msgs__action__NestedMessage_SendGoal_Request)},
    {"NestedMessage_SendGoal_Response",
      sizeof(test_msgs__action__NestedMessage_SendGoal_Response)},
  };
  for (const auto &[message, members_struct]: all_message_members_structs) {
    ASSERT_EQ(members_struct->size_of_, message_struct_sizes[message]) << message;
  }
}

TEST_F(TypeSupportMembersStructuresTest, MembersArrayIsNotNull)
{
  for (const auto &[message, members_struct]: all_message_members_structs) {
    ASSERT_NE(members_struct->members_, nullptr) << message;
    // TODO(gbiggs): Also check the size of the array?
  }
}

#define MESSAGE_STRUCT(ns, prefix, name) \
  ns ## __ ## prefix ## __ ## name

#define MESSAGE_STRUCT_CREATE_FUNCTION(ns, prefix, name) \
  ns ## __ ## prefix ## __ ## name ## __create

#define MESSAGE_STRUCT_DESTROY_FUNCTION(ns, prefix, name) \
  ns ## __ ## prefix ## __ ## name ## __destroy

TEST_F(TypeSupportMembersStructuresTest, CanCallInitFunction)
{
  for (const auto &[message, members_struct]: all_message_members_structs) {
    ASSERT_NE(members_struct->init_function, nullptr) << message;

    // These messages can be init'd but there is no way to check that it succeeded, so they are not
    // tested:
    // BasicTypes
    // Builtins
    // Constants
    // Empty
    // MultiNested
    // Nested
    // Arrays_Request
    // Arrays_Response
    // BasicTypes_Request
    // BasicTypes_Response
    // Empty_Request
    // Empty_Response
    // Fibonacci_Feedback
    // Fibonacci_FeedbackMessage
    // Fibonacci_GetResult_Request
    // Fibonacci_GetResult_Response
    // Fibonacci_Goal
    // Fibonacci_Result
    // Fibonacci_SendGoal_Request
    // Fibonacci_SendGoal_Response
    // NestedMessage_Feedback
    // NestedMessage_FeedbackMessage
    // NestedMessage_GetResult_Request
    // NestedMessage_GetResult_Response
    // NestedMessage_Goal
    // NestedMessage_Result
    // NestedMessage_SendGoal_Request
    // NestedMessage_SendGoal_Response

    // These messages have default values, so the success of the init call can be checked.
    // Arrays
    // BoundedPlainSequences
    // BoundedSequences
    // Defaults
    // Strings
    // UnboundedSequences
    // WStrings

    // TODO(gbiggs): This is a bit brute-force. It could probably be done more intelligently to
    // test exactly the functionality that can be generated by the code generator, without testing
    // so much.

    if (message == "Arrays") {
      MESSAGE_STRUCT(test_msgs, msg, Arrays) * message_struct =
        MESSAGE_STRUCT_CREATE_FUNCTION(test_msgs, msg, Arrays)();

      members_struct->init_function(message_struct, ROSIDL_RUNTIME_C_MSG_INIT_ALL);
      ASSERT_EQ(message_struct->bool_values_default[0], false);
      ASSERT_EQ(message_struct->int8_values_default[2], -128);
      ASSERT_EQ(message_struct->uint64_values_default[2], 18446744073709551615u);
      ASSERT_STREQ(message_struct->string_values_default[1].data, "max value");
    } else if (message == "BoundedPlainSequences") {
      MESSAGE_STRUCT(test_msgs, msg, BoundedPlainSequences) * message_struct =
        MESSAGE_STRUCT_CREATE_FUNCTION(test_msgs, msg, BoundedPlainSequences)();

      members_struct->init_function(message_struct, ROSIDL_RUNTIME_C_MSG_INIT_ALL);
      ASSERT_EQ(message_struct->bool_values_default.data[0], false);
      ASSERT_EQ(message_struct->int8_values_default.data[2], -128);
      ASSERT_EQ(message_struct->uint64_values_default.data[2], 18446744073709551615u);
    } else if (message == "BoundedSequences") {
      MESSAGE_STRUCT(test_msgs, msg, BoundedSequences) * message_struct =
        MESSAGE_STRUCT_CREATE_FUNCTION(test_msgs, msg, BoundedSequences)();

      members_struct->init_function(message_struct, ROSIDL_RUNTIME_C_MSG_INIT_ALL);
      ASSERT_EQ(message_struct->bool_values_default.data[0], false);
      ASSERT_EQ(message_struct->int8_values_default.data[2], -128);
      ASSERT_EQ(message_struct->uint64_values_default.data[2], 18446744073709551615u);
      ASSERT_STREQ(message_struct->string_values_default.data[1].data, "max value");
    } else if (message == "Defaults") {
      MESSAGE_STRUCT(test_msgs, msg, Defaults) * message_struct =
        MESSAGE_STRUCT_CREATE_FUNCTION(test_msgs, msg, Defaults)();

      members_struct->init_function(message_struct, ROSIDL_RUNTIME_C_MSG_INIT_ALL);
      ASSERT_EQ(message_struct->bool_value, true);
      ASSERT_EQ(message_struct->byte_value, 50);
      ASSERT_EQ(message_struct->char_value, 100);
      ASSERT_EQ(message_struct->float32_value, 1.125);
      ASSERT_EQ(message_struct->float64_value, 1.125);
      ASSERT_EQ(message_struct->int8_value, -50);
      ASSERT_EQ(message_struct->uint8_value, 200u);
      ASSERT_EQ(message_struct->int16_value, -1000);
      ASSERT_EQ(message_struct->uint16_value, 2000u);
      ASSERT_EQ(message_struct->int32_value, -30000);
      ASSERT_EQ(message_struct->uint32_value, 60000u);
      ASSERT_EQ(message_struct->int64_value, -40000000);
      ASSERT_EQ(message_struct->uint64_value, 50000000u);
    } else if (message == "Strings") {
      MESSAGE_STRUCT(test_msgs, msg, Strings) * message_struct =
        MESSAGE_STRUCT_CREATE_FUNCTION(test_msgs, msg, Strings)();

      members_struct->init_function(message_struct, ROSIDL_RUNTIME_C_MSG_INIT_ALL);
      ASSERT_STREQ(message_struct->string_value_default1.data, "Hello world!");
      ASSERT_STREQ(message_struct->string_value_default2.data, "Hello'world!");
      ASSERT_STREQ(message_struct->string_value_default3.data, "Hello\"world!");
      ASSERT_STREQ(message_struct->string_value_default4.data, "Hello'world!");
      ASSERT_STREQ(message_struct->string_value_default5.data, "Hello\"world!");
      ASSERT_STREQ(message_struct->bounded_string_value_default1.data, "Hello world!");
      ASSERT_STREQ(message_struct->bounded_string_value_default2.data, "Hello'world!");
      ASSERT_STREQ(message_struct->bounded_string_value_default3.data, "Hello\"world!");
      ASSERT_STREQ(message_struct->bounded_string_value_default4.data, "Hello'world!");
      ASSERT_STREQ(message_struct->bounded_string_value_default5.data, "Hello\"world!");
    } else if (message == "UnboundedSequences") {
      MESSAGE_STRUCT(test_msgs, msg, UnboundedSequences) * message_struct =
        MESSAGE_STRUCT_CREATE_FUNCTION(test_msgs, msg, UnboundedSequences)();

      members_struct->init_function(message_struct, ROSIDL_RUNTIME_C_MSG_INIT_ALL);
      ASSERT_EQ(message_struct->bool_values_default.data[0], false);
      ASSERT_EQ(message_struct->int8_values_default.data[2], -128);
      ASSERT_EQ(message_struct->uint64_values_default.data[2], 18446744073709551615u);
      ASSERT_STREQ(message_struct->string_values_default.data[1].data, "max value");
    } else if (message == "WStrings") {
      MESSAGE_STRUCT(test_msgs, msg, WStrings) * message_struct =
        MESSAGE_STRUCT_CREATE_FUNCTION(test_msgs, msg, WStrings)();

      members_struct->init_function(message_struct, ROSIDL_RUNTIME_C_MSG_INIT_ALL);
      // TODO(gbiggs): Figure out how to make this work with ASSERT_THAT(x, ElementsAre(...))
      ASSERT_EQ(message_struct->wstring_value_default1.data[0], u'H');
      ASSERT_EQ(message_struct->wstring_value_default1.data[1], u'e');
      ASSERT_EQ(message_struct->wstring_value_default1.data[2], u'l');
      ASSERT_EQ(message_struct->wstring_value_default1.data[3], u'l');
      ASSERT_EQ(message_struct->wstring_value_default1.data[4], u'o');
      ASSERT_EQ(message_struct->wstring_value_default1.data[5], u' ');
      ASSERT_EQ(message_struct->wstring_value_default1.data[6], u'w');
      ASSERT_EQ(message_struct->wstring_value_default1.data[7], u'o');
      ASSERT_EQ(message_struct->wstring_value_default1.data[8], u'r');
      ASSERT_EQ(message_struct->wstring_value_default1.data[9], u'l');
      ASSERT_EQ(message_struct->wstring_value_default1.data[10], u'd');
      ASSERT_EQ(message_struct->wstring_value_default1.data[11], u'!');
      ASSERT_EQ(message_struct->wstring_value_default2.data[0], u'H');
      ASSERT_EQ(message_struct->wstring_value_default2.data[1], u'e');
      ASSERT_EQ(message_struct->wstring_value_default2.data[2], u'l');
      ASSERT_EQ(message_struct->wstring_value_default2.data[3], u'l');
      ASSERT_EQ(message_struct->wstring_value_default2.data[4], u'ö');
      ASSERT_EQ(message_struct->wstring_value_default2.data[5], u' ');
      ASSERT_EQ(message_struct->wstring_value_default2.data[6], u'w');
      ASSERT_EQ(message_struct->wstring_value_default2.data[7], u'ö');
      ASSERT_EQ(message_struct->wstring_value_default2.data[8], u'r');
      ASSERT_EQ(message_struct->wstring_value_default2.data[9], u'l');
      ASSERT_EQ(message_struct->wstring_value_default2.data[10], u'd');
      ASSERT_EQ(message_struct->wstring_value_default2.data[11], u'!');
      ASSERT_EQ(message_struct->wstring_value_default3.data[0], u'ハ');
      ASSERT_EQ(message_struct->wstring_value_default3.data[1], u'ロ');
      ASSERT_EQ(message_struct->wstring_value_default3.data[2], u'ー');
      ASSERT_EQ(message_struct->wstring_value_default3.data[3], u'ワ');
      ASSERT_EQ(message_struct->wstring_value_default3.data[4], u'ー');
      ASSERT_EQ(message_struct->wstring_value_default3.data[5], u'ル');
      ASSERT_EQ(message_struct->wstring_value_default3.data[6], u'ド');
    }
  }
}

TEST_F(TypeSupportMembersStructuresTest, CanCallFiniFunction)
{
  for (const auto &[message, members_struct]: all_message_members_structs) {
    ASSERT_NE(members_struct->fini_function, nullptr) << message;

    if (message == "Arrays") {
      MESSAGE_STRUCT(test_msgs, msg, Arrays) * message_struct =
        MESSAGE_STRUCT_CREATE_FUNCTION(test_msgs, msg, Arrays)();

      members_struct->init_function(message_struct, ROSIDL_RUNTIME_C_MSG_INIT_ALL);
      members_struct->fini_function(message_struct);
      // This block of asserts tests an array of strings (so each individual member of the array
      // needs to be fini'd)
      ASSERT_EQ(message_struct->string_values[0].data, nullptr);
      ASSERT_EQ(message_struct->string_values[0].size, 0u);
      ASSERT_EQ(message_struct->string_values[0].capacity, 0u);
      ASSERT_EQ(message_struct->string_values[1].data, nullptr);
      ASSERT_EQ(message_struct->string_values[1].size, 0u);
      ASSERT_EQ(message_struct->string_values[1].capacity, 0u);
      ASSERT_EQ(message_struct->string_values[2].data, nullptr);
      ASSERT_EQ(message_struct->string_values[2].size, 0u);
      ASSERT_EQ(message_struct->string_values[2].capacity, 0u);
      ASSERT_EQ(message_struct->string_values_default[0].data, nullptr);
      ASSERT_EQ(message_struct->string_values_default[0].size, 0u);
      ASSERT_EQ(message_struct->string_values_default[0].capacity, 0u);
      ASSERT_EQ(message_struct->string_values_default[1].data, nullptr);
      ASSERT_EQ(message_struct->string_values_default[1].size, 0u);
      ASSERT_EQ(message_struct->string_values_default[1].capacity, 0u);
      ASSERT_EQ(message_struct->string_values_default[2].data, nullptr);
      ASSERT_EQ(message_struct->string_values_default[2].size, 0u);
      ASSERT_EQ(message_struct->string_values_default[2].capacity, 0u);
    } else if (message == "BoundedPlainSequences") {
      MESSAGE_STRUCT(test_msgs, msg, BoundedPlainSequences) * message_struct =
        MESSAGE_STRUCT_CREATE_FUNCTION(test_msgs, msg, BoundedPlainSequences)();

      members_struct->init_function(message_struct, ROSIDL_RUNTIME_C_MSG_INIT_ALL);
      members_struct->fini_function(message_struct);
      // This block of asserts neatly encapsulates the sequence variant of every basic type
      ASSERT_EQ(message_struct->bool_values.data, nullptr);
      ASSERT_EQ(message_struct->bool_values.size, 0u);
      ASSERT_EQ(message_struct->bool_values.capacity, 0u);
      ASSERT_EQ(message_struct->byte_values.data, nullptr);
      ASSERT_EQ(message_struct->byte_values.size, 0u);
      ASSERT_EQ(message_struct->byte_values.capacity, 0u);
      ASSERT_EQ(message_struct->char_values.data, nullptr);
      ASSERT_EQ(message_struct->char_values.size, 0u);
      ASSERT_EQ(message_struct->char_values.capacity, 0u);
      ASSERT_EQ(message_struct->float32_values.data, nullptr);
      ASSERT_EQ(message_struct->float32_values.size, 0u);
      ASSERT_EQ(message_struct->float32_values.capacity, 0u);
      ASSERT_EQ(message_struct->float64_values.data, nullptr);
      ASSERT_EQ(message_struct->float64_values.size, 0u);
      ASSERT_EQ(message_struct->float64_values.capacity, 0u);
      ASSERT_EQ(message_struct->int8_values.data, nullptr);
      ASSERT_EQ(message_struct->int8_values.size, 0u);
      ASSERT_EQ(message_struct->int8_values.capacity, 0u);
      ASSERT_EQ(message_struct->uint8_values.data, nullptr);
      ASSERT_EQ(message_struct->uint8_values.size, 0u);
      ASSERT_EQ(message_struct->uint8_values.capacity, 0u);
      ASSERT_EQ(message_struct->int16_values.data, nullptr);
      ASSERT_EQ(message_struct->int16_values.size, 0u);
      ASSERT_EQ(message_struct->int16_values.capacity, 0u);
      ASSERT_EQ(message_struct->uint16_values.data, nullptr);
      ASSERT_EQ(message_struct->uint16_values.size, 0u);
      ASSERT_EQ(message_struct->uint16_values.capacity, 0u);
      ASSERT_EQ(message_struct->int32_values.data, nullptr);
      ASSERT_EQ(message_struct->int32_values.size, 0u);
      ASSERT_EQ(message_struct->int32_values.capacity, 0u);
      ASSERT_EQ(message_struct->uint32_values.data, nullptr);
      ASSERT_EQ(message_struct->uint32_values.size, 0u);
      ASSERT_EQ(message_struct->uint32_values.capacity, 0u);
      ASSERT_EQ(message_struct->int64_values.data, nullptr);
      ASSERT_EQ(message_struct->int64_values.size, 0u);
      ASSERT_EQ(message_struct->int64_values.capacity, 0u);
      ASSERT_EQ(message_struct->uint64_values.data, nullptr);
      ASSERT_EQ(message_struct->uint64_values.size, 0u);
      ASSERT_EQ(message_struct->uint64_values.capacity, 0u);
      // This block of asserts captures the sequence variant of a nested type
      ASSERT_EQ(message_struct->basic_types_values.data, nullptr);
      ASSERT_EQ(message_struct->basic_types_values.size, 0u);
      ASSERT_EQ(message_struct->basic_types_values.capacity, 0u);
      // The remainder of the members just duplicate the above types, so don't test them explicitly
    } else if (message == "BoundedSequences") {
      // Apart from the addition of string sequences, this is the same as BoundedPlainSequences. So
      // just test the string sequence members
      MESSAGE_STRUCT(test_msgs, msg, BoundedSequences) * message_struct =
        MESSAGE_STRUCT_CREATE_FUNCTION(test_msgs, msg, BoundedSequences)();

      members_struct->init_function(message_struct, ROSIDL_RUNTIME_C_MSG_INIT_ALL);
      members_struct->fini_function(message_struct);

      ASSERT_EQ(message_struct->string_values.data, nullptr);
      ASSERT_EQ(message_struct->string_values.size, 0u);
      ASSERT_EQ(message_struct->string_values.capacity, 0u);
      ASSERT_EQ(message_struct->string_values_default.data, nullptr);
      ASSERT_EQ(message_struct->string_values_default.size, 0u);
      ASSERT_EQ(message_struct->string_values_default.capacity, 0u);
    } else if (message == "Strings") {
      // This tests strings (singular, not sequence)
      MESSAGE_STRUCT(test_msgs, msg, Strings) * message_struct =
        MESSAGE_STRUCT_CREATE_FUNCTION(test_msgs, msg, Strings)();

      members_struct->init_function(message_struct, ROSIDL_RUNTIME_C_MSG_INIT_ALL);
      members_struct->fini_function(message_struct);

      ASSERT_EQ(message_struct->string_value.data, nullptr);
      ASSERT_EQ(message_struct->string_value.size, 0u);
      ASSERT_EQ(message_struct->string_value.capacity, 0u);
      ASSERT_EQ(message_struct->string_value_default1.data, nullptr);
      ASSERT_EQ(message_struct->string_value_default1.size, 0u);
      ASSERT_EQ(message_struct->string_value_default1.capacity, 0u);
      ASSERT_EQ(message_struct->bounded_string_value.data, nullptr);
      ASSERT_EQ(message_struct->bounded_string_value.size, 0u);
      ASSERT_EQ(message_struct->bounded_string_value.capacity, 0u);
      ASSERT_EQ(message_struct->bounded_string_value_default1.data, nullptr);
      ASSERT_EQ(message_struct->bounded_string_value_default1.size, 0u);
      ASSERT_EQ(message_struct->bounded_string_value_default1.capacity, 0u);
    } else if (message == "WStrings") {
      // This tests wide strings: singular, array and sequence
      MESSAGE_STRUCT(test_msgs, msg, WStrings) * message_struct =
        MESSAGE_STRUCT_CREATE_FUNCTION(test_msgs, msg, WStrings)();

      members_struct->init_function(message_struct, ROSIDL_RUNTIME_C_MSG_INIT_ALL);
      members_struct->fini_function(message_struct);

      ASSERT_EQ(message_struct->wstring_value.data, nullptr);
      ASSERT_EQ(message_struct->wstring_value.size, 0u);
      ASSERT_EQ(message_struct->wstring_value.capacity, 0u);
      ASSERT_EQ(message_struct->wstring_value_default1.data, nullptr);
      ASSERT_EQ(message_struct->wstring_value_default1.size, 0u);
      ASSERT_EQ(message_struct->wstring_value_default1.capacity, 0u);
      // These fields are commented out in the message definition
      //ASSERT_EQ(message_struct->bounded_wstring_value.data, nullptr);
      //ASSERT_EQ(message_struct->bounded_wstring_value.size, 0u);
      //ASSERT_EQ(message_struct->bounded_wstring_value.capacity, 0u);
      //ASSERT_EQ(message_struct->bounded_wstring_value_default1.data, nullptr);
      //ASSERT_EQ(message_struct->bounded_wstring_value_default1.size, 0u);
      //ASSERT_EQ(message_struct->bounded_wstring_value_default1.capacity, 0u);
      ASSERT_EQ(message_struct->array_of_wstrings[0].data, nullptr);
      ASSERT_EQ(message_struct->array_of_wstrings[0].size, 0u);
      ASSERT_EQ(message_struct->array_of_wstrings[0].capacity, 0u);
      ASSERT_EQ(message_struct->array_of_wstrings[1].data, nullptr);
      ASSERT_EQ(message_struct->array_of_wstrings[1].size, 0u);
      ASSERT_EQ(message_struct->array_of_wstrings[1].capacity, 0u);
      ASSERT_EQ(message_struct->array_of_wstrings[2].data, nullptr);
      ASSERT_EQ(message_struct->array_of_wstrings[2].size, 0u);
      ASSERT_EQ(message_struct->array_of_wstrings[2].capacity, 0u);
      ASSERT_EQ(message_struct->bounded_sequence_of_wstrings.data, nullptr);
      ASSERT_EQ(message_struct->bounded_sequence_of_wstrings.size, 0u);
      ASSERT_EQ(message_struct->bounded_sequence_of_wstrings.capacity, 0u);
      ASSERT_EQ(message_struct->unbounded_sequence_of_wstrings.data, nullptr);
      ASSERT_EQ(message_struct->unbounded_sequence_of_wstrings.size, 0u);
      ASSERT_EQ(message_struct->unbounded_sequence_of_wstrings.capacity, 0u);
    }
  }
}

TEST_F(TypeSupportMembersStructuresTest, ServiceMessageMembersPointersAreCorrect)
{
  for (const auto &[service, members_struct]: all_service_members_structs) {
    // Confirm that the pointers do actually point to the same structures we got via the
    // message-getter function for each service message.
    ASSERT_EQ(
      members_struct->request_members_,
      all_message_members_structs[service + "_Request"]);
    ASSERT_EQ(
      members_struct->response_members_,
      all_message_members_structs[service + "_Response"]);
  }
}

class TypeSupportMemberStructuresTest: public TypeSupportMembersStructuresTest
{
protected:
  void SetUp() override
  {
    TypeSupportMembersStructuresTest::SetUp();
  }
};

TEST_F(TypeSupportMemberStructuresTest, BasicTypeMembersAreCorrect)
{
  const rosidl_typesupport_introspection_c__MessageMember * members =
    all_message_members_structs["BasicTypes"]->members_;

  EXPECT_STREQ(members[0].name_, "bool_value");
  EXPECT_EQ(members[0].type_id_, rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN);
  EXPECT_EQ(members[0].string_upper_bound_, 0u);
  EXPECT_EQ(members[0].members_, nullptr);
  EXPECT_EQ(members[0].is_array_, false);
  EXPECT_EQ(members[0].array_size_, 0u);
  EXPECT_EQ(members[0].is_upper_bound_, false);
  EXPECT_EQ(members[0].offset_, 0u);
  EXPECT_EQ(members[0].default_value_, nullptr);
  EXPECT_EQ(members[0].size_function, nullptr);
  EXPECT_EQ(members[0].get_const_function, nullptr);
  EXPECT_EQ(members[0].get_function, nullptr);
  EXPECT_EQ(members[0].resize_function, nullptr);

  EXPECT_STREQ(members[1].name_, "byte_value");
  EXPECT_EQ(members[1].type_id_, rosidl_typesupport_introspection_c__ROS_TYPE_OCTET);
  EXPECT_EQ(members[1].string_upper_bound_, 0u);
  EXPECT_EQ(members[1].members_, nullptr);
  EXPECT_EQ(members[1].is_array_, false);
  EXPECT_EQ(members[1].array_size_, 0u);
  EXPECT_EQ(members[1].is_upper_bound_, false);
  EXPECT_EQ(members[1].offset_, offsetof(MESSAGE_STRUCT(test_msgs, msg, BasicTypes), byte_value));
  EXPECT_EQ(members[1].default_value_, nullptr);
  EXPECT_EQ(members[1].size_function, nullptr);
  EXPECT_EQ(members[1].get_const_function, nullptr);
  EXPECT_EQ(members[1].get_function, nullptr);
  EXPECT_EQ(members[1].resize_function, nullptr);

  EXPECT_STREQ(members[2].name_, "char_value");
  EXPECT_EQ(members[2].type_id_, rosidl_typesupport_introspection_c__ROS_TYPE_UINT8);
  EXPECT_EQ(members[2].string_upper_bound_, 0u);
  EXPECT_EQ(members[2].members_, nullptr);
  EXPECT_EQ(members[2].is_array_, false);
  EXPECT_EQ(members[2].array_size_, 0u);
  EXPECT_EQ(members[2].is_upper_bound_, false);
  EXPECT_EQ(members[2].offset_, offsetof(MESSAGE_STRUCT(test_msgs, msg, BasicTypes), char_value));
  EXPECT_EQ(members[2].default_value_, nullptr);
  EXPECT_EQ(members[2].size_function, nullptr);
  EXPECT_EQ(members[2].get_const_function, nullptr);
  EXPECT_EQ(members[2].get_function, nullptr);
  EXPECT_EQ(members[2].resize_function, nullptr);

  EXPECT_STREQ(members[3].name_, "float32_value");
  EXPECT_EQ(members[3].type_id_, rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT32);
  EXPECT_EQ(members[3].string_upper_bound_, 0u);
  EXPECT_EQ(members[3].members_, nullptr);
  EXPECT_EQ(members[3].is_array_, false);
  EXPECT_EQ(members[3].array_size_, 0u);
  EXPECT_EQ(members[3].is_upper_bound_, false);
  EXPECT_EQ(
    members[3].offset_,
    offsetof(MESSAGE_STRUCT(test_msgs, msg, BasicTypes), float32_value));
  EXPECT_EQ(members[3].default_value_, nullptr);
  EXPECT_EQ(members[3].size_function, nullptr);
  EXPECT_EQ(members[3].get_const_function, nullptr);
  EXPECT_EQ(members[3].get_function, nullptr);
  EXPECT_EQ(members[3].resize_function, nullptr);

  EXPECT_STREQ(members[4].name_, "float64_value");
  EXPECT_EQ(members[4].type_id_, rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT64);
  EXPECT_EQ(members[4].string_upper_bound_, 0u);
  EXPECT_EQ(members[4].members_, nullptr);
  EXPECT_EQ(members[4].is_array_, false);
  EXPECT_EQ(members[4].array_size_, 0u);
  EXPECT_EQ(members[4].is_upper_bound_, false);
  EXPECT_EQ(
    members[4].offset_,
    offsetof(MESSAGE_STRUCT(test_msgs, msg, BasicTypes), float64_value));
  EXPECT_EQ(members[4].default_value_, nullptr);
  EXPECT_EQ(members[4].size_function, nullptr);
  EXPECT_EQ(members[4].get_const_function, nullptr);
  EXPECT_EQ(members[4].get_function, nullptr);
  EXPECT_EQ(members[4].resize_function, nullptr);

  EXPECT_STREQ(members[5].name_, "int8_value");
  EXPECT_EQ(members[5].type_id_, rosidl_typesupport_introspection_c__ROS_TYPE_INT8);
  EXPECT_EQ(members[5].string_upper_bound_, 0u);
  EXPECT_EQ(members[5].members_, nullptr);
  EXPECT_EQ(members[5].is_array_, false);
  EXPECT_EQ(members[5].array_size_, 0u);
  EXPECT_EQ(members[5].is_upper_bound_, false);
  EXPECT_EQ(members[5].offset_, offsetof(MESSAGE_STRUCT(test_msgs, msg, BasicTypes), int8_value));
  EXPECT_EQ(members[5].default_value_, nullptr);
  EXPECT_EQ(members[5].size_function, nullptr);
  EXPECT_EQ(members[5].get_const_function, nullptr);
  EXPECT_EQ(members[5].get_function, nullptr);
  EXPECT_EQ(members[5].resize_function, nullptr);

  EXPECT_STREQ(members[6].name_, "uint8_value");
  EXPECT_EQ(members[6].type_id_, rosidl_typesupport_introspection_c__ROS_TYPE_UINT8);
  EXPECT_EQ(members[6].string_upper_bound_, 0u);
  EXPECT_EQ(members[6].members_, nullptr);
  EXPECT_EQ(members[6].is_array_, false);
  EXPECT_EQ(members[6].array_size_, 0u);
  EXPECT_EQ(members[6].is_upper_bound_, false);
  EXPECT_EQ(members[6].offset_, offsetof(MESSAGE_STRUCT(test_msgs, msg, BasicTypes), uint8_value));
  EXPECT_EQ(members[6].default_value_, nullptr);
  EXPECT_EQ(members[6].size_function, nullptr);
  EXPECT_EQ(members[6].get_const_function, nullptr);
  EXPECT_EQ(members[6].get_function, nullptr);
  EXPECT_EQ(members[6].resize_function, nullptr);

  EXPECT_STREQ(members[7].name_, "int16_value");
  EXPECT_EQ(members[7].type_id_, rosidl_typesupport_introspection_c__ROS_TYPE_INT16);
  EXPECT_EQ(members[7].string_upper_bound_, 0u);
  EXPECT_EQ(members[7].members_, nullptr);
  EXPECT_EQ(members[7].is_array_, false);
  EXPECT_EQ(members[7].array_size_, 0u);
  EXPECT_EQ(members[7].is_upper_bound_, false);
  EXPECT_EQ(members[7].offset_, offsetof(MESSAGE_STRUCT(test_msgs, msg, BasicTypes), int16_value));
  EXPECT_EQ(members[7].default_value_, nullptr);
  EXPECT_EQ(members[7].size_function, nullptr);
  EXPECT_EQ(members[7].get_const_function, nullptr);
  EXPECT_EQ(members[7].get_function, nullptr);
  EXPECT_EQ(members[7].resize_function, nullptr);

  EXPECT_STREQ(members[8].name_, "uint16_value");
  EXPECT_EQ(members[8].type_id_, rosidl_typesupport_introspection_c__ROS_TYPE_UINT16);
  EXPECT_EQ(members[8].string_upper_bound_, 0u);
  EXPECT_EQ(members[8].members_, nullptr);
  EXPECT_EQ(members[8].is_array_, false);
  EXPECT_EQ(members[8].array_size_, 0u);
  EXPECT_EQ(members[8].is_upper_bound_, false);
  EXPECT_EQ(
    members[8].offset_,
    offsetof(MESSAGE_STRUCT(test_msgs, msg, BasicTypes), uint16_value));
  EXPECT_EQ(members[8].default_value_, nullptr);
  EXPECT_EQ(members[8].size_function, nullptr);
  EXPECT_EQ(members[8].get_const_function, nullptr);
  EXPECT_EQ(members[8].get_function, nullptr);
  EXPECT_EQ(members[8].resize_function, nullptr);

  EXPECT_STREQ(members[9].name_, "int32_value");
  EXPECT_EQ(members[9].type_id_, rosidl_typesupport_introspection_c__ROS_TYPE_INT32);
  EXPECT_EQ(members[9].string_upper_bound_, 0u);
  EXPECT_EQ(members[9].members_, nullptr);
  EXPECT_EQ(members[9].is_array_, false);
  EXPECT_EQ(members[9].array_size_, 0u);
  EXPECT_EQ(members[9].is_upper_bound_, false);
  EXPECT_EQ(members[9].offset_, offsetof(MESSAGE_STRUCT(test_msgs, msg, BasicTypes), int32_value));
  EXPECT_EQ(members[9].default_value_, nullptr);
  EXPECT_EQ(members[9].size_function, nullptr);
  EXPECT_EQ(members[9].get_const_function, nullptr);
  EXPECT_EQ(members[9].get_function, nullptr);
  EXPECT_EQ(members[9].resize_function, nullptr);

  EXPECT_STREQ(members[10].name_, "uint32_value");
  EXPECT_EQ(members[10].type_id_, rosidl_typesupport_introspection_c__ROS_TYPE_UINT32);
  EXPECT_EQ(members[10].string_upper_bound_, 0u);
  EXPECT_EQ(members[10].members_, nullptr);
  EXPECT_EQ(members[10].is_array_, false);
  EXPECT_EQ(members[10].array_size_, 0u);
  EXPECT_EQ(members[10].is_upper_bound_, false);
  EXPECT_EQ(
    members[10].offset_,
    offsetof(MESSAGE_STRUCT(test_msgs, msg, BasicTypes), uint32_value));
  EXPECT_EQ(members[10].default_value_, nullptr);
  EXPECT_EQ(members[10].size_function, nullptr);
  EXPECT_EQ(members[10].get_const_function, nullptr);
  EXPECT_EQ(members[10].get_function, nullptr);
  EXPECT_EQ(members[10].resize_function, nullptr);

  EXPECT_STREQ(members[11].name_, "int64_value");
  EXPECT_EQ(members[11].type_id_, rosidl_typesupport_introspection_c__ROS_TYPE_INT64);
  EXPECT_EQ(members[11].string_upper_bound_, 0u);
  EXPECT_EQ(members[11].members_, nullptr);
  EXPECT_EQ(members[11].is_array_, false);
  EXPECT_EQ(members[11].array_size_, 0u);
  EXPECT_EQ(members[11].is_upper_bound_, false);
  EXPECT_EQ(
    members[11].offset_,
    offsetof(MESSAGE_STRUCT(test_msgs, msg, BasicTypes), int64_value));
  EXPECT_EQ(members[11].default_value_, nullptr);
  EXPECT_EQ(members[11].size_function, nullptr);
  EXPECT_EQ(members[11].get_const_function, nullptr);
  EXPECT_EQ(members[11].get_function, nullptr);
  EXPECT_EQ(members[11].resize_function, nullptr);

  EXPECT_STREQ(members[12].name_, "uint64_value");
  EXPECT_EQ(members[12].type_id_, rosidl_typesupport_introspection_c__ROS_TYPE_UINT64);
  EXPECT_EQ(members[12].string_upper_bound_, 0u);
  EXPECT_EQ(members[12].members_, nullptr);
  EXPECT_EQ(members[12].is_array_, false);
  EXPECT_EQ(members[12].array_size_, 0u);
  EXPECT_EQ(members[12].is_upper_bound_, false);
  EXPECT_EQ(
    members[12].offset_,
    offsetof(MESSAGE_STRUCT(test_msgs, msg, BasicTypes), uint64_value));
  EXPECT_EQ(members[12].default_value_, nullptr);
  EXPECT_EQ(members[12].size_function, nullptr);
  EXPECT_EQ(members[12].get_const_function, nullptr);
  EXPECT_EQ(members[12].get_function, nullptr);
  EXPECT_EQ(members[12].resize_function, nullptr);
}

TEST_F(TypeSupportMemberStructuresTest, ArrayMembersAreCorrect)
{
  const rosidl_typesupport_introspection_c__MessageMember * members =
    all_message_members_structs["Arrays"]->members_;
  MESSAGE_STRUCT(test_msgs, msg, Arrays) * message_struct =
    MESSAGE_STRUCT_CREATE_FUNCTION(test_msgs, msg, Arrays)();
  all_message_members_structs["Arrays"]->init_function(
    message_struct,
    ROSIDL_RUNTIME_C_MSG_INIT_ALL);

  EXPECT_STREQ(members[0].name_, "bool_values");
  EXPECT_EQ(members[0].type_id_, rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN);
  EXPECT_EQ(members[0].string_upper_bound_, 0u);
  EXPECT_EQ(members[0].members_, nullptr);
  EXPECT_EQ(members[0].is_array_, true);
  EXPECT_EQ(members[0].array_size_, 3u);
  EXPECT_EQ(members[0].is_upper_bound_, false);
  EXPECT_EQ(members[0].offset_, 0u);
  EXPECT_EQ(members[0].default_value_, nullptr);
  EXPECT_EQ(members[0].size_function, nullptr);
  EXPECT_EQ(members[0].get_const_function, nullptr);
  EXPECT_EQ(members[0].get_function, nullptr);
  EXPECT_EQ(members[0].resize_function, nullptr);

  EXPECT_STREQ(members[1].name_, "byte_values");
  EXPECT_EQ(members[1].type_id_, rosidl_typesupport_introspection_c__ROS_TYPE_OCTET);
  EXPECT_EQ(members[1].string_upper_bound_, 0u);
  EXPECT_EQ(members[1].members_, nullptr);
  EXPECT_EQ(members[1].is_array_, true);
  EXPECT_EQ(members[1].array_size_, 3u);
  EXPECT_EQ(members[1].is_upper_bound_, false);
  EXPECT_EQ(members[1].offset_, offsetof(MESSAGE_STRUCT(test_msgs, msg, Arrays), byte_values));
  EXPECT_EQ(members[1].default_value_, nullptr);
  EXPECT_EQ(members[1].size_function, nullptr);
  EXPECT_EQ(members[1].get_const_function, nullptr);
  EXPECT_EQ(members[1].get_function, nullptr);
  EXPECT_EQ(members[1].resize_function, nullptr);

  EXPECT_STREQ(members[2].name_, "char_values");
  EXPECT_EQ(members[2].type_id_, rosidl_typesupport_introspection_c__ROS_TYPE_UINT8);
  EXPECT_EQ(members[2].string_upper_bound_, 0u);
  EXPECT_EQ(members[2].members_, nullptr);
  EXPECT_EQ(members[2].is_array_, true);
  EXPECT_EQ(members[2].array_size_, 3u);
  EXPECT_EQ(members[2].is_upper_bound_, false);
  EXPECT_EQ(members[2].offset_, offsetof(MESSAGE_STRUCT(test_msgs, msg, Arrays), char_values));
  EXPECT_EQ(members[2].default_value_, nullptr);
  EXPECT_EQ(members[2].size_function, nullptr);
  EXPECT_EQ(members[2].get_const_function, nullptr);
  EXPECT_EQ(members[2].get_function, nullptr);
  EXPECT_EQ(members[2].resize_function, nullptr);

  EXPECT_STREQ(members[3].name_, "float32_values");
  EXPECT_EQ(members[3].type_id_, rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT32);
  EXPECT_EQ(members[3].string_upper_bound_, 0u);
  EXPECT_EQ(members[3].members_, nullptr);
  EXPECT_EQ(members[3].is_array_, true);
  EXPECT_EQ(members[3].array_size_, 3u);
  EXPECT_EQ(members[3].is_upper_bound_, false);
  EXPECT_EQ(
    members[3].offset_,
    offsetof(MESSAGE_STRUCT(test_msgs, msg, Arrays), float32_values));
  EXPECT_EQ(members[3].default_value_, nullptr);
  EXPECT_EQ(members[3].size_function, nullptr);
  EXPECT_EQ(members[3].get_const_function, nullptr);
  EXPECT_EQ(members[3].get_function, nullptr);
  EXPECT_EQ(members[3].resize_function, nullptr);

  EXPECT_STREQ(members[4].name_, "float64_values");
  EXPECT_EQ(members[4].type_id_, rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT64);
  EXPECT_EQ(members[4].string_upper_bound_, 0u);
  EXPECT_EQ(members[4].members_, nullptr);
  EXPECT_EQ(members[4].is_array_, true);
  EXPECT_EQ(members[4].array_size_, 3u);
  EXPECT_EQ(members[4].is_upper_bound_, false);
  EXPECT_EQ(
    members[4].offset_,
    offsetof(MESSAGE_STRUCT(test_msgs, msg, Arrays), float64_values));
  EXPECT_EQ(members[4].default_value_, nullptr);
  EXPECT_EQ(members[4].size_function, nullptr);
  EXPECT_EQ(members[4].get_const_function, nullptr);
  EXPECT_EQ(members[4].get_function, nullptr);
  EXPECT_EQ(members[4].resize_function, nullptr);

  EXPECT_STREQ(members[5].name_, "int8_values");
  EXPECT_EQ(members[5].type_id_, rosidl_typesupport_introspection_c__ROS_TYPE_INT8);
  EXPECT_EQ(members[5].string_upper_bound_, 0u);
  EXPECT_EQ(members[5].members_, nullptr);
  EXPECT_EQ(members[5].is_array_, true);
  EXPECT_EQ(members[5].array_size_, 3u);
  EXPECT_EQ(members[5].is_upper_bound_, false);
  EXPECT_EQ(members[5].offset_, offsetof(MESSAGE_STRUCT(test_msgs, msg, Arrays), int8_values));
  EXPECT_EQ(members[5].default_value_, nullptr);
  EXPECT_EQ(members[5].size_function, nullptr);
  EXPECT_EQ(members[5].get_const_function, nullptr);
  EXPECT_EQ(members[5].get_function, nullptr);
  EXPECT_EQ(members[5].resize_function, nullptr);

  EXPECT_STREQ(members[6].name_, "uint8_values");
  EXPECT_EQ(members[6].type_id_, rosidl_typesupport_introspection_c__ROS_TYPE_UINT8);
  EXPECT_EQ(members[6].string_upper_bound_, 0u);
  EXPECT_EQ(members[6].members_, nullptr);
  EXPECT_EQ(members[6].is_array_, true);
  EXPECT_EQ(members[6].array_size_, 3u);
  EXPECT_EQ(members[6].is_upper_bound_, false);
  EXPECT_EQ(members[6].offset_, offsetof(MESSAGE_STRUCT(test_msgs, msg, Arrays), uint8_values));
  EXPECT_EQ(members[6].default_value_, nullptr);
  EXPECT_EQ(members[6].size_function, nullptr);
  EXPECT_EQ(members[6].get_const_function, nullptr);
  EXPECT_EQ(members[6].get_function, nullptr);
  EXPECT_EQ(members[6].resize_function, nullptr);

  EXPECT_STREQ(members[7].name_, "int16_values");
  EXPECT_EQ(members[7].type_id_, rosidl_typesupport_introspection_c__ROS_TYPE_INT16);
  EXPECT_EQ(members[7].string_upper_bound_, 0u);
  EXPECT_EQ(members[7].members_, nullptr);
  EXPECT_EQ(members[7].is_array_, true);
  EXPECT_EQ(members[7].array_size_, 3u);
  EXPECT_EQ(members[7].is_upper_bound_, false);
  EXPECT_EQ(members[7].offset_, offsetof(MESSAGE_STRUCT(test_msgs, msg, Arrays), int16_values));
  EXPECT_EQ(members[7].default_value_, nullptr);
  EXPECT_EQ(members[7].size_function, nullptr);
  EXPECT_EQ(members[7].get_const_function, nullptr);
  EXPECT_EQ(members[7].get_function, nullptr);
  EXPECT_EQ(members[7].resize_function, nullptr);

  EXPECT_STREQ(members[8].name_, "uint16_values");
  EXPECT_EQ(members[8].type_id_, rosidl_typesupport_introspection_c__ROS_TYPE_UINT16);
  EXPECT_EQ(members[8].string_upper_bound_, 0u);
  EXPECT_EQ(members[8].members_, nullptr);
  EXPECT_EQ(members[8].is_array_, true);
  EXPECT_EQ(members[8].array_size_, 3u);
  EXPECT_EQ(members[8].is_upper_bound_, false);
  EXPECT_EQ(
    members[8].offset_,
    offsetof(MESSAGE_STRUCT(test_msgs, msg, Arrays), uint16_values));
  EXPECT_EQ(members[8].default_value_, nullptr);
  EXPECT_EQ(members[8].size_function, nullptr);
  EXPECT_EQ(members[8].get_const_function, nullptr);
  EXPECT_EQ(members[8].get_function, nullptr);
  EXPECT_EQ(members[8].resize_function, nullptr);

  EXPECT_STREQ(members[9].name_, "int32_values");
  EXPECT_EQ(members[9].type_id_, rosidl_typesupport_introspection_c__ROS_TYPE_INT32);
  EXPECT_EQ(members[9].string_upper_bound_, 0u);
  EXPECT_EQ(members[9].members_, nullptr);
  EXPECT_EQ(members[9].is_array_, true);
  EXPECT_EQ(members[9].array_size_, 3u);
  EXPECT_EQ(members[9].is_upper_bound_, false);
  EXPECT_EQ(members[9].offset_, offsetof(MESSAGE_STRUCT(test_msgs, msg, Arrays), int32_values));
  EXPECT_EQ(members[9].default_value_, nullptr);
  EXPECT_EQ(members[9].size_function, nullptr);
  EXPECT_EQ(members[9].get_const_function, nullptr);
  EXPECT_EQ(members[9].get_function, nullptr);
  EXPECT_EQ(members[9].resize_function, nullptr);

  EXPECT_STREQ(members[10].name_, "uint32_values");
  EXPECT_EQ(members[10].type_id_, rosidl_typesupport_introspection_c__ROS_TYPE_UINT32);
  EXPECT_EQ(members[10].string_upper_bound_, 0u);
  EXPECT_EQ(members[10].members_, nullptr);
  EXPECT_EQ(members[10].is_array_, true);
  EXPECT_EQ(members[10].array_size_, 3u);
  EXPECT_EQ(members[10].is_upper_bound_, false);
  EXPECT_EQ(
    members[10].offset_,
    offsetof(MESSAGE_STRUCT(test_msgs, msg, Arrays), uint32_values));
  EXPECT_EQ(members[10].default_value_, nullptr);
  EXPECT_EQ(members[10].size_function, nullptr);
  EXPECT_EQ(members[10].get_const_function, nullptr);
  EXPECT_EQ(members[10].get_function, nullptr);
  EXPECT_EQ(members[10].resize_function, nullptr);

  EXPECT_STREQ(members[11].name_, "int64_values");
  EXPECT_EQ(members[11].type_id_, rosidl_typesupport_introspection_c__ROS_TYPE_INT64);
  EXPECT_EQ(members[11].string_upper_bound_, 0u);
  EXPECT_EQ(members[11].members_, nullptr);
  EXPECT_EQ(members[11].is_array_, true);
  EXPECT_EQ(members[11].array_size_, 3u);
  EXPECT_EQ(members[11].is_upper_bound_, false);
  EXPECT_EQ(
    members[11].offset_,
    offsetof(MESSAGE_STRUCT(test_msgs, msg, Arrays), int64_values));
  EXPECT_EQ(members[11].default_value_, nullptr);
  EXPECT_EQ(members[11].size_function, nullptr);
  EXPECT_EQ(members[11].get_const_function, nullptr);
  EXPECT_EQ(members[11].get_function, nullptr);
  EXPECT_EQ(members[11].resize_function, nullptr);

  EXPECT_STREQ(members[12].name_, "uint64_values");
  EXPECT_EQ(members[12].type_id_, rosidl_typesupport_introspection_c__ROS_TYPE_UINT64);
  EXPECT_EQ(members[12].string_upper_bound_, 0u);
  EXPECT_EQ(members[12].members_, nullptr);
  EXPECT_EQ(members[12].is_array_, true);
  EXPECT_EQ(members[12].array_size_, 3u);
  EXPECT_EQ(members[12].is_upper_bound_, false);
  EXPECT_EQ(
    members[12].offset_,
    offsetof(MESSAGE_STRUCT(test_msgs, msg, Arrays), uint64_values));
  EXPECT_EQ(members[12].default_value_, nullptr);
  EXPECT_EQ(members[12].size_function, nullptr);
  EXPECT_EQ(members[12].get_const_function, nullptr);
  EXPECT_EQ(members[12].get_function, nullptr);
  EXPECT_EQ(members[12].resize_function, nullptr);

  EXPECT_STREQ(members[13].name_, "string_values");
  EXPECT_EQ(members[13].type_id_, rosidl_typesupport_introspection_c__ROS_TYPE_STRING);
  EXPECT_EQ(members[13].string_upper_bound_, 0u);
  EXPECT_EQ(members[13].members_, nullptr);
  EXPECT_EQ(members[13].is_array_, true);
  EXPECT_EQ(members[13].array_size_, 3u);
  EXPECT_EQ(members[13].is_upper_bound_, false);
  EXPECT_EQ(
    members[13].offset_,
    offsetof(MESSAGE_STRUCT(test_msgs, msg, Arrays), string_values));
  EXPECT_EQ(members[13].default_value_, nullptr);
  EXPECT_EQ(members[13].size_function, nullptr);
  EXPECT_EQ(members[13].get_const_function, nullptr);
  EXPECT_EQ(members[13].get_function, nullptr);
  EXPECT_EQ(members[13].resize_function, nullptr);

  // This tests an array of a nested type
  EXPECT_STREQ(members[16].name_, "defaults_values");
  EXPECT_EQ(members[16].type_id_, rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE);
  EXPECT_EQ(members[16].string_upper_bound_, 0u);
  EXPECT_NE(members[16].members_, nullptr);
  EXPECT_EQ(members[16].is_array_, true);
  EXPECT_EQ(members[16].array_size_, 3u);
  EXPECT_EQ(members[16].is_upper_bound_, false);
  EXPECT_EQ(
    members[16].offset_,
    offsetof(MESSAGE_STRUCT(test_msgs, msg, Arrays), defaults_values));
  EXPECT_EQ(members[16].default_value_, nullptr);
  EXPECT_NE(members[16].size_function, nullptr);
  // The size of this Arrays member is 3
  EXPECT_EQ(members[16].size_function(&message_struct->basic_types_values), 3u);
  // This Arrays member has default values, so we can check if retrieving them via the generated
  // functions works correctly
  EXPECT_NE(members[16].get_const_function, nullptr);
  EXPECT_EQ(
    reinterpret_cast<const MESSAGE_STRUCT(test_msgs, msg, Defaults) *>(
      members[16].get_const_function(&message_struct->defaults_values, 2))->float32_value,
    1.125);
  EXPECT_NE(members[16].get_function, nullptr);
  EXPECT_EQ(
    reinterpret_cast<MESSAGE_STRUCT(test_msgs, msg, Defaults) *>(
      members[16].get_function(&message_struct->defaults_values, 2))->float32_value,
    1.125);
  EXPECT_EQ(members[16].resize_function, nullptr);
}

TEST_F(TypeSupportMemberStructuresTest, EmptyTypeIsCorrect)
{
  const rosidl_typesupport_introspection_c__MessageMember * members =
    all_message_members_structs["Empty"]->members_;

  // Is this really worth testing? It's almost an implementation detail that there is a struct with
  // a single member for an Empty message.
  EXPECT_STREQ(members[0].name_, "structure_needs_at_least_one_member");
  EXPECT_EQ(members[0].type_id_, rosidl_typesupport_introspection_c__ROS_TYPE_UINT8);
  EXPECT_EQ(members[0].string_upper_bound_, 0u);
  EXPECT_EQ(members[0].members_, nullptr);
  EXPECT_EQ(members[0].is_array_, false);
  EXPECT_EQ(members[0].array_size_, 0u);
  EXPECT_EQ(members[0].is_upper_bound_, false);
  EXPECT_EQ(
    members[0].offset_,
    offsetof(MESSAGE_STRUCT(test_msgs, msg, Empty), structure_needs_at_least_one_member));
  EXPECT_EQ(members[0].default_value_, nullptr);
  EXPECT_EQ(members[0].size_function, nullptr);
  EXPECT_EQ(members[0].get_const_function, nullptr);
  EXPECT_EQ(members[0].get_function, nullptr);
  EXPECT_EQ(members[0].resize_function, nullptr);
}

TEST_F(TypeSupportMemberStructuresTest, StringTypeIsCorrect)
{
  const rosidl_typesupport_introspection_c__MessageMember * members =
    all_message_members_structs["Strings"]->members_;

  EXPECT_STREQ(members[0].name_, "string_value");
  EXPECT_EQ(members[0].type_id_, rosidl_typesupport_introspection_c__ROS_TYPE_STRING);
  EXPECT_EQ(members[0].string_upper_bound_, 0u);
  EXPECT_EQ(members[0].members_, nullptr);
  EXPECT_EQ(members[0].is_array_, false);
  EXPECT_EQ(members[0].array_size_, 0u);
  EXPECT_EQ(members[0].is_upper_bound_, false);
  EXPECT_EQ(
    members[0].offset_,
    offsetof(MESSAGE_STRUCT(test_msgs, msg, Strings), string_value));
  EXPECT_EQ(members[0].default_value_, nullptr);
  EXPECT_EQ(members[0].size_function, nullptr);
  EXPECT_EQ(members[0].get_const_function, nullptr);
  EXPECT_EQ(members[0].get_function, nullptr);
  EXPECT_EQ(members[0].resize_function, nullptr);

  EXPECT_STREQ(members[1].name_, "string_value_default1");
  EXPECT_EQ(members[1].type_id_, rosidl_typesupport_introspection_c__ROS_TYPE_STRING);
  EXPECT_EQ(members[1].string_upper_bound_, 0u);
  EXPECT_EQ(members[1].members_, nullptr);
  EXPECT_EQ(members[1].is_array_, false);
  EXPECT_EQ(members[1].array_size_, 0u);
  EXPECT_EQ(members[1].is_upper_bound_, false);
  EXPECT_EQ(
    members[1].offset_,
    offsetof(MESSAGE_STRUCT(test_msgs, msg, Strings), string_value_default1));
  EXPECT_EQ(members[1].default_value_, nullptr);
  EXPECT_EQ(members[1].size_function, nullptr);
  EXPECT_EQ(members[1].get_const_function, nullptr);
  EXPECT_EQ(members[1].get_function, nullptr);
  EXPECT_EQ(members[1].resize_function, nullptr);

  EXPECT_STREQ(members[6].name_, "bounded_string_value");
  EXPECT_EQ(members[6].type_id_, rosidl_typesupport_introspection_c__ROS_TYPE_STRING);
  EXPECT_EQ(members[6].string_upper_bound_, 22u);
  EXPECT_EQ(members[6].members_, nullptr);
  EXPECT_EQ(members[6].is_array_, false);
  EXPECT_EQ(members[6].array_size_, 0u);
  EXPECT_EQ(members[6].is_upper_bound_, false);
  EXPECT_EQ(
    members[6].offset_,
    offsetof(MESSAGE_STRUCT(test_msgs, msg, Strings), bounded_string_value));
  EXPECT_EQ(members[6].default_value_, nullptr);
  EXPECT_EQ(members[6].size_function, nullptr);
  EXPECT_EQ(members[6].get_const_function, nullptr);
  EXPECT_EQ(members[6].get_function, nullptr);
  EXPECT_EQ(members[6].resize_function, nullptr);
}

TEST_F(TypeSupportMemberStructuresTest, BoundedSequencesAreCorrect)
{
  const rosidl_typesupport_introspection_c__MessageMember * members =
    all_message_members_structs["BoundedSequences"]->members_;

  EXPECT_STREQ(members[0].name_, "bool_values");
  EXPECT_EQ(members[0].type_id_, rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN);
  EXPECT_EQ(members[0].string_upper_bound_, 0u);
  EXPECT_EQ(members[0].members_, nullptr);
  EXPECT_EQ(members[0].is_array_, true);
  EXPECT_EQ(members[0].array_size_, 3u);
  EXPECT_EQ(members[0].is_upper_bound_, true);
  EXPECT_EQ(members[0].offset_, 0u);
  EXPECT_EQ(members[0].default_value_, nullptr);
  EXPECT_EQ(members[0].size_function, nullptr);
  EXPECT_EQ(members[0].get_const_function, nullptr);
  EXPECT_EQ(members[0].get_function, nullptr);
  EXPECT_EQ(members[0].resize_function, nullptr);

  EXPECT_STREQ(members[1].name_, "byte_values");
  EXPECT_EQ(members[1].type_id_, rosidl_typesupport_introspection_c__ROS_TYPE_OCTET);
  EXPECT_EQ(members[1].string_upper_bound_, 0u);
  EXPECT_EQ(members[1].members_, nullptr);
  EXPECT_EQ(members[1].is_array_, true);
  EXPECT_EQ(members[1].array_size_, 3u);
  EXPECT_EQ(members[1].is_upper_bound_, true);
  EXPECT_EQ(
    members[1].offset_,
    offsetof(MESSAGE_STRUCT(test_msgs, msg, BoundedSequences), byte_values));
  EXPECT_EQ(members[1].default_value_, nullptr);
  EXPECT_EQ(members[1].size_function, nullptr);
  EXPECT_EQ(members[1].get_const_function, nullptr);
  EXPECT_EQ(members[1].get_function, nullptr);
  EXPECT_EQ(members[1].resize_function, nullptr);

  EXPECT_STREQ(members[2].name_, "char_values");
  EXPECT_EQ(members[2].type_id_, rosidl_typesupport_introspection_c__ROS_TYPE_UINT8);
  EXPECT_EQ(members[2].string_upper_bound_, 0u);
  EXPECT_EQ(members[2].members_, nullptr);
  EXPECT_EQ(members[2].is_array_, true);
  EXPECT_EQ(members[2].array_size_, 3u);
  EXPECT_EQ(members[2].is_upper_bound_, true);
  EXPECT_EQ(
    members[2].offset_,
    offsetof(MESSAGE_STRUCT(test_msgs, msg, BoundedSequences), char_values));
  EXPECT_EQ(members[2].default_value_, nullptr);
  EXPECT_EQ(members[2].size_function, nullptr);
  EXPECT_EQ(members[2].get_const_function, nullptr);
  EXPECT_EQ(members[2].get_function, nullptr);
  EXPECT_EQ(members[2].resize_function, nullptr);

  EXPECT_STREQ(members[3].name_, "float32_values");
  EXPECT_EQ(members[3].type_id_, rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT32);
  EXPECT_EQ(members[3].string_upper_bound_, 0u);
  EXPECT_EQ(members[3].members_, nullptr);
  EXPECT_EQ(members[3].is_array_, true);
  EXPECT_EQ(members[3].array_size_, 3u);
  EXPECT_EQ(members[3].is_upper_bound_, true);
  EXPECT_EQ(
    members[3].offset_,
    offsetof(MESSAGE_STRUCT(test_msgs, msg, BoundedSequences), float32_values));
  EXPECT_EQ(members[3].default_value_, nullptr);
  EXPECT_EQ(members[3].size_function, nullptr);
  EXPECT_EQ(members[3].get_const_function, nullptr);
  EXPECT_EQ(members[3].get_function, nullptr);
  EXPECT_EQ(members[3].resize_function, nullptr);

  EXPECT_STREQ(members[4].name_, "float64_values");
  EXPECT_EQ(members[4].type_id_, rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT64);
  EXPECT_EQ(members[4].string_upper_bound_, 0u);
  EXPECT_EQ(members[4].members_, nullptr);
  EXPECT_EQ(members[4].is_array_, true);
  EXPECT_EQ(members[4].array_size_, 3u);
  EXPECT_EQ(members[4].is_upper_bound_, true);
  EXPECT_EQ(
    members[4].offset_,
    offsetof(MESSAGE_STRUCT(test_msgs, msg, BoundedSequences), float64_values));
  EXPECT_EQ(members[4].default_value_, nullptr);
  EXPECT_EQ(members[4].size_function, nullptr);
  EXPECT_EQ(members[4].get_const_function, nullptr);
  EXPECT_EQ(members[4].get_function, nullptr);
  EXPECT_EQ(members[4].resize_function, nullptr);

  EXPECT_STREQ(members[5].name_, "int8_values");
  EXPECT_EQ(members[5].type_id_, rosidl_typesupport_introspection_c__ROS_TYPE_INT8);
  EXPECT_EQ(members[5].string_upper_bound_, 0u);
  EXPECT_EQ(members[5].members_, nullptr);
  EXPECT_EQ(members[5].is_array_, true);
  EXPECT_EQ(members[5].array_size_, 3u);
  EXPECT_EQ(members[5].is_upper_bound_, true);
  EXPECT_EQ(
    members[5].offset_,
    offsetof(MESSAGE_STRUCT(test_msgs, msg, BoundedSequences), int8_values));
  EXPECT_EQ(members[5].default_value_, nullptr);
  EXPECT_EQ(members[5].size_function, nullptr);
  EXPECT_EQ(members[5].get_const_function, nullptr);
  EXPECT_EQ(members[5].get_function, nullptr);
  EXPECT_EQ(members[5].resize_function, nullptr);

  EXPECT_STREQ(members[6].name_, "uint8_values");
  EXPECT_EQ(members[6].type_id_, rosidl_typesupport_introspection_c__ROS_TYPE_UINT8);
  EXPECT_EQ(members[6].string_upper_bound_, 0u);
  EXPECT_EQ(members[6].members_, nullptr);
  EXPECT_EQ(members[6].is_array_, true);
  EXPECT_EQ(members[6].array_size_, 3u);
  EXPECT_EQ(members[6].is_upper_bound_, true);
  EXPECT_EQ(
    members[6].offset_,
    offsetof(MESSAGE_STRUCT(test_msgs, msg, BoundedSequences), uint8_values));
  EXPECT_EQ(members[6].default_value_, nullptr);
  EXPECT_EQ(members[6].size_function, nullptr);
  EXPECT_EQ(members[6].get_const_function, nullptr);
  EXPECT_EQ(members[6].get_function, nullptr);
  EXPECT_EQ(members[6].resize_function, nullptr);

  EXPECT_STREQ(members[7].name_, "int16_values");
  EXPECT_EQ(members[7].type_id_, rosidl_typesupport_introspection_c__ROS_TYPE_INT16);
  EXPECT_EQ(members[7].string_upper_bound_, 0u);
  EXPECT_EQ(members[7].members_, nullptr);
  EXPECT_EQ(members[7].is_array_, true);
  EXPECT_EQ(members[7].array_size_, 3u);
  EXPECT_EQ(members[7].is_upper_bound_, true);
  EXPECT_EQ(
    members[7].offset_,
    offsetof(MESSAGE_STRUCT(test_msgs, msg, BoundedSequences), int16_values));
  EXPECT_EQ(members[7].default_value_, nullptr);
  EXPECT_EQ(members[7].size_function, nullptr);
  EXPECT_EQ(members[7].get_const_function, nullptr);
  EXPECT_EQ(members[7].get_function, nullptr);
  EXPECT_EQ(members[7].resize_function, nullptr);

  EXPECT_STREQ(members[8].name_, "uint16_values");
  EXPECT_EQ(members[8].type_id_, rosidl_typesupport_introspection_c__ROS_TYPE_UINT16);
  EXPECT_EQ(members[8].string_upper_bound_, 0u);
  EXPECT_EQ(members[8].members_, nullptr);
  EXPECT_EQ(members[8].is_array_, true);
  EXPECT_EQ(members[8].array_size_, 3u);
  EXPECT_EQ(members[8].is_upper_bound_, true);
  EXPECT_EQ(
    members[8].offset_,
    offsetof(MESSAGE_STRUCT(test_msgs, msg, BoundedSequences), uint16_values));
  EXPECT_EQ(members[8].default_value_, nullptr);
  EXPECT_EQ(members[8].size_function, nullptr);
  EXPECT_EQ(members[8].get_const_function, nullptr);
  EXPECT_EQ(members[8].get_function, nullptr);
  EXPECT_EQ(members[8].resize_function, nullptr);

  EXPECT_STREQ(members[9].name_, "int32_values");
  EXPECT_EQ(members[9].type_id_, rosidl_typesupport_introspection_c__ROS_TYPE_INT32);
  EXPECT_EQ(members[9].string_upper_bound_, 0u);
  EXPECT_EQ(members[9].members_, nullptr);
  EXPECT_EQ(members[9].is_array_, true);
  EXPECT_EQ(members[9].array_size_, 3u);
  EXPECT_EQ(members[9].is_upper_bound_, true);
  EXPECT_EQ(
    members[9].offset_,
    offsetof(MESSAGE_STRUCT(test_msgs, msg, BoundedSequences), int32_values));
  EXPECT_EQ(members[9].default_value_, nullptr);
  EXPECT_EQ(members[9].size_function, nullptr);
  EXPECT_EQ(members[9].get_const_function, nullptr);
  EXPECT_EQ(members[9].get_function, nullptr);
  EXPECT_EQ(members[9].resize_function, nullptr);

  EXPECT_STREQ(members[10].name_, "uint32_values");
  EXPECT_EQ(members[10].type_id_, rosidl_typesupport_introspection_c__ROS_TYPE_UINT32);
  EXPECT_EQ(members[10].string_upper_bound_, 0u);
  EXPECT_EQ(members[10].members_, nullptr);
  EXPECT_EQ(members[10].is_array_, true);
  EXPECT_EQ(members[10].array_size_, 3u);
  EXPECT_EQ(members[10].is_upper_bound_, true);
  EXPECT_EQ(
    members[10].offset_,
    offsetof(MESSAGE_STRUCT(test_msgs, msg, BoundedSequences), uint32_values));
  EXPECT_EQ(members[10].default_value_, nullptr);
  EXPECT_EQ(members[10].size_function, nullptr);
  EXPECT_EQ(members[10].get_const_function, nullptr);
  EXPECT_EQ(members[10].get_function, nullptr);
  EXPECT_EQ(members[10].resize_function, nullptr);

  EXPECT_STREQ(members[11].name_, "int64_values");
  EXPECT_EQ(members[11].type_id_, rosidl_typesupport_introspection_c__ROS_TYPE_INT64);
  EXPECT_EQ(members[11].string_upper_bound_, 0u);
  EXPECT_EQ(members[11].members_, nullptr);
  EXPECT_EQ(members[11].is_array_, true);
  EXPECT_EQ(members[11].array_size_, 3u);
  EXPECT_EQ(members[11].is_upper_bound_, true);
  EXPECT_EQ(
    members[11].offset_,
    offsetof(MESSAGE_STRUCT(test_msgs, msg, BoundedSequences), int64_values));
  EXPECT_EQ(members[11].default_value_, nullptr);
  EXPECT_EQ(members[11].size_function, nullptr);
  EXPECT_EQ(members[11].get_const_function, nullptr);
  EXPECT_EQ(members[11].get_function, nullptr);
  EXPECT_EQ(members[11].resize_function, nullptr);

  EXPECT_STREQ(members[12].name_, "uint64_values");
  EXPECT_EQ(members[12].type_id_, rosidl_typesupport_introspection_c__ROS_TYPE_UINT64);
  EXPECT_EQ(members[12].string_upper_bound_, 0u);
  EXPECT_EQ(members[12].members_, nullptr);
  EXPECT_EQ(members[12].is_array_, true);
  EXPECT_EQ(members[12].array_size_, 3u);
  EXPECT_EQ(members[12].is_upper_bound_, true);
  EXPECT_EQ(
    members[12].offset_,
    offsetof(MESSAGE_STRUCT(test_msgs, msg, BoundedSequences), uint64_values));
  EXPECT_EQ(members[12].default_value_, nullptr);
  EXPECT_EQ(members[12].size_function, nullptr);
  EXPECT_EQ(members[12].get_const_function, nullptr);
  EXPECT_EQ(members[12].get_function, nullptr);
  EXPECT_EQ(members[12].resize_function, nullptr);

  EXPECT_STREQ(members[13].name_, "string_values");
  EXPECT_EQ(members[13].type_id_, rosidl_typesupport_introspection_c__ROS_TYPE_STRING);
  EXPECT_EQ(members[13].string_upper_bound_, 0u);
  EXPECT_EQ(members[13].members_, nullptr);
  EXPECT_EQ(members[13].is_array_, true);
  EXPECT_EQ(members[13].array_size_, 3u);
  EXPECT_EQ(members[13].is_upper_bound_, true);
  EXPECT_EQ(
    members[13].offset_,
    offsetof(MESSAGE_STRUCT(test_msgs, msg, BoundedSequences), string_values));
  EXPECT_EQ(members[13].default_value_, nullptr);
  EXPECT_EQ(members[13].size_function, nullptr);
  EXPECT_EQ(members[13].get_const_function, nullptr);
  EXPECT_EQ(members[13].get_function, nullptr);
  EXPECT_EQ(members[13].resize_function, nullptr);

  EXPECT_STREQ(members[16].name_, "defaults_values");
  EXPECT_EQ(members[16].type_id_, rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE);
  EXPECT_EQ(members[16].string_upper_bound_, 0u);
  // TODO(gbiggs): Test the contained members are correct to verify this field was initialised
  // correctly
  EXPECT_NE(members[16].members_, nullptr);
  EXPECT_EQ(members[16].is_array_, true);
  EXPECT_EQ(members[16].array_size_, 3u);
  EXPECT_EQ(members[16].is_upper_bound_, true);
  EXPECT_EQ(
    members[16].offset_,
    offsetof(MESSAGE_STRUCT(test_msgs, msg, BoundedSequences), defaults_values));
  EXPECT_EQ(members[16].default_value_, nullptr);
  // TODO(gbiggs): These functions need to be called to check they are correct
  EXPECT_NE(members[16].size_function, nullptr);
  EXPECT_NE(members[16].get_const_function, nullptr);
  EXPECT_NE(members[16].get_function, nullptr);
  EXPECT_NE(members[16].resize_function, nullptr);
}

TEST_F(TypeSupportMemberStructuresTest, UnboundedSequencesAreCorrect)
{
  const rosidl_typesupport_introspection_c__MessageMember * members =
    all_message_members_structs["UnboundedSequences"]->members_;

  EXPECT_STREQ(members[0].name_, "bool_values");
  EXPECT_EQ(members[0].type_id_, rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN);
  EXPECT_EQ(members[0].string_upper_bound_, 0u);
  EXPECT_EQ(members[0].members_, nullptr);
  EXPECT_EQ(members[0].is_array_, true);
  EXPECT_EQ(members[0].array_size_, 0u);
  EXPECT_EQ(members[0].is_upper_bound_, false);
  EXPECT_EQ(members[0].offset_, 0u);
  EXPECT_EQ(members[0].default_value_, nullptr);
  EXPECT_EQ(members[0].size_function, nullptr);
  EXPECT_EQ(members[0].get_const_function, nullptr);
  EXPECT_EQ(members[0].get_function, nullptr);
  EXPECT_EQ(members[0].resize_function, nullptr);

  EXPECT_STREQ(members[1].name_, "byte_values");
  EXPECT_EQ(members[1].type_id_, rosidl_typesupport_introspection_c__ROS_TYPE_OCTET);
  EXPECT_EQ(members[1].string_upper_bound_, 0u);
  EXPECT_EQ(members[1].members_, nullptr);
  EXPECT_EQ(members[1].is_array_, true);
  EXPECT_EQ(members[1].array_size_, 0u);
  EXPECT_EQ(members[1].is_upper_bound_, false);
  EXPECT_EQ(
    members[1].offset_,
    offsetof(MESSAGE_STRUCT(test_msgs, msg, UnboundedSequences), byte_values));
  EXPECT_EQ(members[1].default_value_, nullptr);
  EXPECT_EQ(members[1].size_function, nullptr);
  EXPECT_EQ(members[1].get_const_function, nullptr);
  EXPECT_EQ(members[1].get_function, nullptr);
  EXPECT_EQ(members[1].resize_function, nullptr);

  EXPECT_STREQ(members[2].name_, "char_values");
  EXPECT_EQ(members[2].type_id_, rosidl_typesupport_introspection_c__ROS_TYPE_UINT8);
  EXPECT_EQ(members[2].string_upper_bound_, 0u);
  EXPECT_EQ(members[2].members_, nullptr);
  EXPECT_EQ(members[2].is_array_, true);
  EXPECT_EQ(members[2].array_size_, 0u);
  EXPECT_EQ(members[2].is_upper_bound_, false);
  EXPECT_EQ(
    members[2].offset_,
    offsetof(MESSAGE_STRUCT(test_msgs, msg, UnboundedSequences), char_values));
  EXPECT_EQ(members[2].default_value_, nullptr);
  EXPECT_EQ(members[2].size_function, nullptr);
  EXPECT_EQ(members[2].get_const_function, nullptr);
  EXPECT_EQ(members[2].get_function, nullptr);
  EXPECT_EQ(members[2].resize_function, nullptr);

  EXPECT_STREQ(members[3].name_, "float32_values");
  EXPECT_EQ(members[3].type_id_, rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT32);
  EXPECT_EQ(members[3].string_upper_bound_, 0u);
  EXPECT_EQ(members[3].members_, nullptr);
  EXPECT_EQ(members[3].is_array_, true);
  EXPECT_EQ(members[3].array_size_, 0u);
  EXPECT_EQ(members[3].is_upper_bound_, false);
  EXPECT_EQ(
    members[3].offset_,
    offsetof(MESSAGE_STRUCT(test_msgs, msg, UnboundedSequences), float32_values));
  EXPECT_EQ(members[3].default_value_, nullptr);
  EXPECT_EQ(members[3].size_function, nullptr);
  EXPECT_EQ(members[3].get_const_function, nullptr);
  EXPECT_EQ(members[3].get_function, nullptr);
  EXPECT_EQ(members[3].resize_function, nullptr);

  EXPECT_STREQ(members[4].name_, "float64_values");
  EXPECT_EQ(members[4].type_id_, rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT64);
  EXPECT_EQ(members[4].string_upper_bound_, 0u);
  EXPECT_EQ(members[4].members_, nullptr);
  EXPECT_EQ(members[4].is_array_, true);
  EXPECT_EQ(members[4].array_size_, 0u);
  EXPECT_EQ(members[4].is_upper_bound_, false);
  EXPECT_EQ(
    members[4].offset_,
    offsetof(MESSAGE_STRUCT(test_msgs, msg, UnboundedSequences), float64_values));
  EXPECT_EQ(members[4].default_value_, nullptr);
  EXPECT_EQ(members[4].size_function, nullptr);
  EXPECT_EQ(members[4].get_const_function, nullptr);
  EXPECT_EQ(members[4].get_function, nullptr);
  EXPECT_EQ(members[4].resize_function, nullptr);

  EXPECT_STREQ(members[5].name_, "int8_values");
  EXPECT_EQ(members[5].type_id_, rosidl_typesupport_introspection_c__ROS_TYPE_INT8);
  EXPECT_EQ(members[5].string_upper_bound_, 0u);
  EXPECT_EQ(members[5].members_, nullptr);
  EXPECT_EQ(members[5].is_array_, true);
  EXPECT_EQ(members[5].array_size_, 0u);
  EXPECT_EQ(members[5].is_upper_bound_, false);
  EXPECT_EQ(
    members[5].offset_,
    offsetof(MESSAGE_STRUCT(test_msgs, msg, UnboundedSequences), int8_values));
  EXPECT_EQ(members[5].default_value_, nullptr);
  EXPECT_EQ(members[5].size_function, nullptr);
  EXPECT_EQ(members[5].get_const_function, nullptr);
  EXPECT_EQ(members[5].get_function, nullptr);
  EXPECT_EQ(members[5].resize_function, nullptr);

  EXPECT_STREQ(members[6].name_, "uint8_values");
  EXPECT_EQ(members[6].type_id_, rosidl_typesupport_introspection_c__ROS_TYPE_UINT8);
  EXPECT_EQ(members[6].string_upper_bound_, 0u);
  EXPECT_EQ(members[6].members_, nullptr);
  EXPECT_EQ(members[6].is_array_, true);
  EXPECT_EQ(members[6].array_size_, 0u);
  EXPECT_EQ(members[6].is_upper_bound_, false);
  EXPECT_EQ(
    members[6].offset_,
    offsetof(MESSAGE_STRUCT(test_msgs, msg, UnboundedSequences), uint8_values));
  EXPECT_EQ(members[6].default_value_, nullptr);
  EXPECT_EQ(members[6].size_function, nullptr);
  EXPECT_EQ(members[6].get_const_function, nullptr);
  EXPECT_EQ(members[6].get_function, nullptr);
  EXPECT_EQ(members[6].resize_function, nullptr);

  EXPECT_STREQ(members[7].name_, "int16_values");
  EXPECT_EQ(members[7].type_id_, rosidl_typesupport_introspection_c__ROS_TYPE_INT16);
  EXPECT_EQ(members[7].string_upper_bound_, 0u);
  EXPECT_EQ(members[7].members_, nullptr);
  EXPECT_EQ(members[7].is_array_, true);
  EXPECT_EQ(members[7].array_size_, 0u);
  EXPECT_EQ(members[7].is_upper_bound_, false);
  EXPECT_EQ(
    members[7].offset_,
    offsetof(MESSAGE_STRUCT(test_msgs, msg, UnboundedSequences), int16_values));
  EXPECT_EQ(members[7].default_value_, nullptr);
  EXPECT_EQ(members[7].size_function, nullptr);
  EXPECT_EQ(members[7].get_const_function, nullptr);
  EXPECT_EQ(members[7].get_function, nullptr);
  EXPECT_EQ(members[7].resize_function, nullptr);

  EXPECT_STREQ(members[8].name_, "uint16_values");
  EXPECT_EQ(members[8].type_id_, rosidl_typesupport_introspection_c__ROS_TYPE_UINT16);
  EXPECT_EQ(members[8].string_upper_bound_, 0u);
  EXPECT_EQ(members[8].members_, nullptr);
  EXPECT_EQ(members[8].is_array_, true);
  EXPECT_EQ(members[8].array_size_, 0u);
  EXPECT_EQ(members[8].is_upper_bound_, false);
  EXPECT_EQ(
    members[8].offset_,
    offsetof(MESSAGE_STRUCT(test_msgs, msg, UnboundedSequences), uint16_values));
  EXPECT_EQ(members[8].default_value_, nullptr);
  EXPECT_EQ(members[8].size_function, nullptr);
  EXPECT_EQ(members[8].get_const_function, nullptr);
  EXPECT_EQ(members[8].get_function, nullptr);
  EXPECT_EQ(members[8].resize_function, nullptr);

  EXPECT_STREQ(members[9].name_, "int32_values");
  EXPECT_EQ(members[9].type_id_, rosidl_typesupport_introspection_c__ROS_TYPE_INT32);
  EXPECT_EQ(members[9].string_upper_bound_, 0u);
  EXPECT_EQ(members[9].members_, nullptr);
  EXPECT_EQ(members[9].is_array_, true);
  EXPECT_EQ(members[9].array_size_, 0u);
  EXPECT_EQ(members[9].is_upper_bound_, false);
  EXPECT_EQ(
    members[9].offset_,
    offsetof(MESSAGE_STRUCT(test_msgs, msg, UnboundedSequences), int32_values));
  EXPECT_EQ(members[9].default_value_, nullptr);
  EXPECT_EQ(members[9].size_function, nullptr);
  EXPECT_EQ(members[9].get_const_function, nullptr);
  EXPECT_EQ(members[9].get_function, nullptr);
  EXPECT_EQ(members[9].resize_function, nullptr);

  EXPECT_STREQ(members[10].name_, "uint32_values");
  EXPECT_EQ(members[10].type_id_, rosidl_typesupport_introspection_c__ROS_TYPE_UINT32);
  EXPECT_EQ(members[10].string_upper_bound_, 0u);
  EXPECT_EQ(members[10].members_, nullptr);
  EXPECT_EQ(members[10].is_array_, true);
  EXPECT_EQ(members[10].array_size_, 0u);
  EXPECT_EQ(members[10].is_upper_bound_, false);
  EXPECT_EQ(
    members[10].offset_,
    offsetof(MESSAGE_STRUCT(test_msgs, msg, UnboundedSequences), uint32_values));
  EXPECT_EQ(members[10].default_value_, nullptr);
  EXPECT_EQ(members[10].size_function, nullptr);
  EXPECT_EQ(members[10].get_const_function, nullptr);
  EXPECT_EQ(members[10].get_function, nullptr);
  EXPECT_EQ(members[10].resize_function, nullptr);

  EXPECT_STREQ(members[11].name_, "int64_values");
  EXPECT_EQ(members[11].type_id_, rosidl_typesupport_introspection_c__ROS_TYPE_INT64);
  EXPECT_EQ(members[11].string_upper_bound_, 0u);
  EXPECT_EQ(members[11].members_, nullptr);
  EXPECT_EQ(members[11].is_array_, true);
  EXPECT_EQ(members[11].array_size_, 0u);
  EXPECT_EQ(members[11].is_upper_bound_, false);
  EXPECT_EQ(
    members[11].offset_,
    offsetof(MESSAGE_STRUCT(test_msgs, msg, UnboundedSequences), int64_values));
  EXPECT_EQ(members[11].default_value_, nullptr);
  EXPECT_EQ(members[11].size_function, nullptr);
  EXPECT_EQ(members[11].get_const_function, nullptr);
  EXPECT_EQ(members[11].get_function, nullptr);
  EXPECT_EQ(members[11].resize_function, nullptr);

  EXPECT_STREQ(members[12].name_, "uint64_values");
  EXPECT_EQ(members[12].type_id_, rosidl_typesupport_introspection_c__ROS_TYPE_UINT64);
  EXPECT_EQ(members[12].string_upper_bound_, 0u);
  EXPECT_EQ(members[12].members_, nullptr);
  EXPECT_EQ(members[12].is_array_, true);
  EXPECT_EQ(members[12].array_size_, 0u);
  EXPECT_EQ(members[12].is_upper_bound_, false);
  EXPECT_EQ(
    members[12].offset_,
    offsetof(MESSAGE_STRUCT(test_msgs, msg, UnboundedSequences), uint64_values));
  EXPECT_EQ(members[12].default_value_, nullptr);
  EXPECT_EQ(members[12].size_function, nullptr);
  EXPECT_EQ(members[12].get_const_function, nullptr);
  EXPECT_EQ(members[12].get_function, nullptr);
  EXPECT_EQ(members[12].resize_function, nullptr);

  EXPECT_STREQ(members[13].name_, "string_values");
  EXPECT_EQ(members[13].type_id_, rosidl_typesupport_introspection_c__ROS_TYPE_STRING);
  EXPECT_EQ(members[13].string_upper_bound_, 0u);
  EXPECT_EQ(members[13].members_, nullptr);
  EXPECT_EQ(members[13].is_array_, true);
  EXPECT_EQ(members[13].array_size_, 0u);
  EXPECT_EQ(members[13].is_upper_bound_, false);
  EXPECT_EQ(
    members[13].offset_,
    offsetof(MESSAGE_STRUCT(test_msgs, msg, UnboundedSequences), string_values));
  EXPECT_EQ(members[13].default_value_, nullptr);
  EXPECT_EQ(members[13].size_function, nullptr);
  EXPECT_EQ(members[13].get_const_function, nullptr);
  EXPECT_EQ(members[13].get_function, nullptr);
  EXPECT_EQ(members[13].resize_function, nullptr);

  EXPECT_STREQ(members[14].name_, "basic_types_values");
  EXPECT_EQ(members[14].type_id_, rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE);
  EXPECT_EQ(members[14].string_upper_bound_, 0u);
  // TODO(gbiggs): Test the contained members are correct to verify this field was initialised
  // correctly
  EXPECT_NE(members[14].members_, nullptr);
  EXPECT_EQ(members[14].is_array_, true);
  EXPECT_EQ(members[14].array_size_, 0u);
  EXPECT_EQ(members[14].is_upper_bound_, false);
  EXPECT_EQ(
    members[14].offset_,
    offsetof(MESSAGE_STRUCT(test_msgs, msg, UnboundedSequences), basic_types_values));
  EXPECT_EQ(members[14].default_value_, nullptr);
  // TODO(gbiggs): These functions need to be called to check they are correct
  EXPECT_NE(members[14].size_function, nullptr);
  EXPECT_NE(members[14].get_const_function, nullptr);
  EXPECT_NE(members[14].get_function, nullptr);
  EXPECT_NE(members[14].resize_function, nullptr);
}

TEST_F(TypeSupportMemberStructuresTest, WStringTypeIsCorrect)
{
  const rosidl_typesupport_introspection_c__MessageMember * members =
    all_message_members_structs["WStrings"]->members_;

  EXPECT_STREQ(members[0].name_, "wstring_value");
  EXPECT_EQ(members[0].type_id_, rosidl_typesupport_introspection_c__ROS_TYPE_WSTRING);
  EXPECT_EQ(members[0].string_upper_bound_, 0u);
  EXPECT_EQ(members[0].members_, nullptr);
  EXPECT_EQ(members[0].is_array_, false);
  EXPECT_EQ(members[0].array_size_, 0u);
  EXPECT_EQ(members[0].is_upper_bound_, false);
  EXPECT_EQ(
    members[0].offset_,
    offsetof(MESSAGE_STRUCT(test_msgs, msg, WStrings), wstring_value));
  EXPECT_EQ(members[0].default_value_, nullptr);
  EXPECT_EQ(members[0].size_function, nullptr);
  EXPECT_EQ(members[0].get_const_function, nullptr);
  EXPECT_EQ(members[0].get_function, nullptr);
  EXPECT_EQ(members[0].resize_function, nullptr);

  EXPECT_STREQ(members[1].name_, "wstring_value_default1");
  EXPECT_EQ(members[1].type_id_, rosidl_typesupport_introspection_c__ROS_TYPE_WSTRING);
  EXPECT_EQ(members[1].string_upper_bound_, 0u);
  EXPECT_EQ(members[1].members_, nullptr);
  EXPECT_EQ(members[1].is_array_, false);
  EXPECT_EQ(members[1].array_size_, 0u);
  EXPECT_EQ(members[1].is_upper_bound_, false);
  EXPECT_EQ(
    members[1].offset_,
    offsetof(MESSAGE_STRUCT(test_msgs, msg, WStrings), wstring_value_default1));
  EXPECT_EQ(members[1].default_value_, nullptr);
  EXPECT_EQ(members[1].size_function, nullptr);
  EXPECT_EQ(members[1].get_const_function, nullptr);
  EXPECT_EQ(members[1].get_function, nullptr);
  EXPECT_EQ(members[1].resize_function, nullptr);

  EXPECT_STREQ(members[4].name_, "array_of_wstrings");
  EXPECT_EQ(members[4].type_id_, rosidl_typesupport_introspection_c__ROS_TYPE_WSTRING);
  EXPECT_EQ(members[4].string_upper_bound_, 0u);
  EXPECT_EQ(members[4].members_, nullptr);
  EXPECT_EQ(members[4].is_array_, true);
  EXPECT_EQ(members[4].array_size_, 3u);
  EXPECT_EQ(members[4].is_upper_bound_, false);
  EXPECT_EQ(
    members[4].offset_,
    offsetof(MESSAGE_STRUCT(test_msgs, msg, WStrings), array_of_wstrings));
  EXPECT_EQ(members[4].default_value_, nullptr);
  EXPECT_EQ(members[4].size_function, nullptr);
  EXPECT_EQ(members[4].get_const_function, nullptr);
  EXPECT_EQ(members[4].get_function, nullptr);
  EXPECT_EQ(members[4].resize_function, nullptr);

  EXPECT_STREQ(members[5].name_, "bounded_sequence_of_wstrings");
  EXPECT_EQ(members[5].type_id_, rosidl_typesupport_introspection_c__ROS_TYPE_WSTRING);
  EXPECT_EQ(members[5].string_upper_bound_, 0u);
  EXPECT_EQ(members[5].members_, nullptr);
  EXPECT_EQ(members[5].is_array_, true);
  EXPECT_EQ(members[5].array_size_, 3u);
  EXPECT_EQ(members[5].is_upper_bound_, true);
  EXPECT_EQ(
    members[5].offset_,
    offsetof(MESSAGE_STRUCT(test_msgs, msg, WStrings), bounded_sequence_of_wstrings));
  EXPECT_EQ(members[5].default_value_, nullptr);
  EXPECT_EQ(members[5].size_function, nullptr);
  EXPECT_EQ(members[5].get_const_function, nullptr);
  EXPECT_EQ(members[5].get_function, nullptr);
  EXPECT_EQ(members[5].resize_function, nullptr);

  EXPECT_STREQ(members[6].name_, "unbounded_sequence_of_wstrings");
  EXPECT_EQ(members[6].type_id_, rosidl_typesupport_introspection_c__ROS_TYPE_WSTRING);
  EXPECT_EQ(members[6].string_upper_bound_, 0u);
  EXPECT_EQ(members[6].members_, nullptr);
  EXPECT_EQ(members[6].is_array_, true);
  EXPECT_EQ(members[6].array_size_, 0u);
  EXPECT_EQ(members[6].is_upper_bound_, false);
  EXPECT_EQ(
    members[6].offset_,
    offsetof(MESSAGE_STRUCT(test_msgs, msg, WStrings), unbounded_sequence_of_wstrings));
  EXPECT_EQ(members[6].default_value_, nullptr);
  EXPECT_EQ(members[6].size_function, nullptr);
  EXPECT_EQ(members[6].get_const_function, nullptr);
  EXPECT_EQ(members[6].get_function, nullptr);
  EXPECT_EQ(members[6].resize_function, nullptr);
}
