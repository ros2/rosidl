// Copyright 2022 Open Source Robotics Foundation, Inc.
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

#ifndef TEST_MSGS_INTROSPECTION__TEST_MSGS_TYPES_HPP_
#define TEST_MSGS_INTROSPECTION__TEST_MSGS_TYPES_HPP_

#include <rcutils/macros.h>

#include <rosidl_runtime_c/string_functions.h>
#include <rosidl_runtime_c/u16string_functions.h>

#include <rosidl_typesupport_interface/macros.h>
#include <rosidl_typesupport_introspection_c/message_introspection.h>
#include <rosidl_typesupport_introspection_c/service_introspection.h>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
#include <rosidl_typesupport_introspection_cpp/service_introspection.hpp>

#include <test_msgs/msg/arrays.h>
#include <test_msgs/msg/arrays.hpp>
#include <test_msgs/msg/basic_types.h>
#include <test_msgs/msg/basic_types.hpp>
#include <test_msgs/msg/constants.h>
#include <test_msgs/msg/constants.hpp>
#include <test_msgs/msg/defaults.h>
#include <test_msgs/msg/defaults.hpp>
#include <test_msgs/msg/empty.h>
#include <test_msgs/msg/empty.hpp>

#include <memory>

#include "rosidl_typesupport_introspection_tests/fixtures.hpp"
#include "rosidl_typesupport_introspection_tests/helpers.hpp"
#include "rosidl_typesupport_introspection_tests/type_traits.hpp"

namespace rosidl_typesupport_introspection_tests
{

// Definition of typesupport library for introspection
// of `test_msgs` package interfaces in C
struct TestMsgsIntrospectionCTypeSupportLibrary
{
  using MessageDescriptorT =
    rosidl_typesupport_introspection_c__MessageMembers;
  using ServiceDescriptorT =
    rosidl_typesupport_introspection_c__ServiceMembers;
  using MemberDescriptorT =
    rosidl_typesupport_introspection_c__MessageMember;

  static constexpr const char * name =
    RCUTILS_STRINGIFY(
    ROSIDL_TYPESUPPORT_INTERFACE__LIBRARY_NAME(
      rosidl_typesupport_introspection_c, test_msgs));
  static constexpr const char * identifier =
    "rosidl_typesupport_introspection_c";

  static constexpr const char * messages_namespace = "test_msgs__msg";
  static constexpr const char * services_namespace = "test_msgs__srv";
  static constexpr const char * actions_namespace = "test_msgs__action";

  static constexpr const MessageTypeSupportSymbolRecord messages[] = {
    MESSAGE_TYPESUPPORT_SYMBOL_RECORD(
      rosidl_typesupport_introspection_c, test_msgs, msg, Arrays),
    MESSAGE_TYPESUPPORT_SYMBOL_RECORD(
      rosidl_typesupport_introspection_c, test_msgs, msg, BasicTypes),
    MESSAGE_TYPESUPPORT_SYMBOL_RECORD(
      rosidl_typesupport_introspection_c, test_msgs, msg, Constants),
    MESSAGE_TYPESUPPORT_SYMBOL_RECORD(
      rosidl_typesupport_introspection_c, test_msgs, msg, Defaults),
    MESSAGE_TYPESUPPORT_SYMBOL_RECORD(
      rosidl_typesupport_introspection_c, test_msgs, msg, Empty)
  };
  static constexpr const ServiceTypeSupportSymbolRecord services[] = {
    SERVICE_TYPESUPPORT_SYMBOL_RECORD(
      rosidl_typesupport_introspection_c, test_msgs, srv, BasicTypes)
  };
  static constexpr const ActionTypeSupportSymbolRecord actions[] = {
    ACTION_TYPESUPPORT_SYMBOL_RECORD(
      rosidl_typesupport_introspection_c, test_msgs, action, Fibonacci)
  };
};

// Traits to aid introspection of `test_msgs` package interfaces in C
template<>
struct introspection_traits<test_msgs__msg__Arrays>
{
  static constexpr const MessageTypeSupportSymbolRecord typesupport =
    MESSAGE_TYPESUPPORT_SYMBOL_RECORD(
    rosidl_typesupport_introspection_c, test_msgs, msg, Arrays);
  using TypeSupportLibraryT = TestMsgsIntrospectionCTypeSupportLibrary;
};

template<>
struct introspection_traits<test_msgs__msg__BasicTypes>
{
  static constexpr const MessageTypeSupportSymbolRecord typesupport =
    MESSAGE_TYPESUPPORT_SYMBOL_RECORD(
    rosidl_typesupport_introspection_c, test_msgs, msg, BasicTypes);
  using TypeSupportLibraryT = TestMsgsIntrospectionCTypeSupportLibrary;
};

template<>
struct introspection_traits<test_msgs__msg__Constants>
{
  static constexpr const MessageTypeSupportSymbolRecord typesupport =
    MESSAGE_TYPESUPPORT_SYMBOL_RECORD(
    rosidl_typesupport_introspection_c, test_msgs, msg, Constants);
  using TypeSupportLibraryT = TestMsgsIntrospectionCTypeSupportLibrary;
};

template<>
struct introspection_traits<test_msgs__msg__Defaults>
{
  static constexpr const MessageTypeSupportSymbolRecord typesupport =
    MESSAGE_TYPESUPPORT_SYMBOL_RECORD(
    rosidl_typesupport_introspection_c, test_msgs, msg, Defaults);
  using TypeSupportLibraryT = TestMsgsIntrospectionCTypeSupportLibrary;
};

template<>
struct introspection_traits<test_msgs__msg__Empty>
{
  static constexpr const MessageTypeSupportSymbolRecord typesupport =
    MESSAGE_TYPESUPPORT_SYMBOL_RECORD(
    rosidl_typesupport_introspection_c, test_msgs, msg, Empty);
  using TypeSupportLibraryT = TestMsgsIntrospectionCTypeSupportLibrary;
};

// Examples of `test_msgs` package interfaces in C, useful in test fixtures
template<>
struct Example<test_msgs__msg__Arrays>
{
  static auto Make()
  {
    using ReturnT = std::unique_ptr<
      test_msgs__msg__Arrays,
      std::function<void (test_msgs__msg__Arrays *)>>;

    auto deleter = [](test_msgs__msg__Arrays * message) {
        test_msgs__msg__Arrays__fini(message);
        delete message;
      };
    auto * message = new test_msgs__msg__Arrays;
    test_msgs__msg__Arrays__init(message);
    message->bool_values[2] = true;
    message->uint16_values[0] = 1234u;
    return ReturnT{message, deleter};
  }
};

// Definition of typesupport library for introspection
// of `test_msgs` package interfaces in C++
struct TestMsgsIntrospectionCppTypeSupportLibrary
{
  using MessageDescriptorT =
    rosidl_typesupport_introspection_cpp::MessageMembers;
  using ServiceDescriptorT =
    rosidl_typesupport_introspection_cpp::ServiceMembers;
  using MemberDescriptorT =
    rosidl_typesupport_introspection_cpp::MessageMember;

  static constexpr const char * name =
    RCUTILS_STRINGIFY(
    ROSIDL_TYPESUPPORT_INTERFACE__LIBRARY_NAME(
      rosidl_typesupport_introspection_cpp, test_msgs));
  static constexpr const char * identifier =
    "rosidl_typesupport_introspection_cpp";

  static constexpr const char * messages_namespace = "test_msgs::msg";
  static constexpr const char * services_namespace = "test_msgs::srv";
  static constexpr const char * actions_namespace = "test_msgs::action";

  static constexpr const MessageTypeSupportSymbolRecord messages[] = {
    MESSAGE_TYPESUPPORT_SYMBOL_RECORD(
      rosidl_typesupport_introspection_cpp, test_msgs, msg, Arrays),
    MESSAGE_TYPESUPPORT_SYMBOL_RECORD(
      rosidl_typesupport_introspection_cpp, test_msgs, msg, BasicTypes),
    MESSAGE_TYPESUPPORT_SYMBOL_RECORD(
      rosidl_typesupport_introspection_cpp, test_msgs, msg, Constants),
    MESSAGE_TYPESUPPORT_SYMBOL_RECORD(
      rosidl_typesupport_introspection_cpp, test_msgs, msg, Defaults),
    MESSAGE_TYPESUPPORT_SYMBOL_RECORD(
      rosidl_typesupport_introspection_cpp, test_msgs, msg, Empty)
  };
  static constexpr const ServiceTypeSupportSymbolRecord services[] = {
    SERVICE_TYPESUPPORT_SYMBOL_RECORD(
      rosidl_typesupport_introspection_cpp, test_msgs, srv, BasicTypes)
  };
  static constexpr const ActionTypeSupportSymbolRecord actions[] = {
    ACTION_TYPESUPPORT_SYMBOL_RECORD(
      rosidl_typesupport_introspection_cpp, test_msgs, action, Fibonacci)
  };
};

// Traits to aid introspection of `test_msgs` package interfaces in C++
template<>
struct introspection_traits<test_msgs::msg::Arrays>
{
  static constexpr const MessageTypeSupportSymbolRecord typesupport =
    MESSAGE_TYPESUPPORT_SYMBOL_RECORD(
    rosidl_typesupport_introspection_cpp, test_msgs, msg, Arrays);
  using TypeSupportLibraryT = TestMsgsIntrospectionCppTypeSupportLibrary;
};

template<>
struct introspection_traits<test_msgs::msg::BasicTypes>
{
  static constexpr const MessageTypeSupportSymbolRecord typesupport =
    MESSAGE_TYPESUPPORT_SYMBOL_RECORD(
    rosidl_typesupport_introspection_cpp, test_msgs, msg, BasicTypes);
  using TypeSupportLibraryT = TestMsgsIntrospectionCppTypeSupportLibrary;
};

template<>
struct introspection_traits<test_msgs::msg::Constants>
{
  static constexpr const MessageTypeSupportSymbolRecord typesupport =
    MESSAGE_TYPESUPPORT_SYMBOL_RECORD(
    rosidl_typesupport_introspection_cpp, test_msgs, msg, Constants);
  using TypeSupportLibraryT = TestMsgsIntrospectionCppTypeSupportLibrary;
};

template<>
struct introspection_traits<test_msgs::msg::Defaults>
{
  static constexpr const MessageTypeSupportSymbolRecord typesupport =
    MESSAGE_TYPESUPPORT_SYMBOL_RECORD(
    rosidl_typesupport_introspection_cpp, test_msgs, msg, Defaults);
  using TypeSupportLibraryT = TestMsgsIntrospectionCppTypeSupportLibrary;
};

template<>
struct introspection_traits<test_msgs::msg::Empty>
{
  static constexpr const MessageTypeSupportSymbolRecord typesupport =
    MESSAGE_TYPESUPPORT_SYMBOL_RECORD(
    rosidl_typesupport_introspection_cpp, test_msgs, msg, Empty);
  using TypeSupportLibraryT = TestMsgsIntrospectionCppTypeSupportLibrary;
};

// Examples of `test_msgs` package interfaces in C++, useful in test fixtures
template<>
struct Example<test_msgs::msg::Arrays>
{
  static std::unique_ptr<test_msgs::msg::Arrays> Make()
  {
    auto message = std::make_unique<test_msgs::msg::Arrays>();
    message->bool_values[2] = true;
    message->uint16_values[0] = 1234u;
    return message;
  }
};

}  // namespace rosidl_typesupport_introspection_tests

// Extra C++ APIs to homogeneize access to `test_msgs` package
// interfaces in C and C++
DEFINE_CXX_API_FOR_C_MESSAGE_MEMBER(rosidl_runtime_c__String)
DEFINE_CXX_API_FOR_C_MESSAGE(test_msgs, msg, Arrays)
DEFINE_CXX_API_FOR_C_MESSAGE(test_msgs, msg, BasicTypes)
DEFINE_CXX_API_FOR_C_MESSAGE(test_msgs, msg, Constants)
DEFINE_CXX_API_FOR_C_MESSAGE(test_msgs, msg, Defaults)
DEFINE_CXX_API_FOR_C_MESSAGE(test_msgs, msg, Empty)

#endif  // TEST_MSGS_INTROSPECTION__TEST_MSGS_TYPES_HPP_
