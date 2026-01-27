// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// Copyright 2019 Open Source Robotics Foundation, Inc.
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
#include <string>
#include <tuple>

// the idl file is commented out in the test_interface_files package
// #include "rosidl_generator_tests/idl/idl_only_types.hpp"
#include "rosidl_generator_tests/msg/defaults.hpp"
#include "rosidl_generator_tests/msg/empty.hpp"
#include "rosidl_generator_tests/msg/bounded_sequences.hpp"
#include "rosidl_generator_tests/msg/nested.hpp"
#include "rosidl_generator_tests/msg/strings.hpp"
#include "rosidl_generator_tests/msg/w_strings.hpp"
#include "rosidl_generator_tests/srv/empty.hpp"

using rosidl_generator_traits::name;
using rosidl_generator_traits::data_type;
using rosidl_generator_traits::is_message;
using rosidl_generator_traits::is_service;
using rosidl_generator_traits::is_service_request;
using rosidl_generator_traits::is_service_response;
using rosidl_generator_traits::MessageTraits;
using rosidl_generator_tests::msg::to_yaml;

TEST(Test_rosidl_generator_traits, to_yaml_default_style) {
  {
    const rosidl_generator_tests::msg::Empty msg;
    EXPECT_STREQ("null\n", to_yaml(msg).c_str());
  }

  {
    rosidl_generator_tests::msg::Defaults msg;
    msg.float64_value = 1.0;
    EXPECT_STREQ(
      R"(bool_value: true
byte_value: 0x32
char_value: 100
float32_value: 1.12500
float64_value: 1.00000
int8_value: -50
uint8_value: 200
int16_value: -1000
uint16_value: 2000
int32_value: -30000
uint32_value: 60000
int64_value: -40000000
uint64_value: 50000000
)",
      to_yaml(
        msg).c_str());
  }

  {
    rosidl_generator_tests::msg::Strings msg;
    msg.string_value = "Hello\nworld";
    EXPECT_STREQ(
      R"(string_value: "Hello
world"
string_value_default1: "Hello world!"
string_value_default2: "Hello'world!"
string_value_default3: "Hello\"world!"
string_value_default4: "Hello'world!"
string_value_default5: "Hello\"world!"
bounded_string_value: ""
bounded_string_value_default1: "Hello world!"
bounded_string_value_default2: "Hello'world!"
bounded_string_value_default3: "Hello\"world!"
bounded_string_value_default4: "Hello'world!"
bounded_string_value_default5: "Hello\"world!"
)",
      to_yaml(
        msg).c_str());
  }

  {
    rosidl_generator_tests::msg::WStrings msg;
    msg.wstring_value = u"Hello\nwörld";
    EXPECT_STREQ(
      R"(wstring_value: "Hello
w\xf6rld"
wstring_value_default1: "Hello world!"
wstring_value_default2: "Hell\xf6 w\xf6rld!"
wstring_value_default3: "\u30cf\u30ed\u30fc\u30ef\u30fc\u30eb\u30c9"
array_of_wstrings:
- ""
- ""
- ""
bounded_sequence_of_wstrings: []
unbounded_sequence_of_wstrings: []
)",
      to_yaml(
        msg).c_str());
  }

  /*{
    test_msgs::idl::IdlOnlyTypes msg;
    msg.wchar_value = u'ö';
    msg.long_double_value = 1.125;
    EXPECT_STREQ(
      R"(wchar_value: "\u00f6"
long_double_value: 1.12500
)",
      to_yaml(
        msg).c_str());

    msg.wchar_value = u'貓';
    EXPECT_STREQ(
      R"(wchar_value: "\u8c93"
long_double_value: 1.12500
)",
      to_yaml(
        msg).c_str());
  }*/

  {
    rosidl_generator_tests::msg::Nested msg;
    std::string yaml = to_yaml(msg);
#ifdef _WIN32
    // update yaml to handle variance of floating point decimals on Windows
    size_t index = 0;
    while ((index = yaml.find("0.000000", index)) != std::string::npos) {
      yaml = yaml.replace(index, 8, "0.00000");
    }
#endif
    EXPECT_STREQ(
      R"(basic_types_value:
  bool_value: false
  byte_value: 0x00
  char_value: 0
  float32_value: 0.00000
  float64_value: 0.00000
  int8_value: 0
  uint8_value: 0
  int16_value: 0
  uint16_value: 0
  int32_value: 0
  uint32_value: 0
  int64_value: 0
  uint64_value: 0
)",
      yaml.c_str());
  }

  {
    rosidl_generator_tests::msg::BoundedSequences msg;
    msg.defaults_values.push_back(rosidl_generator_tests::msg::Defaults());
    std::string yaml = to_yaml(msg);
#ifdef _WIN32
    // update yaml to handle variance of floating point decimals on Windows
    size_t index = 0;
    while ((index = yaml.find("0.000000", index)) != std::string::npos) {
      yaml = yaml.replace(index, 8, "0.00000");
    }
#endif
    EXPECT_STREQ(
      R"(bool_values: []
byte_values: []
char_values: []
float32_values: []
float64_values: []
int8_values: []
uint8_values: []
int16_values: []
uint16_values: []
int32_values: []
uint32_values: []
int64_values: []
uint64_values: []
string_values: []
basic_types_values: []
constants_values: []
defaults_values:
-
  bool_value: true
  byte_value: 0x32
  char_value: 100
  float32_value: 1.12500
  float64_value: 1.12500
  int8_value: -50
  uint8_value: 200
  int16_value: -1000
  uint16_value: 2000
  int32_value: -30000
  uint32_value: 60000
  int64_value: -40000000
  uint64_value: 50000000
bool_values_default:
- false
- true
- false
byte_values_default:
- 0x00
- 0x01
- 0xff
char_values_default:
- 0
- 1
- 127
float32_values_default:
- 1.12500
- 0.00000
- -1.12500
float64_values_default:
- 3.14150
- 0.00000
- -3.14150
int8_values_default:
- 0
- 127
- -128
uint8_values_default:
- 0
- 1
- 255
int16_values_default:
- 0
- 32767
- -32768
uint16_values_default:
- 0
- 1
- 65535
int32_values_default:
- 0
- 2147483647
- -2147483648
uint32_values_default:
- 0
- 1
- 4294967295
int64_values_default:
- 0
- 9223372036854775807
- -9223372036854775808
uint64_values_default:
- 0
- 1
- 18446744073709551615
string_values_default:
- ""
- "max value"
- "min value"
alignment_check: 0
)",
      yaml.c_str());
  }
}

TEST(Test_rosidl_generator_traits, to_yaml_flow_style) {
  constexpr bool use_flow_style = true;
  {
    const rosidl_generator_tests::msg::Empty msg;
    EXPECT_STREQ("null", to_yaml(msg, use_flow_style).c_str());
  }

  {
    rosidl_generator_tests::msg::Defaults msg;
    msg.float64_value = 1.0;
    EXPECT_STREQ(
      "{bool_value: true, byte_value: 0x32, char_value: 100, "
      "float32_value: 1.12500, float64_value: 1.00000, int8_value: -50, "
      "uint8_value: 200, int16_value: -1000, uint16_value: 2000, "
      "int32_value: -30000, uint32_value: 60000, int64_value: -40000000, "
      "uint64_value: 50000000}",
      to_yaml(msg, use_flow_style).c_str());
  }

  {
    rosidl_generator_tests::msg::Strings msg;
    msg.string_value = "Hello\nworld";
    EXPECT_STREQ(
      R"({string_value: "Hello
world", string_value_default1: "Hello world!", )"
      R"(string_value_default2: "Hello'world!", )"
      R"(string_value_default3: "Hello\"world!", )"
      R"(string_value_default4: "Hello'world!", )"
      R"(string_value_default5: "Hello\"world!", )"
      R"(bounded_string_value: "", )"
      R"(bounded_string_value_default1: "Hello world!", )"
      R"(bounded_string_value_default2: "Hello'world!", )"
      R"(bounded_string_value_default3: "Hello\"world!", )"
      R"(bounded_string_value_default4: "Hello'world!", )"
      R"(bounded_string_value_default5: "Hello\"world!"})",
      to_yaml(msg, use_flow_style).c_str());
  }

  {
    rosidl_generator_tests::msg::WStrings msg;
    msg.wstring_value = u"Hello\nwörld";
    EXPECT_STREQ(
      R"({wstring_value: "Hello
w\xf6rld", wstring_value_default1: "Hello world!", )"
      R"(wstring_value_default2: "Hell\xf6 w\xf6rld!", )"
      R"(wstring_value_default3: "\u30cf\u30ed\u30fc\u30ef\u30fc\u30eb\u30c9", )"
      R"(array_of_wstrings: ["", "", ""], bounded_sequence_of_wstrings: [], )"
      R"(unbounded_sequence_of_wstrings: []})",
      to_yaml(msg, use_flow_style).c_str());
  }

  {
    rosidl_generator_tests::msg::Nested msg;
    std::string yaml = to_yaml(msg, use_flow_style);
#ifdef _WIN32
    // update yaml to handle variance of floating point decimals on Windows
    size_t index = 0;
    while ((index = yaml.find("0.000000", index)) != std::string::npos) {
      yaml = yaml.replace(index, 8, "0.00000");
    }
#endif
    EXPECT_STREQ(
      R"({basic_types_value: {bool_value: false, byte_value: 0x00, )"
      R"(char_value: 0, float32_value: 0.00000, float64_value: 0.00000, )"
      R"(int8_value: 0, uint8_value: 0, int16_value: 0, uint16_value: 0, )"
      R"(int32_value: 0, uint32_value: 0, int64_value: 0, uint64_value: 0}})",
      yaml.c_str());
  }

  {
    rosidl_generator_tests::msg::BoundedSequences msg;
    msg.defaults_values.push_back(rosidl_generator_tests::msg::Defaults());
    std::string yaml = to_yaml(msg, use_flow_style);
#ifdef _WIN32
    // update yaml to handle variance of floating point decimals on Windows
    size_t index = 0;
    while ((index = yaml.find("0.000000", index)) != std::string::npos) {
      yaml = yaml.replace(index, 8, "0.00000");
    }
#endif
    EXPECT_STREQ(
      R"({bool_values: [], byte_values: [], char_values: [], )"
      R"(float32_values: [], float64_values: [], int8_values: [], )"
      R"(uint8_values: [], int16_values: [], uint16_values: [], )"
      R"(int32_values: [], uint32_values: [], int64_values: [], )"
      R"(uint64_values: [], string_values: [], basic_types_values: [], )"
      R"(constants_values: [], defaults_values: [{bool_value: true, )"
      R"(byte_value: 0x32, char_value: 100, float32_value: 1.12500, )"
      R"(float64_value: 1.12500, int8_value: -50, uint8_value: 200, )"
      R"(int16_value: -1000, uint16_value: 2000, int32_value: -30000, )"
      R"(uint32_value: 60000, int64_value: -40000000, uint64_value: 50000000}],)"
      R"( bool_values_default: [false, true, false], )"
      R"(byte_values_default: [0x00, 0x01, 0xff], )"
      R"(char_values_default: [0, 1, 127], float32_values_default: [1.12500, )"
      R"(0.00000, -1.12500], float64_values_default: [3.14150, 0.00000, )"
      R"(-3.14150], int8_values_default: [0, 127, -128], )"
      R"(uint8_values_default: [0, 1, 255], int16_values_default: )"
      R"([0, 32767, -32768], uint16_values_default: [0, 1, 65535], )"
      R"(int32_values_default: [0, 2147483647, -2147483648], )"
      R"(uint32_values_default: [0, 1, 4294967295], )"
      R"(int64_values_default: [0, 9223372036854775807, -9223372036854775808],)"
      R"( uint64_values_default: [0, 1, 18446744073709551615], )"
      R"(string_values_default: ["", "max value", "min value"], )"
      R"(alignment_check: 0})",
      yaml.c_str());
  }
}


// Empty testing struct
struct Message {};

// Empty testing struct, with template instantiation
struct Message2 {};

namespace rosidl_generator_traits
{

template<>
struct is_message<Message2>: std::true_type {};

}  // namespace rosidl_generator_traits

TEST(Test_rosidl_generator_traits, is_message) {
  // A message is not a service
  using Empty = rosidl_generator_tests::msg::Empty;
  EXPECT_TRUE(is_message<Empty>());
  EXPECT_FALSE(is_service<Empty>());
  EXPECT_FALSE(is_service_request<Empty>());
  EXPECT_FALSE(is_service_response<Empty>());

  // A message is not a service
  using Strings = rosidl_generator_tests::msg::Strings;
  EXPECT_TRUE(is_message<Strings>());
  EXPECT_FALSE(is_service<Strings>());
  EXPECT_FALSE(is_service_request<Strings>());
  EXPECT_FALSE(is_service_response<Strings>());

  // Other datatypes should have is_message == false
  EXPECT_FALSE(is_message<double>());
  EXPECT_FALSE(is_message<Message>());

  // Unless the template has been specifically instantiated for the type
  EXPECT_TRUE(is_message<Message2>());
}

TEST(Test_rosidl_generator_traits, is_service) {
  using Service = rosidl_generator_tests::srv::Empty;
  using ServiceReq = Service::Request;
  using ServiceResp = Service::Response;
  using ServiceEvent = Service::Event;

  EXPECT_TRUE(is_service<Service>());
  EXPECT_FALSE(is_message<Service>());
  EXPECT_FALSE(is_service_request<Service>());
  EXPECT_FALSE(is_service_response<Service>());

  // Requests are additionally messages
  EXPECT_FALSE(is_service<ServiceReq>());
  EXPECT_TRUE(is_message<ServiceReq>());
  EXPECT_TRUE(is_service_request<ServiceReq>());
  EXPECT_FALSE(is_service_response<ServiceReq>());

  // Responses are additionally messages
  EXPECT_FALSE(is_service<ServiceResp>());
  EXPECT_TRUE(is_message<ServiceResp>());
  EXPECT_FALSE(is_service_request<ServiceResp>());
  EXPECT_TRUE(is_service_response<ServiceResp>());

  // Events are additionally messages
  EXPECT_FALSE(is_service<ServiceEvent>());
  EXPECT_TRUE(is_message<ServiceEvent>());
  EXPECT_FALSE(is_service_request<ServiceEvent>());
  EXPECT_FALSE(is_service_response<ServiceEvent>());
}

constexpr bool streq(const char * a, const char * b)
{
  while (*a && (*a == *b)) {
    ++a;
    ++b;
  }
  return *a == *b;
}

static_assert(streq(name<rosidl_generator_tests::srv::Empty>(),
"rosidl_generator_tests/srv/Empty"));
static_assert(streq(data_type<rosidl_generator_tests::srv::Empty>(),
"rosidl_generator_tests::srv::Empty"));

// Compile-time tests for member count and member names in MessageTraits
using MessageTraitsDefaults = MessageTraits<rosidl_generator_tests::msg::Defaults>;
static_assert(MessageTraitsDefaults::member_count == 13,
              "Unexpected number of members for Defaults message");
static_assert(MessageTraitsDefaults::member_names[0] == "bool_value",
              "Unexpected member name for Defaults::bool_value");
static_assert(MessageTraitsDefaults::member_names[1] == "byte_value",
              "Unexpected member name for Defaults::byte_value");
static_assert(MessageTraitsDefaults::member_names[2] == "char_value",
              "Unexpected member name for Defaults::char_value");
static_assert(MessageTraitsDefaults::member_names[3] == "float32_value",
              "Unexpected member name for Defaults::float32_value");
static_assert(MessageTraitsDefaults::member_names[4] == "float64_value",
              "Unexpected member name for Defaults::float64_value");
static_assert(MessageTraitsDefaults::member_names[5] == "int8_value",
              "Unexpected member name for Defaults::int8_value");
static_assert(MessageTraitsDefaults::member_names[6] == "uint8_value",
              "Unexpected member name for Defaults::uint8_value");
static_assert(MessageTraitsDefaults::member_names[7] == "int16_value",
              "Unexpected member name for Defaults::int16_value");
static_assert(MessageTraitsDefaults::member_names[8] == "uint16_value",
              "Unexpected member name for Defaults::uint16_value");
static_assert(MessageTraitsDefaults::member_names[9] == "int32_value",
              "Unexpected member name for Defaults::int32_value");
static_assert(MessageTraitsDefaults::member_names[10] == "uint32_value",
              "Unexpected member name for Defaults::uint32_value");
static_assert(MessageTraitsDefaults::member_names[11] == "int64_value",
              "Unexpected member name for Defaults::int64_value");
static_assert(MessageTraitsDefaults::member_names[12] == "uint64_value",
              "Unexpected member name for Defaults::uint64_value");

TEST(Test_rosidl_generator_traits, structured_binding_support)
{
  rosidl_generator_tests::msg::Defaults msg;
  auto [bool_value, byte_value, char_value, float32_value, float64_value, int8_value, uint8_value,
    int16_value, uint16_value, int32_value, uint32_value, int64_value, uint64_value] = msg;

  ASSERT_TRUE(bool_value);
  ASSERT_EQ(50, byte_value);
  ASSERT_EQ(100, char_value);
  ASSERT_EQ(1.125f, float32_value);
  ASSERT_EQ(1.125, float64_value);
  ASSERT_EQ(-50, int8_value);
  ASSERT_EQ(200, uint8_value);
  ASSERT_EQ(-1000, int16_value);
  ASSERT_EQ(2000, uint16_value);
  ASSERT_EQ(-30000L, int32_value);
  ASSERT_EQ(60000UL, uint32_value);
  ASSERT_EQ(-40000000LL, int64_value);
  ASSERT_EQ(50000000ULL, uint64_value);

  // Structured binding without & are copies and should not be modifiable
  bool_value = false;
  ASSERT_TRUE(msg.bool_value);

  // Test with references
  auto & [bool_ref, byte_ref, char_ref, float32_ref, float64_ref, int8_ref, uint8_ref, int16_ref,
    uint16_ref, int32_ref, uint32_ref, int64_ref, uint64_ref] = msg;

  // Now the structural binding returns modifiable references
  bool_ref = false;
  ASSERT_FALSE(msg.bool_value);
}

TEST(Test_rosidl_generator_traits, as_tuple_ref)
{
  rosidl_generator_tests::msg::Defaults msg;

  // Check initial value
  ASSERT_EQ(-1000, msg.int16_value);

  // Default-initialize all fields via tuple reference
  std::apply([&msg](auto & ... field) {
      ((field = {}), ...);
    }, as_tuple_ref(msg));

  // Check that field has been modified
  ASSERT_EQ(0, msg.int16_value);
}
