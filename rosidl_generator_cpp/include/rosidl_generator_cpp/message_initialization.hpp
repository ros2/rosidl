// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#ifndef ROSIDL_GENERATOR_CPP__MESSAGE_INITIALIZATION_HPP_
#define ROSIDL_GENERATOR_CPP__MESSAGE_INITIALIZATION_HPP_

// TODO(clalancette): this should be moved out into a separate runtime package

#include <rosidl_generator_c/message_initialization.h>

namespace rosidl_generator_cpp
{

enum class MessageInitialization
{
  ALL = ROSIDL_RUNTIME_C_MSG_INIT_ALL,
  SKIP = ROSIDL_RUNTIME_C_MSG_INIT_SKIP,
  ZERO = ROSIDL_RUNTIME_C_MSG_INIT_ZERO,
  DEFAULTS_ONLY = ROSIDL_RUNTIME_C_MSG_INIT_DEFAULTS_ONLY,
};

}  // namespace rosidl_generator_cpp

// Macros for implementing the rigorous builder pattern
#define ROSIDL_GENERATOR_CPP__INIT_FIRST_FIELD( \
    MessageName, field_name, next_field_name) \
  struct Init__ ## field_name \
  { \
    Init__ ## field_name() : _msg(rosidl_generator_cpp::MessageInitialization::SKIP) {} \
 \
    Init__ ## next_field_name field_name(_ ## field_name ## _type _arg) \
    { \
      _msg.field_name = std::move(_arg); \
      return Init__ ## next_field_name(_msg); \
    } \
 \
private: \
    MessageName _msg; \
  }

#define ROSIDL_GENERATOR_CPP__INIT_MID_FIELD( \
    MessageName, field_name, next_field_name) \
  struct Init__ ## field_name \
  { \
    Init__ ## field_name(MessageName & msg) : _msg(msg) {} \
 \
    Init__ ## next_field_name field_name(_ ## field_name ## _type _arg) \
    { \
      _msg.field_name = std::move(_arg); \
      return Init__ ## next_field_name(_msg); \
    } \
 \
private: \
    MessageName & _msg; \
  }

#define ROSIDL_GENERATOR_CPP__INIT_LAST_FIELD( \
    MessageName, field_name) \
  struct Init__ ## field_name \
  { \
    Init__ ## field_name(MessageName & msg) : _msg(msg) {} \
 \
    MessageName field_name(_ ## field_name ## _type _arg) \
    { \
      _msg.field_name = std::move(_arg); \
      return std::move(_msg); \
    } \
 \
private: \
    MessageName & _msg; \
  }

#define ROSIDL_GENERATOR_CPP__INIT_ONLY_FIELD( \
    MessageName, field_name) \
  struct Init__ ## field_name \
  { \
    MessageName field_name(_ ## field_name ## _type _arg) \
    { \
      MessageName _msg(rosidl_generator_cpp::MessageInitialization::SKIP); \
      _msg.field_name = std::move(_arg); \
      return std::move(_msg); \
    } \
  }

#endif  // ROSIDL_GENERATOR_CPP__MESSAGE_INITIALIZATION_HPP_
