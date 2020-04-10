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

#endif  // ROSIDL_GENERATOR_CPP__MESSAGE_INITIALIZATION_HPP_
