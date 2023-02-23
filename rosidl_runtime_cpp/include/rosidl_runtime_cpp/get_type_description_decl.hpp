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

#ifndef ROSIDL_RUNTIME_CPP__GET_TYPE_DESCRIPTION_DECL_HPP_
#define ROSIDL_RUNTIME_CPP__GET_TYPE_DESCRIPTION_DECL_HPP_

#include <rosidl_runtime_cpp/type_description/type_description__struct.hpp>

namespace rosidl_runtime_cpp
{

/// Get the description of an interface type (msg/srv/action).
/**
 * Note: this is implemented by generated sources of interface packages.
 * \return A const reference to the static type description of the interface.
 */
template<typename T>
const rosidl_runtime_cpp::type_description::TypeDescription & get_type_description();

}  // namespace rosidl_runtime_cpp

#endif  // ROSIDL_RUNTIME_CPP__GET_TYPE_DESCRIPTION_DECL_HPP_
