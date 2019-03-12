// Copyright 2018 Open Source Robotics Foundation, Inc.
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

#ifndef ROSIDL_GENERATOR_CPP__TRAITS_HPP_
#define ROSIDL_GENERATOR_CPP__TRAITS_HPP_

#include <type_traits>

// TODO(dirk-thomas) this should be in the rosidl_generator_cpp namespace
namespace rosidl_generator_traits
{

template<typename T>
inline const char * data_type();

template<typename T>
struct has_fixed_size : std::false_type {};

template<typename T>
struct has_bounded_size : std::false_type {};

}  // namespace rosidl_generator_traits

#endif  // ROSIDL_GENERATOR_CPP__TRAITS_HPP_
