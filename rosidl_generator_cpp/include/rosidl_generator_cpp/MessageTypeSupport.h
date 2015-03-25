/* Copyright 2015 Open Source Robotics Foundation, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __rosidl_generator_cpp__MessageTypeSupport__h__
#define __rosidl_generator_cpp__MessageTypeSupport__h__

#include <rosidl_generator_c/message_type_support.h>
#include <rosidl_generator_c/visibility_control.h>

namespace rosidl_generator_cpp
{

template<typename T>
ROSIDL_PUBLIC
const rosidl_message_type_support_t * get_type_support_handle();

}  // namespace rosidl_generator_cpp

// Tail include the implementation for this function.
// This include is provided by the implementation and therefore only
// one version of this header will get included from the path.
// This header maps the get_type_support_handle function to an implementation
// specific version of that function.
#include <rosidl_generator_cpp/get_type_support_handle_impl.hpp>

#endif  // __rosidl_generator_cpp__MessageTypeSupport__h__
