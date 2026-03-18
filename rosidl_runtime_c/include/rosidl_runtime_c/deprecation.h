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

#ifndef ROSIDL_RUNTIME_C__DEPRECATION_H_
#define ROSIDL_RUNTIME_C__DEPRECATION_H_

#if defined(_MSC_VER)
  #define DISABLE_DEPRECATED_PUSH __pragma(warning(push)) \
                                   __pragma(warning(disable: 4996))
  #define DISABLE_DEPRECATED_POP  __pragma(warning(pop))
#elif defined(__clang__) || defined(__GNUC__)
  #define DISABLE_DEPRECATED_PUSH _Pragma("GCC diagnostic push") \
                                   _Pragma("GCC diagnostic ignored \"-Wdeprecated-declarations\"")
  #define DISABLE_DEPRECATED_POP  _Pragma("GCC diagnostic pop")
#else
  #define DISABLE_DEPRECATED_PUSH
  #define DISABLE_DEPRECATED_POP
#endif

#endif  // ROSIDL_RUNTIME_C__DEPRECATION_H_
