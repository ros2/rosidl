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

#ifndef __rosidl_typesupport_introspection_cpp__impl__visibility_control__h__
#define __rosidl_typesupport_introspection_cpp__impl__visibility_control__h__

#if __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROSIDL_TSI_CPP_DLLIMPORT __attribute__ ((dllimport))
  #else
    #define ROSIDL_TSI_CPP_DLLIMPORT __declspec(dllimport)
  #endif
  #ifdef ROSIDL_TSI_CPP_BUILDING_DLL
    #ifdef __GNUC__
      #define ROSIDL_TSI_CPP_PUBLIC __attribute__ ((dllexport))
    #else
      #define ROSIDL_TSI_CPP_PUBLIC __declspec(dllexport)
    #endif
  #else
    #ifdef __GNUC__
      #define ROSIDL_TSI_CPP_PUBLIC __attribute__ ((dllimport))
    #else
      #define ROSIDL_TSI_CPP_PUBLIC __declspec(dllimport)
    #endif
  #endif
  #define ROSIDL_TSI_CPP_LOCAL
#else
  #if __GNUC__ >= 4
    #define ROSIDL_TSI_CPP_PUBLIC __attribute__ ((visibility ("default")))
    #define ROSIDL_TSI_CPP_LOCAL __attribute__ ((visibility ("hidden")))
  #else
    #define ROSIDL_TSI_CPP_PUBLIC
    #define ROSIDL_TSI_CPP_LOCAL
  #endif
#endif

#if __cplusplus
}
#endif

#endif  // __rosidl_typesupport_introspection_cpp__impl__visibility_control__h__
