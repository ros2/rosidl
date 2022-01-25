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

#ifndef ROSIDL_TYPESUPPORT_INTROSPECTION_TESTS__API_HPP_
#define ROSIDL_TYPESUPPORT_INTROSPECTION_TESTS__API_HPP_

#include <rosidl_runtime_c/message_initialization.h>
#include <rosidl_typesupport_introspection_c/message_introspection.h>

#include <rcpputils/shared_library.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>

#include "rosidl_typesupport_introspection_tests/type_traits.hpp"
#include "rosidl_typesupport_introspection_tests/types.hpp"

namespace rosidl_typesupport_introspection_tests
{

void * initialize_message(
  void * message,
  const rosidl_typesupport_introspection_cpp::MessageMembers * message_descriptor)
{
  using rosidl_runtime_cpp::MessageInitialization;
  message_descriptor->init_function(message, MessageInitialization::ALL);
  return message;
}

void * initialize_message(
  void * message,
  const rosidl_typesupport_introspection_c__MessageMembers * message_descriptor)
{
  message_descriptor->init_function(message, ROSIDL_RUNTIME_C_MSG_INIT_ALL);
  return message;
}

template<typename MessageDescriptorT>
void finalize_message(
  void * message,
  const MessageDescriptorT * message_descriptor)
{
  message_descriptor->fini_function(message);
}

template<typename MessageDescriptorT>
const char *
get_message_namespace(const MessageDescriptorT * message_descriptor)
{
  return message_descriptor->message_namespace_;
}

template<typename MessageDescriptorT>
const char * get_message_name(const MessageDescriptorT * message_descriptor)
{
  return message_descriptor->message_name_;
}

template<typename MessageDescriptorT>
size_t get_message_size(const MessageDescriptorT * message_descriptor)
{
  return message_descriptor->size_of_;
}

template<typename MessageDescriptorT>
size_t get_member_count(const MessageDescriptorT * message_descriptor)
{
  return message_descriptor->member_count_;
}

template<typename MemberDescriptorT>
void * get_member(void * message, const MemberDescriptorT * member_descriptor)
{
  return reinterpret_cast<uint8_t *>(message) +
         member_descriptor->offset_;
}

template<typename MemberT, typename MemberDescriptorT>
MemberT &
get_member(void * message, const MemberDescriptorT * member_descriptor)
{
  return *reinterpret_cast<MemberT *>(get_member(message, member_descriptor));
}

template<typename MessageDescriptorT>
auto get_member_descriptor(
  const MessageDescriptorT * message_descriptor,
  const size_t index)
{
  return &(message_descriptor->members_[index]);
}

template<typename MemberDescriptorT>
bool has_simple_structure(const MemberDescriptorT * member_descriptor)
{
  return !member_descriptor->is_array_ &&
         member_descriptor->size_function == nullptr &&
         member_descriptor->get_function == nullptr &&
         member_descriptor->get_const_function == nullptr &&
         member_descriptor->fetch_function == nullptr &&
         member_descriptor->assign_function == nullptr &&
         member_descriptor->resize_function == nullptr;
}

template<typename MemberDescriptorT>
bool has_iterable_structure(const MemberDescriptorT * member_descriptor)
{
  return member_descriptor->is_array_ &&
         member_descriptor->size_function != nullptr &&
         member_descriptor->fetch_function != nullptr &&
         member_descriptor->assign_function != nullptr;
}

template<typename MemberDescriptorT>
bool has_array_structure(const MemberDescriptorT * member_descriptor)
{
  return has_iterable_structure(member_descriptor) &&
         member_descriptor->get_function != nullptr &&
         member_descriptor->get_const_function != nullptr &&
         member_descriptor->resize_function == nullptr;
}

template<typename MemberDescriptorT>
bool has_array_structure(
  const MemberDescriptorT * member_descriptor, const size_t size)
{
  return has_array_structure(member_descriptor) &&
         member_descriptor->array_size_ == size;
}

template<typename MemberDescriptorT>
bool has_sequence_structure(const MemberDescriptorT * member_descriptor)
{
  return has_iterable_structure(member_descriptor) &&
         !member_descriptor->is_upper_bound_ &&
         member_descriptor->resize_function != nullptr;
}

template<typename MessageT, typename MemberDescriptorT>
bool is_message_type_member(const MemberDescriptorT * member_descriptor)
{
  using TypeSupportLibraryT =
    typename introspection_traits<MessageT>::TypeSupportLibraryT;
  const auto & typesupport = introspection_traits<MessageT>::typesupport;
  rcpputils::SharedLibrary library{
    rcpputils::get_platform_library_name(TypeSupportLibraryT::name)};
  auto message_typesupport_fetch =
    reinterpret_cast<MessageTypeSupportFetchFunctionT>(
    library.get_symbol(typesupport.symbol));
  if (!message_typesupport_fetch) {
    return false;
  }
  const rosidl_message_type_support_t * message_typesupport =
    get_message_typesupport_handle(
    message_typesupport_fetch(),
    TypeSupportLibraryT::identifier);
  return member_descriptor->type_id_ == ROS_TYPE_MESSAGE &&
         member_descriptor->members_ == message_typesupport;
}

template<typename MemberDescriptorT>
bool is_base_type_member(const MemberDescriptorT * member_descriptor, int type_id)
{
  return member_descriptor->type_id_ == type_id &&
         member_descriptor->members_ == nullptr;
}

template<typename MemberDescriptorT>
bool is_string_member(const MemberDescriptorT * member_descriptor, const size_t upper_bound = 0u)
{
  return member_descriptor->type_id_ == ROS_TYPE_STRING &&
         member_descriptor->members_ == nullptr &&
         member_descriptor->string_upper_bound_ == upper_bound;
}

template<typename MemberDescriptorT>
const void * get_const_member(
  const void * message,
  const MemberDescriptorT * member_descriptor)
{
  return reinterpret_cast<const uint8_t *>(message) +
         member_descriptor->offset_;
}

template<typename MemberT, typename MemberDescriptorT>
const MemberT & get_const_member(
  const void * message,
  const MemberDescriptorT * member_descriptor)
{
  return *reinterpret_cast<const MemberT *>(
    get_const_member(message, member_descriptor));
}

template<typename MemberDescriptorT>
const char * get_member_name(const MemberDescriptorT * member_descriptor)
{
  return member_descriptor->name_;
}

template<typename MemberDescriptorT>
int get_member_base_type(const MemberDescriptorT * member_descriptor)
{
  return member_descriptor->type_id_;
}

template<typename ItemT, typename MemberDescriptorT>
ItemT fetch_member_item(
  const void * member,
  const MemberDescriptorT * member_descriptor,
  const size_t index)
{
  ItemT value;
  member_descriptor->fetch_function(member, index, &value);
  return value;
}

template<typename ItemT, typename MemberDescriptorT>
void assign_member_item(
  void * member,
  const MemberDescriptorT * member_descriptor,
  const size_t index, ItemT && value)
{
  member_descriptor->assign_function(member, index, &value);
}

template<typename MemberDescriptorT>
size_t get_member_size(
  const void * member,
  const MemberDescriptorT * member_descriptor)
{
  return member_descriptor->size_function(member);
}

template<typename MemberDescriptorT>
void resize_member(
  void * member,
  const MemberDescriptorT * member_descriptor,
  const size_t size)
{
  member_descriptor->resize_function(member, size);
}


}  // namespace rosidl_typesupport_introspection_tests

#endif  // ROSIDL_TYPESUPPORT_INTROSPECTION_TESTS__API_HPP_
