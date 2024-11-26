// DO NOT EDIT MANUALLY - this copied file managed by copy_type_description_generated_sources.bash
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from type_description_interfaces:msg/IndividualTypeDescription.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "type_description_interfaces/msg/individual_type_description.hpp"


#ifndef ROSIDL_RUNTIME_CPP__TYPE_DESCRIPTION__INDIVIDUAL_TYPE_DESCRIPTION__STRUCT_HPP_
#define ROSIDL_RUNTIME_CPP__TYPE_DESCRIPTION__INDIVIDUAL_TYPE_DESCRIPTION__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'fields'
#include "rosidl_runtime_cpp/type_description/field__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__rosidl_runtime_cpp__type_description__IndividualTypeDescription __attribute__((deprecated))
#else
# define DEPRECATED__rosidl_runtime_cpp__type_description__IndividualTypeDescription __declspec(deprecated)
#endif

namespace rosidl_runtime_cpp
{

namespace type_description
{

// message struct
template<class ContainerAllocator>
struct IndividualTypeDescription_
{
  using Type = IndividualTypeDescription_<ContainerAllocator>;

  explicit IndividualTypeDescription_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->type_name = "";
    }
  }

  explicit IndividualTypeDescription_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : type_name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->type_name = "";
    }
  }

  // field types and members
  using _type_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _type_name_type type_name;
  using _fields_type =
    std::vector<rosidl_runtime_cpp::type_description::Field_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<rosidl_runtime_cpp::type_description::Field_<ContainerAllocator>>>;
  _fields_type fields;

  // setters for named parameter idiom
  Type & set__type_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->type_name = _arg;
    return *this;
  }
  Type & set__fields(
    const std::vector<rosidl_runtime_cpp::type_description::Field_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<rosidl_runtime_cpp::type_description::Field_<ContainerAllocator>>> & _arg)
  {
    this->fields = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rosidl_runtime_cpp::type_description::IndividualTypeDescription_<ContainerAllocator> *;
  using ConstRawPtr =
    const rosidl_runtime_cpp::type_description::IndividualTypeDescription_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rosidl_runtime_cpp::type_description::IndividualTypeDescription_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rosidl_runtime_cpp::type_description::IndividualTypeDescription_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rosidl_runtime_cpp::type_description::IndividualTypeDescription_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rosidl_runtime_cpp::type_description::IndividualTypeDescription_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rosidl_runtime_cpp::type_description::IndividualTypeDescription_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rosidl_runtime_cpp::type_description::IndividualTypeDescription_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rosidl_runtime_cpp::type_description::IndividualTypeDescription_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rosidl_runtime_cpp::type_description::IndividualTypeDescription_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rosidl_runtime_cpp__type_description__IndividualTypeDescription
    std::shared_ptr<rosidl_runtime_cpp::type_description::IndividualTypeDescription_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rosidl_runtime_cpp__type_description__IndividualTypeDescription
    std::shared_ptr<rosidl_runtime_cpp::type_description::IndividualTypeDescription_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const IndividualTypeDescription_ & other) const
  {
    if (this->type_name != other.type_name) {
      return false;
    }
    if (this->fields != other.fields) {
      return false;
    }
    return true;
  }
  bool operator!=(const IndividualTypeDescription_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct IndividualTypeDescription_

// alias to use template instance with default allocator
using IndividualTypeDescription =
  rosidl_runtime_cpp::type_description::IndividualTypeDescription_<std::allocator<void>>;

// constant definitions

}  // namespace type_description

}  // namespace rosidl_runtime_cpp

#endif  // ROSIDL_RUNTIME_CPP__TYPE_DESCRIPTION__INDIVIDUAL_TYPE_DESCRIPTION__STRUCT_HPP_
