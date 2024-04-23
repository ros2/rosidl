// DO NOT EDIT MANUALLY - this copied file managed by copy_type_description_generated_sources.bash
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from type_description_interfaces:msg/FieldType.idl
// generated code does not contain a copyright notice

#ifndef ROSIDL_RUNTIME_CPP__TYPE_DESCRIPTION__FIELD_TYPE__STRUCT_HPP_
#define ROSIDL_RUNTIME_CPP__TYPE_DESCRIPTION__FIELD_TYPE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__rosidl_runtime_cpp__type_description__FieldType __attribute__((deprecated))
#else
# define DEPRECATED__rosidl_runtime_cpp__type_description__FieldType __declspec(deprecated)
#endif

namespace rosidl_runtime_cpp
{

namespace type_description
{

// message struct
template<class ContainerAllocator>
struct FieldType_
{
  using Type = FieldType_<ContainerAllocator>;

  explicit FieldType_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->type_id = 0;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->type_id = 0;
      this->capacity = 0ull;
      this->string_capacity = 0ull;
      this->nested_type_name = "";
    }
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->capacity = 0ull;
      this->string_capacity = 0ull;
      this->nested_type_name = "";
    }
  }

  explicit FieldType_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : nested_type_name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->type_id = 0;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->type_id = 0;
      this->capacity = 0ull;
      this->string_capacity = 0ull;
      this->nested_type_name = "";
    }
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->capacity = 0ull;
      this->string_capacity = 0ull;
      this->nested_type_name = "";
    }
  }

  // field types and members
  using _type_id_type =
    uint8_t;
  _type_id_type type_id;
  using _capacity_type =
    uint64_t;
  _capacity_type capacity;
  using _string_capacity_type =
    uint64_t;
  _string_capacity_type string_capacity;
  using _nested_type_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _nested_type_name_type nested_type_name;

  // setters for named parameter idiom
  Type & set__type_id(
    const uint8_t & _arg)
  {
    this->type_id = _arg;
    return *this;
  }
  Type & set__capacity(
    const uint64_t & _arg)
  {
    this->capacity = _arg;
    return *this;
  }
  Type & set__string_capacity(
    const uint64_t & _arg)
  {
    this->string_capacity = _arg;
    return *this;
  }
  Type & set__nested_type_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->nested_type_name = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t FIELD_TYPE_NOT_SET =
    0u;
  static constexpr uint8_t FIELD_TYPE_NESTED_TYPE =
    1u;
  static constexpr uint8_t FIELD_TYPE_INT8 =
    2u;
  static constexpr uint8_t FIELD_TYPE_UINT8 =
    3u;
  static constexpr uint8_t FIELD_TYPE_INT16 =
    4u;
  static constexpr uint8_t FIELD_TYPE_UINT16 =
    5u;
  static constexpr uint8_t FIELD_TYPE_INT32 =
    6u;
  static constexpr uint8_t FIELD_TYPE_UINT32 =
    7u;
  static constexpr uint8_t FIELD_TYPE_INT64 =
    8u;
  static constexpr uint8_t FIELD_TYPE_UINT64 =
    9u;
  static constexpr uint8_t FIELD_TYPE_FLOAT =
    10u;
  static constexpr uint8_t FIELD_TYPE_DOUBLE =
    11u;
  static constexpr uint8_t FIELD_TYPE_LONG_DOUBLE =
    12u;
  static constexpr uint8_t FIELD_TYPE_CHAR =
    13u;
  static constexpr uint8_t FIELD_TYPE_WCHAR =
    14u;
  static constexpr uint8_t FIELD_TYPE_BOOLEAN =
    15u;
  static constexpr uint8_t FIELD_TYPE_BYTE =
    16u;
  static constexpr uint8_t FIELD_TYPE_STRING =
    17u;
  static constexpr uint8_t FIELD_TYPE_WSTRING =
    18u;
  static constexpr uint8_t FIELD_TYPE_FIXED_STRING =
    19u;
  static constexpr uint8_t FIELD_TYPE_FIXED_WSTRING =
    20u;
  static constexpr uint8_t FIELD_TYPE_BOUNDED_STRING =
    21u;
  static constexpr uint8_t FIELD_TYPE_BOUNDED_WSTRING =
    22u;
  static constexpr uint8_t FIELD_TYPE_NESTED_TYPE_ARRAY =
    49u;
  static constexpr uint8_t FIELD_TYPE_INT8_ARRAY =
    50u;
  static constexpr uint8_t FIELD_TYPE_UINT8_ARRAY =
    51u;
  static constexpr uint8_t FIELD_TYPE_INT16_ARRAY =
    52u;
  static constexpr uint8_t FIELD_TYPE_UINT16_ARRAY =
    53u;
  static constexpr uint8_t FIELD_TYPE_INT32_ARRAY =
    54u;
  static constexpr uint8_t FIELD_TYPE_UINT32_ARRAY =
    55u;
  static constexpr uint8_t FIELD_TYPE_INT64_ARRAY =
    56u;
  static constexpr uint8_t FIELD_TYPE_UINT64_ARRAY =
    57u;
  static constexpr uint8_t FIELD_TYPE_FLOAT_ARRAY =
    58u;
  static constexpr uint8_t FIELD_TYPE_DOUBLE_ARRAY =
    59u;
  static constexpr uint8_t FIELD_TYPE_LONG_DOUBLE_ARRAY =
    60u;
  static constexpr uint8_t FIELD_TYPE_CHAR_ARRAY =
    61u;
  static constexpr uint8_t FIELD_TYPE_WCHAR_ARRAY =
    62u;
  static constexpr uint8_t FIELD_TYPE_BOOLEAN_ARRAY =
    63u;
  static constexpr uint8_t FIELD_TYPE_BYTE_ARRAY =
    64u;
  static constexpr uint8_t FIELD_TYPE_STRING_ARRAY =
    65u;
  static constexpr uint8_t FIELD_TYPE_WSTRING_ARRAY =
    66u;
  static constexpr uint8_t FIELD_TYPE_FIXED_STRING_ARRAY =
    67u;
  static constexpr uint8_t FIELD_TYPE_FIXED_WSTRING_ARRAY =
    68u;
  static constexpr uint8_t FIELD_TYPE_BOUNDED_STRING_ARRAY =
    69u;
  static constexpr uint8_t FIELD_TYPE_BOUNDED_WSTRING_ARRAY =
    70u;
  static constexpr uint8_t FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE =
    97u;
  static constexpr uint8_t FIELD_TYPE_INT8_BOUNDED_SEQUENCE =
    98u;
  static constexpr uint8_t FIELD_TYPE_UINT8_BOUNDED_SEQUENCE =
    99u;
  static constexpr uint8_t FIELD_TYPE_INT16_BOUNDED_SEQUENCE =
    100u;
  static constexpr uint8_t FIELD_TYPE_UINT16_BOUNDED_SEQUENCE =
    101u;
  static constexpr uint8_t FIELD_TYPE_INT32_BOUNDED_SEQUENCE =
    102u;
  static constexpr uint8_t FIELD_TYPE_UINT32_BOUNDED_SEQUENCE =
    103u;
  static constexpr uint8_t FIELD_TYPE_INT64_BOUNDED_SEQUENCE =
    104u;
  static constexpr uint8_t FIELD_TYPE_UINT64_BOUNDED_SEQUENCE =
    105u;
  static constexpr uint8_t FIELD_TYPE_FLOAT_BOUNDED_SEQUENCE =
    106u;
  static constexpr uint8_t FIELD_TYPE_DOUBLE_BOUNDED_SEQUENCE =
    107u;
  static constexpr uint8_t FIELD_TYPE_LONG_DOUBLE_BOUNDED_SEQUENCE =
    108u;
  static constexpr uint8_t FIELD_TYPE_CHAR_BOUNDED_SEQUENCE =
    109u;
  static constexpr uint8_t FIELD_TYPE_WCHAR_BOUNDED_SEQUENCE =
    110u;
  static constexpr uint8_t FIELD_TYPE_BOOLEAN_BOUNDED_SEQUENCE =
    111u;
  static constexpr uint8_t FIELD_TYPE_BYTE_BOUNDED_SEQUENCE =
    112u;
  static constexpr uint8_t FIELD_TYPE_STRING_BOUNDED_SEQUENCE =
    113u;
  static constexpr uint8_t FIELD_TYPE_WSTRING_BOUNDED_SEQUENCE =
    114u;
  static constexpr uint8_t FIELD_TYPE_FIXED_STRING_BOUNDED_SEQUENCE =
    115u;
  static constexpr uint8_t FIELD_TYPE_FIXED_WSTRING_BOUNDED_SEQUENCE =
    116u;
  static constexpr uint8_t FIELD_TYPE_BOUNDED_STRING_BOUNDED_SEQUENCE =
    117u;
  static constexpr uint8_t FIELD_TYPE_BOUNDED_WSTRING_BOUNDED_SEQUENCE =
    118u;
  static constexpr uint8_t FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE =
    145u;
  static constexpr uint8_t FIELD_TYPE_INT8_UNBOUNDED_SEQUENCE =
    146u;
  static constexpr uint8_t FIELD_TYPE_UINT8_UNBOUNDED_SEQUENCE =
    147u;
  static constexpr uint8_t FIELD_TYPE_INT16_UNBOUNDED_SEQUENCE =
    148u;
  static constexpr uint8_t FIELD_TYPE_UINT16_UNBOUNDED_SEQUENCE =
    149u;
  static constexpr uint8_t FIELD_TYPE_INT32_UNBOUNDED_SEQUENCE =
    150u;
  static constexpr uint8_t FIELD_TYPE_UINT32_UNBOUNDED_SEQUENCE =
    151u;
  static constexpr uint8_t FIELD_TYPE_INT64_UNBOUNDED_SEQUENCE =
    152u;
  static constexpr uint8_t FIELD_TYPE_UINT64_UNBOUNDED_SEQUENCE =
    153u;
  static constexpr uint8_t FIELD_TYPE_FLOAT_UNBOUNDED_SEQUENCE =
    154u;
  static constexpr uint8_t FIELD_TYPE_DOUBLE_UNBOUNDED_SEQUENCE =
    155u;
  static constexpr uint8_t FIELD_TYPE_LONG_DOUBLE_UNBOUNDED_SEQUENCE =
    156u;
  static constexpr uint8_t FIELD_TYPE_CHAR_UNBOUNDED_SEQUENCE =
    157u;
  static constexpr uint8_t FIELD_TYPE_WCHAR_UNBOUNDED_SEQUENCE =
    158u;
  static constexpr uint8_t FIELD_TYPE_BOOLEAN_UNBOUNDED_SEQUENCE =
    159u;
  static constexpr uint8_t FIELD_TYPE_BYTE_UNBOUNDED_SEQUENCE =
    160u;
  static constexpr uint8_t FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE =
    161u;
  static constexpr uint8_t FIELD_TYPE_WSTRING_UNBOUNDED_SEQUENCE =
    162u;
  static constexpr uint8_t FIELD_TYPE_FIXED_STRING_UNBOUNDED_SEQUENCE =
    163u;
  static constexpr uint8_t FIELD_TYPE_FIXED_WSTRING_UNBOUNDED_SEQUENCE =
    164u;
  static constexpr uint8_t FIELD_TYPE_BOUNDED_STRING_UNBOUNDED_SEQUENCE =
    165u;
  static constexpr uint8_t FIELD_TYPE_BOUNDED_WSTRING_UNBOUNDED_SEQUENCE =
    166u;

  // pointer types
  using RawPtr =
    rosidl_runtime_cpp::type_description::FieldType_<ContainerAllocator> *;
  using ConstRawPtr =
    const rosidl_runtime_cpp::type_description::FieldType_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rosidl_runtime_cpp::type_description::FieldType_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rosidl_runtime_cpp::type_description::FieldType_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rosidl_runtime_cpp::type_description::FieldType_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rosidl_runtime_cpp::type_description::FieldType_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rosidl_runtime_cpp::type_description::FieldType_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rosidl_runtime_cpp::type_description::FieldType_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rosidl_runtime_cpp::type_description::FieldType_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rosidl_runtime_cpp::type_description::FieldType_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rosidl_runtime_cpp__type_description__FieldType
    std::shared_ptr<rosidl_runtime_cpp::type_description::FieldType_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rosidl_runtime_cpp__type_description__FieldType
    std::shared_ptr<rosidl_runtime_cpp::type_description::FieldType_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FieldType_ & other) const
  {
    if (this->type_id != other.type_id) {
      return false;
    }
    if (this->capacity != other.capacity) {
      return false;
    }
    if (this->string_capacity != other.string_capacity) {
      return false;
    }
    if (this->nested_type_name != other.nested_type_name) {
      return false;
    }
    return true;
  }
  bool operator!=(const FieldType_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FieldType_

// alias to use template instance with default allocator
using FieldType =
  rosidl_runtime_cpp::type_description::FieldType_<std::allocator<void>>;

// constant definitions
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_NOT_SET;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_NESTED_TYPE;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_INT8;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_UINT8;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_INT16;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_UINT16;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_INT32;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_UINT32;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_INT64;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_UINT64;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_FLOAT;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_DOUBLE;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_LONG_DOUBLE;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_CHAR;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_WCHAR;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_BOOLEAN;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_BYTE;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_STRING;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_WSTRING;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_FIXED_STRING;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_FIXED_WSTRING;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_BOUNDED_STRING;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_BOUNDED_WSTRING;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_NESTED_TYPE_ARRAY;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_INT8_ARRAY;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_UINT8_ARRAY;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_INT16_ARRAY;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_UINT16_ARRAY;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_INT32_ARRAY;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_UINT32_ARRAY;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_INT64_ARRAY;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_UINT64_ARRAY;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_FLOAT_ARRAY;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_DOUBLE_ARRAY;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_LONG_DOUBLE_ARRAY;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_CHAR_ARRAY;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_WCHAR_ARRAY;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_BOOLEAN_ARRAY;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_BYTE_ARRAY;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_STRING_ARRAY;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_WSTRING_ARRAY;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_FIXED_STRING_ARRAY;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_FIXED_WSTRING_ARRAY;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_BOUNDED_STRING_ARRAY;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_BOUNDED_WSTRING_ARRAY;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_INT8_BOUNDED_SEQUENCE;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_UINT8_BOUNDED_SEQUENCE;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_INT16_BOUNDED_SEQUENCE;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_UINT16_BOUNDED_SEQUENCE;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_INT32_BOUNDED_SEQUENCE;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_UINT32_BOUNDED_SEQUENCE;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_INT64_BOUNDED_SEQUENCE;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_UINT64_BOUNDED_SEQUENCE;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_FLOAT_BOUNDED_SEQUENCE;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_DOUBLE_BOUNDED_SEQUENCE;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_LONG_DOUBLE_BOUNDED_SEQUENCE;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_CHAR_BOUNDED_SEQUENCE;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_WCHAR_BOUNDED_SEQUENCE;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_BOOLEAN_BOUNDED_SEQUENCE;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_BYTE_BOUNDED_SEQUENCE;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_STRING_BOUNDED_SEQUENCE;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_WSTRING_BOUNDED_SEQUENCE;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_FIXED_STRING_BOUNDED_SEQUENCE;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_FIXED_WSTRING_BOUNDED_SEQUENCE;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_BOUNDED_STRING_BOUNDED_SEQUENCE;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_BOUNDED_WSTRING_BOUNDED_SEQUENCE;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_INT8_UNBOUNDED_SEQUENCE;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_UINT8_UNBOUNDED_SEQUENCE;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_INT16_UNBOUNDED_SEQUENCE;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_UINT16_UNBOUNDED_SEQUENCE;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_INT32_UNBOUNDED_SEQUENCE;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_UINT32_UNBOUNDED_SEQUENCE;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_INT64_UNBOUNDED_SEQUENCE;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_UINT64_UNBOUNDED_SEQUENCE;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_FLOAT_UNBOUNDED_SEQUENCE;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_DOUBLE_UNBOUNDED_SEQUENCE;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_LONG_DOUBLE_UNBOUNDED_SEQUENCE;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_CHAR_UNBOUNDED_SEQUENCE;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_WCHAR_UNBOUNDED_SEQUENCE;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_BOOLEAN_UNBOUNDED_SEQUENCE;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_BYTE_UNBOUNDED_SEQUENCE;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_WSTRING_UNBOUNDED_SEQUENCE;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_FIXED_STRING_UNBOUNDED_SEQUENCE;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_FIXED_WSTRING_UNBOUNDED_SEQUENCE;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_BOUNDED_STRING_UNBOUNDED_SEQUENCE;
template<typename ContainerAllocator>
constexpr uint8_t FieldType_<ContainerAllocator>::FIELD_TYPE_BOUNDED_WSTRING_UNBOUNDED_SEQUENCE;

}  // namespace type_description

}  // namespace rosidl_runtime_cpp

#endif  // ROSIDL_RUNTIME_CPP__TYPE_DESCRIPTION__FIELD_TYPE__STRUCT_HPP_
