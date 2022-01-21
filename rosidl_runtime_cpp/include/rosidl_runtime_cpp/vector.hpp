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

#ifndef ROSIDL_RUNTIME_CPP__VECTOR_HPP_
#define ROSIDL_RUNTIME_CPP__VECTOR_HPP_

#include <memory>
#include <type_traits>
#include <vector>

namespace rosidl_runtime_cpp
{
namespace internal
{
/// A bool value wrapper to avoid std::vector<bool> specializations.
/**
 * For all practical purposes, an instance of this class is a bool value.
 */
class BoolWrapper
{
public:
  constexpr BoolWrapper()
  : value_(false) {}
  constexpr BoolWrapper(bool value)
  : value_(value) {}

  BoolWrapper & operator=(bool value)
  {
    value_ = value;
    return *this;
  }

  bool operator==(BoolWrapper other)
  {
    return value_ == other.value_;
  }

  bool operator!=(BoolWrapper other)
  {
    return value_ != other.value_;
  }

  operator bool &() {return value_;}

  operator const bool &() const {return value_;}

  bool * operator&() {return &value_;}  // NOLINT(runtime/operator)

private:
  bool value_;
};

static_assert(
  std::is_standard_layout<BoolWrapper>::value,
  "BoolWrapper must be a standard layout type "
  "to afford reinterpret casts to bool.");

}  // namespace internal

/// A container based on std::vector with its own specializations.
/**
 * This container transparently interacts with std::vector<T> instances.
 */
template<typename T, typename AllocatorT = std::allocator<T>>
class Vector : public std::vector<T, AllocatorT>
{
  using Base = std::vector<T, AllocatorT>;

public:
  using typename Base::value_type;
  using typename Base::pointer;
  using typename Base::const_pointer;
  using typename Base::reference;
  using typename Base::const_reference;
  using typename Base::iterator;
  using typename Base::const_iterator;
  using typename Base::const_reverse_iterator;
  using typename Base::reverse_iterator;
  using typename Base::size_type;
  using typename Base::difference_type;
  using typename Base::allocator_type;

  using Base::vector;

  Vector() = default;

  // Implicit "copy" and "move" constructors from std::vector<T> instances
  Vector(const std::vector<T, AllocatorT> & other)  // NOLINT(runtime/explicit)
  : Base(other) {}

  Vector(std::vector<T, AllocatorT> && other)  // NOLINT(runtime/explicit)
  : Base(other) {}
};

template<typename T, typename AllocatorT>
inline void
swap(Vector<T, AllocatorT> & x, Vector<T, AllocatorT> & y)
{
  x.swap(y);
}

/// Specialization for a boolean vector with no space optimizations.
/**
 * This container transparently interacts with std::vector<bool>
 * instances while avoiding all the quirks in the latter.
 */
template<typename AllocatorT>
class Vector<bool, AllocatorT>: public std::vector<
    internal::BoolWrapper,
    typename std::allocator_traits<AllocatorT>::
    template rebind_alloc<internal::BoolWrapper>>
{
  using Base = std::vector<
    internal::BoolWrapper,
    typename std::allocator_traits<AllocatorT>::
    template rebind_alloc<internal::BoolWrapper>>;

public:
  using value_type = bool;
  using reference = bool &;
  using const_reference = const bool &;
  using pointer = bool *;
  using const_pointer = const bool *;
  using typename Base::iterator;
  using typename Base::const_iterator;
  using typename Base::reverse_iterator;
  using typename Base::const_reverse_iterator;
  using typename Base::size_type;
  using typename Base::difference_type;
  using typename Base::allocator_type;

  using Base::vector;

  Vector() = default;

  // Implicit "copy" and "move" constructors from std::vector<bool> instances
  Vector(const std::vector<bool, AllocatorT> & other)  // NOLINT(runtime/explicit)
  : Base(other.begin(), other.end())
  {
  }

  Vector(std::vector<bool, AllocatorT> && other)  // NOLINT(runtime/explicit)
  : Base(other.begin(), other.end())
  {
  }

  // Constructor for list initialization with bool values
  Vector(
    std::initializer_list<bool> list,
    const AllocatorT & allocator = AllocatorT())
  : Base(list.begin(), list.end(), allocator)
  {
  }

  // Assignment operator using list initializers with bool values
  Vector & operator=(std::initializer_list<bool> list)
  {
    this->assign(list.begin(), list.end());
    return *this;
  }

  // Implicit conversion to std::vector<bool>
  operator std::vector<bool, AllocatorT>() const {
    return std::vector<bool, AllocatorT>{
      this->begin(), this->end(),
      this->get_allocator()};
  }
};

// Equality operators to compare Vector<bool> and std::vector<bool>
template<typename AllocatorT>
inline bool
operator==(
  const Vector<bool, AllocatorT> & a,
  const std::vector<bool, AllocatorT> & b)
{
  if (a.size() != b.size()) {
    return false;
  }
  for (size_t i = 0; i < a.size(); ++i) {
    if (a[i] != b[i]) {return false;}
  }
  return true;
}

template<typename AllocatorT>
inline bool
operator==(
  const std::vector<bool, AllocatorT> & a,
  const Vector<bool, AllocatorT> & b)
{
  return b == a;
}

// Inequality operators to compare Vector<bool> and std::vector<bool>
template<typename AllocatorT>
inline bool
operator!=(
  const Vector<bool, AllocatorT> & a,
  const std::vector<bool, AllocatorT> & b)
{
  return !(a == b);
}

template<typename AllocatorT>
inline bool
operator!=(
  const std::vector<bool, AllocatorT> & a,
  const Vector<bool, AllocatorT> & b)
{
  return !(b == a);
}

}  // namespace rosidl_runtime_cpp

#endif  // ROSIDL_RUNTIME_CPP__VECTOR_HPP_
