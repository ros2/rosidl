// Copyright 2025 Open Source Robotics Foundation, Inc.
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

#ifndef ROSIDL_RUNTIME_CPP__BYTE_CONTAINER_MIXIN_HPP_
#define ROSIDL_RUNTIME_CPP__BYTE_CONTAINER_MIXIN_HPP_

#include <algorithm>
#include <array>
#include <cstddef>
#include <type_traits>
#include <iterator>
#include <utility>
#include <vector>

#include "byte_conversion_helpers.hpp"
#include "deprecated_helper_byte.hpp"

namespace
{
template<typename Container, typename B>
struct rebind_container;

// std::vector specialization
template<typename T, typename Alloc, typename B>
struct rebind_container<std::vector<T, Alloc>, B>
{
  using type = std::vector<B, Alloc>;
};

// std::array specialization
template<typename T, std::size_t N, typename B>
struct rebind_container<std::array<T, N>, B>
{
  using type = std::array<B, N>;
};

template<typename Container>
constexpr auto & to_byte_container_ref(Container & c)
{
  using ByteContainer = typename rebind_container<Container, std::byte>::type;
  return reinterpret_cast<ByteContainer &>(c);
}

template<typename Container>
constexpr auto & to_uchar_container_ref(Container & c)
{
  using UCharContainer = typename rebind_container<typename Container::container_type,
      unsigned char>::type;
  return reinterpret_cast<UCharContainer &>(c);
}
}  // namespace


namespace rosidl_runtime_cpp
{

// Functions shared between std::array and std::vector
template<typename Container>
struct ByteContainerMixin : Container
{
  static_assert(std::is_same_v<typename Container::value_type, std::byte>,
                "ByteContainerMixin requires a container of std::byte");


  // Override Defaults to match old types
  using value_type = unsigned char;
  using reference = value_type &;
  using const_reference = const reference;
  using pointer = value_type *;
  using const_pointer = const pointer;
  using iterator = pointer;
  using const_iterator = const_pointer;
  using reverse_iterator = std::reverse_iterator<iterator>;
  using reverse_const_iterator = std::reverse_iterator<const_iterator>;

  using size_type = std::size_t;
  using difference_type = std::ptrdiff_t;

  using container_type = Container;
  using UCharContainer = typename rebind_container<Container, unsigned char>::type;

  // Member Functions
  // Constructor
  using Container::Container;

  [[deprecated("Use std::array<std::byte, N> instead of unsigned char array")]]
  constexpr ByteContainerMixin(const UCharContainer & arr) noexcept  // NOLINT(runtime/explicit)
  {
    for (std::size_t i = 0; i < Container::size(); ++i) {
      Container::at(i) = std::byte{arr[i]};
    }
  }

  [[deprecated("Use initializer_list of bytes instead of unsigned chars")]]
  constexpr ByteContainerMixin(
    std::initializer_list<unsigned char> init)  // NOLINT(runtime/explicit)
  {
    std::transform(init.begin(), init.end(), Container::begin(),
      [](unsigned char v) {return std::byte{v};});
  }

  constexpr ByteContainerMixin(
    std::initializer_list<std::byte> init)  // NOLINT(runtime/explicit)
  {
    std::copy(init.begin(), init.end(), Container::begin());
  }

  // Assignment
  using Container::operator=;

  [[deprecated("Use Container of std::byte instead")]]
  constexpr UCharContainer & operator=(const UCharContainer & arr) noexcept
  {
    for (std::size_t i = 0; i < Container::size(); ++i) {
      Container::operator[](i) = std::byte{arr[i]};
    }
    return to_uchar_container_ref(*this);
  }

  constexpr ByteContainerMixin & operator=(std::initializer_list<std::byte> init) noexcept
  {
    std::copy(init.begin(), init.end(), Container::begin());
    return *this;
  }

  [[deprecated("Use initializer_list<std::byte> instead of unsigned chars")]]
  constexpr ByteContainerMixin & operator=(std::initializer_list<unsigned char> init) noexcept
  {
    std::transform(init.begin(), init.end(), Container::begin(),
      [](unsigned char v) {return std::byte{v};});
    return *this;
  }

  // Implicitly-defined Member Functions

  // Element Access

  // at
  constexpr detail::ByteRef at(std::size_t pos)
  {
    return detail::ByteRef{Container::at(pos)};
  }

  constexpr DeprecatedHelperByte at(std::size_t pos) const
  {
    return DeprecatedHelperByte{Container::at(pos)};
  }

  // operator[]
  constexpr detail::ByteRef operator[](std::size_t pos)
  {
    return detail::ByteRef{Container::operator[](pos)};
  }

  constexpr DeprecatedHelperByte operator[](std::size_t pos) const
  {
    return DeprecatedHelperByte{Container::operator[](pos)};
  }

  // front
  constexpr detail::ByteRef front()
  {
    return detail::ByteRef{Container::front()};
  }

  constexpr DeprecatedHelperByte front() const
  {
    return DeprecatedHelperByte{Container::front()};
  }

  // back
  constexpr detail::ByteRef back()
  {
    return detail::ByteRef{Container::back()};
  }

  constexpr DeprecatedHelperByte back() const
  {
    return DeprecatedHelperByte{Container::back()};
  }

  // data()
  constexpr detail::BytePtr data()
  {
    return detail::BytePtr{Container::data()};
  }

  constexpr detail::ConstBytePtr data() const
  {
    return detail::ConstBytePtr{Container::data()};
  }

  // Iterators

  // begin, cbegin
  constexpr detail::BytePtr begin() noexcept
  {
    return detail::BytePtr{Container::begin()};
  }

  constexpr detail::ConstBytePtr begin() const noexcept
  {
    return detail::ConstBytePtr{Container::begin()};
  }

  constexpr detail::ConstBytePtr cbegin() const noexcept
  {
    return detail::ConstBytePtr{Container::cbegin()};
  }

  // end, cend
  constexpr detail::BytePtr end() noexcept
  {
    return detail::BytePtr{Container::end()};
  }

  constexpr detail::ConstBytePtr end() const noexcept
  {
    return detail::ConstBytePtr{Container::end()};
  }

  constexpr detail::ConstBytePtr cend() const noexcept
  {
    return detail::ConstBytePtr{Container::cend()};
  }

  // rbegin, crbegin
  constexpr detail::BytePtr rbegin() noexcept
  {
    return detail::BytePtr{Container::rbegin()};
  }

  constexpr detail::ConstBytePtr rbegin() const noexcept
  {
    return detail::ConstBytePtr{Container::rbegin()};
  }

  constexpr detail::ConstBytePtr crbegin() const noexcept
  {
    return detail::ConstBytePtr{Container::crbegin()};
  }

  // rend, crend
  constexpr detail::BytePtr rend() noexcept
  {
    return detail::BytePtr{Container::rend()};
  }

  constexpr detail::ConstBytePtr rend() const noexcept
  {
    return detail::ConstBytePtr{Container::rend()};
  }

  constexpr detail::ConstBytePtr crend() const noexcept
  {
    return detail::ConstBytePtr{Container::crend()};
  }

  // Getter
  [[deprecated("Convert to container of std::byte instead")]]
  constexpr operator UCharContainer &() noexcept
  {
    return to_uchar_container_ref(*this);
  }

  // Operations
  [[deprecated("Switch to calling swap with container of std::byte")]]
  constexpr void swap(UCharContainer & other)
  {
    Container::swap(to_byte_container_ref(other));
  }
};
}  // namespace rosidl_runtime_cpp

#endif  // ROSIDL_RUNTIME_CPP__BYTE_CONTAINER_MIXIN_HPP_
