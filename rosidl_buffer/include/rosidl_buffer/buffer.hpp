// Copyright 2026 Open Source Robotics Foundation, Inc.
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

#ifndef ROSIDL_BUFFER__BUFFER_HPP_
#define ROSIDL_BUFFER__BUFFER_HPP_

#include <algorithm>
#include <cstddef>
#include <initializer_list>
#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "rosidl_buffer/buffer_impl_base.hpp"
#include "rosidl_buffer/cpu_buffer_impl.hpp"

namespace rosidl
{

/// Buffer<T> provides a drop-in replacement for std::vector<T> with support
/// for vendor-specific memory backends (CPU, GPU, etc.).
///
/// For CPU backends, it provides implicit conversion to std::vector<T>&
/// for seamless backward compatibility. For non-CPU backends, direct access
/// throws exceptions, requiring explicit conversion via to_vector().
template<typename T, typename Allocator = std::allocator<T>>
class Buffer
{
public:
  // Type aliases for std::vector compatibility
  using value_type = T;
  using allocator_type = Allocator;
  using size_type = size_t;
  using difference_type = std::ptrdiff_t;
  using reference = T &;
  using const_reference = const T &;
  using pointer = T *;
  using const_pointer = const T *;
  using iterator = T *;
  using const_iterator = const T *;

  /// Default constructor creates CPU buffer
  Buffer()
  : impl_(std::make_unique<CpuBufferImpl<T>>())
  {
  }

  /// Construct with a backend implementation (set once at construction).
  /// This is the only way to set a non-CPU backend; there is no post-
  /// construction setter, which avoids race conditions with concurrent reads.
  explicit Buffer(std::unique_ptr<BufferImplBase<T>> impl)
  : impl_(std::move(impl))
  {
    if (!impl_) {
      throw std::invalid_argument("Buffer implementation must not be null");
    }
  }

  /// Construct with initial size (CPU backend)
  explicit Buffer(size_t count)
  : impl_(std::make_unique<CpuBufferImpl<T>>())
  {
    get_cpu_impl()->get_storage().resize(count);
  }

  /// Construct with initial size and value (CPU backend)
  Buffer(size_t count, const T & value)
  : impl_(std::make_unique<CpuBufferImpl<T>>())
  {
    get_cpu_impl()->get_storage().assign(count, value);
  }

  /// Construct from std::vector (copy) - for backward compatibility
  Buffer(const std::vector<T> & vec)  // NOLINT(runtime/explicit) - intentionally implicit
  : impl_(std::make_unique<CpuBufferImpl<T>>())
  {
    get_cpu_impl()->get_storage() = vec;
  }

  /// Construct from std::vector (move) - for backward compatibility
  Buffer(std::vector<T> && vec)  // NOLINT(runtime/explicit) - intentionally implicit
  : impl_(std::make_unique<CpuBufferImpl<T>>())
  {
    get_cpu_impl()->get_storage() = std::move(vec);
  }

  /// Construct from initializer list - for backward compatibility
  Buffer(std::initializer_list<T> init)
  : impl_(std::make_unique<CpuBufferImpl<T>>())
  {
    get_cpu_impl()->get_storage() = init;
  }

  /// Copy constructor (deep copy via clone())
  Buffer(const Buffer & other)
  : impl_(other.impl_ ? other.impl_->clone() : nullptr)
  {
  }

  /// Move constructor
  Buffer(Buffer && other) noexcept
  : impl_(std::move(other.impl_))
  {
  }

  /// Copy assignment (deep copy via clone())
  Buffer & operator=(const Buffer & other)
  {
    if (this != &other) {
      impl_ = other.impl_ ? other.impl_->clone() : nullptr;
    }
    return *this;
  }

  /// Move assignment
  Buffer & operator=(Buffer && other) noexcept
  {
    if (this != &other) {
      impl_ = std::move(other.impl_);
    }
    return *this;
  }

  /// Assignment from initializer list - for backward compatibility
  /// This must come before vector assignment to resolve ambiguity with {{...}} syntax
  Buffer & operator=(std::initializer_list<T> init)
  {
    impl_ = std::make_unique<CpuBufferImpl<T>>();
    get_cpu_impl()->get_storage() = init;
    return *this;
  }

  /// Assignment from std::vector (copy) - for backward compatibility
  /// Uses SFINAE to avoid ambiguity with initializer lists
  template<typename U = std::vector<T>,
    typename std::enable_if<std::is_same<U, std::vector<T>>::value, int>::type = 0>
  Buffer & operator=(const U & vec)
  {
    impl_ = std::make_unique<CpuBufferImpl<T>>();
    get_cpu_impl()->get_storage() = vec;
    return *this;
  }

  /// Assignment from std::vector (move) - for backward compatibility
  /// Uses SFINAE to avoid ambiguity with initializer lists
  template<typename U = std::vector<T>,
    typename std::enable_if<std::is_same<U, std::vector<T>>::value, int>::type = 0>
  Buffer & operator=(U && vec)
  {
    impl_ = std::make_unique<CpuBufferImpl<T>>();
    get_cpu_impl()->get_storage() = std::move(vec);
    return *this;
  }

  // ========== Element Access (CPU only) ==========

  /// Access element at position (CPU only)
  reference operator[](size_t pos)
  {
    throw_if_not_cpu_backend();
    return get_cpu_impl()->get_storage()[pos];
  }

  const_reference operator[](size_t pos) const
  {
    throw_if_not_cpu_backend();
    return get_cpu_impl()->get_storage()[pos];
  }

  /// Access element with bounds checking (CPU only)
  reference at(size_t pos)
  {
    throw_if_not_cpu_backend();
    return get_cpu_impl()->get_storage().at(pos);
  }

  const_reference at(size_t pos) const
  {
    throw_if_not_cpu_backend();
    return get_cpu_impl()->get_storage().at(pos);
  }

  /// Access first element (CPU only)
  reference front()
  {
    throw_if_not_cpu_backend();
    return get_cpu_impl()->get_storage().front();
  }

  const_reference front() const
  {
    throw_if_not_cpu_backend();
    return get_cpu_impl()->get_storage().front();
  }

  /// Access last element (CPU only)
  reference back()
  {
    throw_if_not_cpu_backend();
    return get_cpu_impl()->get_storage().back();
  }

  const_reference back() const
  {
    throw_if_not_cpu_backend();
    return get_cpu_impl()->get_storage().back();
  }

  /// Get pointer to data (CPU only)
  pointer data()
  {
    throw_if_not_cpu_backend();
    return get_cpu_impl()->get_storage().data();
  }

  const_pointer data() const
  {
    throw_if_not_cpu_backend();
    return get_cpu_impl()->get_storage().data();
  }

  // ========== Iterators (CPU only) ==========

  iterator begin()
  {
    throw_if_not_cpu_backend();
    return get_cpu_impl()->get_storage().data();
  }

  const_iterator begin() const
  {
    throw_if_not_cpu_backend();
    return get_cpu_impl()->get_storage().data();
  }

  const_iterator cbegin() const
  {
    throw_if_not_cpu_backend();
    return get_cpu_impl()->get_storage().data();
  }

  iterator end()
  {
    throw_if_not_cpu_backend();
    auto & s = get_cpu_impl()->get_storage();
    return s.data() + s.size();
  }

  const_iterator end() const
  {
    throw_if_not_cpu_backend();
    auto & s = get_cpu_impl()->get_storage();
    return s.data() + s.size();
  }

  const_iterator cend() const
  {
    throw_if_not_cpu_backend();
    auto & s = get_cpu_impl()->get_storage();
    return s.data() + s.size();
  }

  // ========== Capacity ==========

  /// Works for all backends (delegates to BufferImplBase::size()).
  bool empty() const {return !impl_ || impl_->size() == 0;}

  /// Works for all backends (delegates to BufferImplBase::size()).
  size_t size() const {return impl_ ? impl_->size() : 0;}

  void reserve(size_t new_cap)
  {
    throw_if_not_cpu_backend();
    get_cpu_impl()->get_storage().reserve(new_cap);
  }

  size_t capacity() const
  {
    throw_if_not_cpu_backend();
    return get_cpu_impl()->get_storage().capacity();
  }

  void shrink_to_fit()
  {
    throw_if_not_cpu_backend();
    get_cpu_impl()->get_storage().shrink_to_fit();
  }

  // ========== Modifiers (CPU only) ==========

  void assign(size_t count, const T & value)
  {
    throw_if_not_cpu_backend();
    get_cpu_impl()->get_storage().assign(count, value);
  }

  template<typename InputIt>
  void assign(InputIt first, InputIt last)
  {
    throw_if_not_cpu_backend();
    get_cpu_impl()->get_storage().assign(first, last);
  }

  void assign(std::initializer_list<T> ilist)
  {
    throw_if_not_cpu_backend();
    get_cpu_impl()->get_storage().assign(ilist);
  }

  iterator insert(const_iterator pos, const T & value)
  {
    throw_if_not_cpu_backend();
    auto & s = get_cpu_impl()->get_storage();
    auto offset = pos - s.data();
    auto it = s.insert(s.begin() + offset, value);
    return s.data() + (it - s.begin());
  }

  iterator insert(const_iterator pos, T && value)
  {
    throw_if_not_cpu_backend();
    auto & s = get_cpu_impl()->get_storage();
    auto offset = pos - s.data();
    auto it = s.insert(s.begin() + offset, std::move(value));
    return s.data() + (it - s.begin());
  }

  iterator insert(const_iterator pos, size_t count, const T & value)
  {
    throw_if_not_cpu_backend();
    auto & s = get_cpu_impl()->get_storage();
    auto offset = pos - s.data();
    auto it = s.insert(s.begin() + offset, count, value);
    return s.data() + (it - s.begin());
  }

  template<typename InputIt>
  iterator insert(const_iterator pos, InputIt first, InputIt last)
  {
    throw_if_not_cpu_backend();
    auto & s = get_cpu_impl()->get_storage();
    auto offset = pos - s.data();
    auto it = s.insert(s.begin() + offset, first, last);
    return s.data() + (it - s.begin());
  }

  iterator insert(const_iterator pos, std::initializer_list<T> ilist)
  {
    throw_if_not_cpu_backend();
    auto & s = get_cpu_impl()->get_storage();
    auto offset = pos - s.data();
    auto it = s.insert(s.begin() + offset, ilist);
    return s.data() + (it - s.begin());
  }

  void clear()
  {
    throw_if_not_cpu_backend();
    get_cpu_impl()->get_storage().clear();
  }

  void resize(size_t n)
  {
    throw_if_not_cpu_backend();
    get_cpu_impl()->get_storage().resize(n);
  }

  void resize(size_t n, const T & value)
  {
    throw_if_not_cpu_backend();
    get_cpu_impl()->get_storage().resize(n, value);
  }

  void push_back(const T & value)
  {
    throw_if_not_cpu_backend();
    get_cpu_impl()->get_storage().push_back(value);
  }

  void push_back(T && value)
  {
    throw_if_not_cpu_backend();
    get_cpu_impl()->get_storage().push_back(std::move(value));
  }

  void pop_back()
  {
    throw_if_not_cpu_backend();
    get_cpu_impl()->get_storage().pop_back();
  }

  template<typename ... Args>
  void emplace_back(Args && ... args)
  {
    throw_if_not_cpu_backend();
    get_cpu_impl()->get_storage().emplace_back(std::forward<Args>(args)...);
  }

  // ========== Conversion Operators ==========

  /// Implicit conversion to std::vector<T>& (CPU only).
  /// Provides backward compatibility with existing code.
  /// @throws std::runtime_error if backend is not CPU.
  operator std::vector<T> &()
  {
    throw_if_not_cpu_backend();
    return get_cpu_impl()->get_storage();
  }

  operator const std::vector<T> &() const
  {
    throw_if_not_cpu_backend();
    return get_cpu_impl()->get_storage();
  }

  /// Escape hatch: Explicit conversion to std::vector<T> (all backends).
  /// For non-CPU backends, this triggers a copy to CPU memory.
  /// @return A std::vector containing a copy of the buffer data.
  std::vector<T> to_vector() const
  {
    if (get_backend_type() == "cpu") {
      return get_cpu_impl()->get_storage();
    } else {
      auto cpu_impl_ptr = impl_->to_cpu();
      auto * cpu_impl = static_cast<CpuBufferImpl<T> *>(cpu_impl_ptr.get());
      return cpu_impl->get_storage();
    }
  }

  // ========== Comparison Operators ==========

  bool operator==(const Buffer & other) const
  {
    if (size() != other.size()) {
      return false;
    }
    if (get_backend_type() == "cpu" && other.get_backend_type() == "cpu") {
      return get_cpu_impl()->get_storage() == other.get_cpu_impl()->get_storage();
    }
    return to_vector() == other.to_vector();
  }

  bool operator!=(const Buffer & other) const
  {
    return !(*this == other);
  }

  // ========== Backend Management ==========

  /// Get the backend type identifier (e.g., "cpu", "cuda").
  /// Delegates to the underlying implementation — the impl is the single
  /// source of truth for its own backend type.
  std::string get_backend_type() const
  {
    return impl_ ? impl_->get_backend_type() : "cpu";
  }

  /// Get the implementation pointer (for serialization) - returns raw pointer
  /// for read-only access
  const BufferImplBase<T> * get_impl() const {return impl_.get();}

  /// Throw exception if not CPU backend.
  /// @throws std::runtime_error if backend is not CPU.
  void throw_if_not_cpu_backend() const
  {
    const std::string bt = get_backend_type();
    if (bt != "cpu") {
      throw std::runtime_error(
              "Operation requires CPU backend. Current backend: " + bt +
              ". Use to_vector() for explicit conversion to CPU.");
    }
  }

private:
  /// Unique pointer for proper ownership and value semantics.
  /// The implementation is the sole source of truth for the backend type.
  std::unique_ptr<BufferImplBase<T>> impl_;

  /// Get CPU implementation (assumes throw_if_not_cpu_backend() was called)
  CpuBufferImpl<T> * get_cpu_impl() const
  {
    return static_cast<CpuBufferImpl<T> *>(impl_.get());
  }
};

/// Free-function comparison: std::vector<T> == Buffer<T>
/// Needed because template argument deduction does not consider implicit conversions.
template<typename T, typename Allocator>
bool operator==(const std::vector<T, Allocator> & lhs, const Buffer<T, Allocator> & rhs)
{
  if (rhs.get_backend_type() == "cpu") {
    return lhs == static_cast<const std::vector<T, Allocator> &>(rhs);
  }
  return lhs == rhs.to_vector();
}

template<typename T, typename Allocator>
bool operator==(const Buffer<T, Allocator> & lhs, const std::vector<T, Allocator> & rhs)
{
  return rhs == lhs;
}

template<typename T, typename Allocator>
bool operator!=(const std::vector<T, Allocator> & lhs, const Buffer<T, Allocator> & rhs)
{
  return !(lhs == rhs);
}

template<typename T, typename Allocator>
bool operator!=(const Buffer<T, Allocator> & lhs, const std::vector<T, Allocator> & rhs)
{
  return !(lhs == rhs);
}

}  // namespace rosidl

#endif  // ROSIDL_BUFFER__BUFFER_HPP_
