// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#ifndef ROSIDL_GENERATOR_CPP__BOUNDED_VECTOR_HPP_
#define ROSIDL_GENERATOR_CPP__BOUNDED_VECTOR_HPP_

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

namespace rosidl_generator_cpp
{

/**
 *  @brief  A container based on std::vector but with an upper bound.
 *
 *  @tparam  _Tp  Type of element.
 *  @tparam  _UpperBound  The upper bound for the number of elements.
 *  @tparam  _Alloc  Allocator type, defaults to allocator<_Tp>.
 *
 *  Meets the same requirements as std::vector.
*/
template<typename _Tp, std::size_t _UpperBound, typename _Alloc = std::allocator<_Tp>>
class BoundedVector
  : protected std::vector<_Tp, _Alloc>
{
  using _Base = std::vector<_Tp, _Alloc>;

public:
  using typename _Base::value_type;
  using typename _Base::pointer;
  using typename _Base::const_pointer;
  using typename _Base::reference;
  using typename _Base::const_reference;
  using typename _Base::iterator;
  using typename _Base::const_iterator;
  using typename _Base::const_reverse_iterator;
  using typename _Base::reverse_iterator;
  using typename _Base::size_type;
  using typename _Base::difference_type;
  using typename _Base::allocator_type;

  /**
   *  @brief  Creates a %BoundedVector with no elements.
   */
  BoundedVector()
  noexcept(std::is_nothrow_default_constructible<_Alloc>::value)
  : _Base()
  {}

  /**
   *  @brief  Creates a %BoundedVector with no elements.
   *  @param  __a  An allocator object.
   */
  explicit
  BoundedVector(
    const typename _Base::allocator_type & __a)
  noexcept
  : _Base(__a)
  {}

  /**
   *  @brief  Creates a %BoundedVector with default constructed elements.
   *  @param  __n  The number of elements to initially create.
   *  @param  __a  An allocator.
   *
   *  This constructor fills the %BoundedVector with @a __n default
   *  constructed elements.
   */
  explicit
  BoundedVector(
    typename _Base::size_type __n,
    const typename _Base::allocator_type & __a = allocator_type())
  : _Base(__n, __a)
  {
    if (__n > _UpperBound) {
      throw std::length_error("Exceeded upper bound");
    }
  }

  /**
   *  @brief  Creates a %BoundedVector with copies of an exemplar element.
   *  @param  __n  The number of elements to initially create.
   *  @param  __value  An element to copy.
   *  @param  __a  An allocator.
   *
   *  This constructor fills the %BoundedVector with @a __n copies of @a __value.
   */
  BoundedVector(
    typename _Base::size_type __n,
    const typename _Base::value_type & __value,
    const typename _Base::allocator_type & __a = allocator_type())
  : _Base(__n, __value, __a)
  {
    if (__n > _UpperBound) {
      throw std::length_error("Exceeded upper bound");
    }
  }

  /**
   *  @brief  %BoundedVector copy constructor.
   *  @param  __x  A %BoundedVector of identical element and allocator types.
   *
   *  The newly-created %BoundedVector uses a copy of the allocation
   *  object used by @a __x.  All the elements of @a __x are copied,
   *  but any extra memory in
   *  @a __x (for fast expansion) will not be copied.
   */
  BoundedVector(
    const BoundedVector & __x)
  : _Base(__x)
  {}

  /**
   *  @brief  %BoundedVector move constructor.
   *  @param  __x  A %BoundedVector of identical element and allocator types.
   *
   *  The newly-created %BoundedVector contains the exact contents of @a __x.
   *  The contents of @a __x are a valid, but unspecified %BoundedVector.
   */
  BoundedVector(BoundedVector && __x) noexcept
  : _Base(std::move(__x))
  {}

  /// Copy constructor with alternative allocator
  BoundedVector(const BoundedVector & __x, const typename _Base::allocator_type & __a)
  : _Base(__x, __a)
  {}

  /**
   *  @brief  Builds a %BoundedVector from an initializer list.
   *  @param  __l  An initializer_list.
   *  @param  __a  An allocator.
   *
   *  Create a %BoundedVector consisting of copies of the elements in the
   *  initializer_list @a __l.
   *
   *  This will call the element type's copy constructor N times
   *  (where N is @a __l.size()) and do no memory reallocation.
   */
  BoundedVector(
    std::initializer_list<typename _Base::value_type> __l,
    const typename _Base::allocator_type & __a = typename _Base::allocator_type())
  : _Base(__l, __a)
  {
    if (__l.size() > _UpperBound) {
      throw std::length_error("Exceeded upper bound");
    }
  }

  /**
   *  @brief  Builds a %BoundedVector from a range.
   *  @param  __first  An input iterator.
   *  @param  __last  An input iterator.
   *  @param  __a  An allocator.
   *
   *  Create a %BoundedVector consisting of copies of the elements from
   *  [first,last).
   *
   *  If the iterators are forward, bidirectional, or
   *  random-access, then this will call the elements' copy
   *  constructor N times (where N is distance(first,last)) and do
   *  no memory reallocation.  But if only input iterators are
   *  used, then this will do at most 2N calls to the copy
   *  constructor, and logN memory reallocations.
   */
  template<
    typename _InputIterator
  >
  BoundedVector(
    _InputIterator __first,
    _InputIterator __last,
    const typename _Base::allocator_type & __a = allocator_type())
  : _Base(__first, __last, __a)
  {
    if (size() > _UpperBound) {
      throw std::length_error("Exceeded upper bound");
    }
  }

  /**
   *  The dtor only erases the elements, and note that if the
   *  elements themselves are pointers, the pointed-to memory is
   *  not touched in any way.  Managing the pointer is the user's
   *  responsibility.
   */
  ~BoundedVector() noexcept
  {}

  /**
   *  @brief  %BoundedVector assignment operator.
   *  @param  __x  A %BoundedVector of identical element and allocator types.
   *
   *  All the elements of @a __x are copied, but any extra memory in
   *  @a __x (for fast expansion) will not be copied.  Unlike the
   *  copy constructor, the allocator object is not copied.
   */
  BoundedVector &
  operator=(const BoundedVector & __x)
  {
    reinterpret_cast<std::vector<_Tp, _Alloc> *>(this)->operator=(
      *reinterpret_cast<const std::vector<_Tp, _Alloc> *>(&__x));
    return *this;
  }

  /**
   *  @brief  %BoundedVector list assignment operator.
   *  @param  __l  An initializer_list.
   *
   *  This function fills a %BoundedVector with copies of the elements in the
   *  initializer list @a __l.
   *
   *  Note that the assignment completely changes the %BoundedVector and
   *  that the resulting %BoundedVector's size is the same as the number
   *  of elements assigned.  Old data may be lost.
   */
  BoundedVector &
  operator=(std::initializer_list<typename _Base::value_type> __l)
  {
    if (__l.size() > _UpperBound) {
      throw std::length_error("Exceeded upper bound");
    }
    _Base::operator=(__l);
    return *this;
  }

  /**
   *  @brief  Assigns a given value to a %BoundedVector.
   *  @param  __n  Number of elements to be assigned.
   *  @param  __val  Value to be assigned.
   *
   *  This function fills a %BoundedVector with @a __n copies of the given
   *  value.  Note that the assignment completely changes the
   *  %BoundedVector and that the resulting %BoundedVector's size is the same as
   *  the number of elements assigned.  Old data may be lost.
   */
  void
  assign(
    typename _Base::size_type __n,
    const typename _Base::value_type & __val)
  {
    if (__n > _UpperBound) {
      throw std::length_error("Exceeded upper bound");
    }
    _Base::assign(__n, __val);
  }

  /**
   *  @brief  Assigns a range to a %BoundedVector.
   *  @param  __first  An input iterator.
   *  @param  __last   An input iterator.
   *
   *  This function fills a %BoundedVector with copies of the elements in the
   *  range [__first,__last).
   *
   *  Note that the assignment completely changes the %BoundedVector and
   *  that the resulting %BoundedVector's size is the same as the number
   *  of elements assigned.  Old data may be lost.
   */
  template<
    typename _InputIterator
  >
  void
  assign(_InputIterator __first, _InputIterator __last)
  {
    if (size() + std::distance(__first, __last) > _UpperBound) {
      throw std::length_error("Exceeded upper bound");
    }
    _Base::assign(__first, __last);
  }

  /**
   *  @brief  Assigns an initializer list to a %BoundedVector.
   *  @param  __l  An initializer_list.
   *
   *  This function fills a %BoundedVector with copies of the elements in the
   *  initializer list @a __l.
   *
   *  Note that the assignment completely changes the %BoundedVector and
   *  that the resulting %BoundedVector's size is the same as the number
   *  of elements assigned.  Old data may be lost.
   */
  void
  assign(std::initializer_list<typename _Base::value_type> __l)
  {
    if (__l.size() > _UpperBound) {
      throw std::length_error("Exceeded upper bound");
    }
    _Base::assign(__l);
  }

  using _Base::begin;
  using _Base::end;
  using _Base::rbegin;
  using _Base::rend;
  using _Base::cbegin;
  using _Base::cend;
  using _Base::crbegin;
  using _Base::crend;
  using _Base::size;

  /**  Returns the size() of the largest possible %BoundedVector.  */
  typename _Base::size_type
  max_size() const noexcept
  {
    return std::min(_UpperBound, _Base::max_size());
  }

  /**
   *  @brief  Resizes the %BoundedVector to the specified number of elements.
   *  @param  __new_size  Number of elements the %BoundedVector should contain.
   *
   *  This function will %resize the %BoundedVector to the specified
   *  number of elements.  If the number is smaller than the
   *  %BoundedVector's current size the %BoundedVector is truncated, otherwise
   *  default constructed elements are appended.
   */
  void
  resize(typename _Base::size_type __new_size)
  {
    if (__new_size > _UpperBound) {
      throw std::length_error("Exceeded upper bound");
    }
    _Base::resize(__new_size);
  }

  /**
   *  @brief  Resizes the %BoundedVector to the specified number of elements.
   *  @param  __new_size  Number of elements the %BoundedVector should contain.
   *  @param  __x  Data with which new elements should be populated.
   *
   *  This function will %resize the %BoundedVector to the specified
   *  number of elements.  If the number is smaller than the
   *  %BoundedVector's current size the %BoundedVector is truncated, otherwise
   *  the %BoundedVector is extended and new elements are populated with
   *  given data.
   */
  void
  resize(
    typename _Base::size_type __new_size,
    const typename _Base::value_type & __x)
  {
    if (__new_size > _UpperBound) {
      throw std::length_error("Exceeded upper bound");
    }
    _Base::resize(__new_size, __x);
  }

  using _Base::shrink_to_fit;
  using _Base::capacity;
  using _Base::empty;

  /**
   *  @brief  Attempt to preallocate enough memory for specified number of
   *          elements.
   *  @param  __n  Number of elements required.
   *  @throw  std::length_error  If @a n exceeds @c max_size().
   *
   *  This function attempts to reserve enough memory for the
   *  %BoundedVector to hold the specified number of elements.  If the
   *  number requested is more than max_size(), length_error is
   *  thrown.
   *
   *  The advantage of this function is that if optimal code is a
   *  necessity and the user can determine the number of elements
   *  that will be required, the user can reserve the memory in
   *  %advance, and thus prevent a possible reallocation of memory
   *  and copying of %BoundedVector data.
   */
  void
  reserve(typename _Base::size_type __n)
  {
    if (__n > _UpperBound) {
      throw std::length_error("Exceeded upper bound");
    }
    _Base::reserve(__n);
  }

  using _Base::operator[];
  using _Base::at;
  using _Base::front;
  using _Base::back;

  // DR 464. Suggestion for new member functions in standard containers.
  // data access
  /**
   *  Returns a pointer such that [data(), data() + size()) is a valid
   *  range.  For a non-empty %BoundedVector, data() == &front().
   */
  template<
    typename T,
    typename std::enable_if<
      !std::is_same<T, _Tp>::value &&
      !std::is_same<T, bool>::value
    >::type * = nullptr
  >
  T *
  data() noexcept
  {
    return _Base::data();
  }

  template<
    typename T,
    typename std::enable_if<
      !std::is_same<T, _Tp>::value &&
      !std::is_same<T, bool>::value
    >::type * = nullptr
  >
  const T *
  data() const noexcept
  {
    return _Base::data();
  }

  // [23.2.4.3] modifiers
  /**
   *  @brief  Add data to the end of the %BoundedVector.
   *  @param  __x  Data to be added.
   *
   *  This is a typical stack operation.  The function creates an
   *  element at the end of the %BoundedVector and assigns the given data
   *  to it.  Due to the nature of a %BoundedVector this operation can be
   *  done in constant time if the %BoundedVector has preallocated space
   *  available.
   */
  void
  push_back(const typename _Base::value_type & __x)
  {
    if (size() >= _UpperBound) {
      throw std::length_error("Exceeded upper bound");
    }
    _Base::push_back(__x);
  }

  void
  push_back(typename _Base::value_type && __x)
  {
    if (size() >= _UpperBound) {
      throw std::length_error("Exceeded upper bound");
    }
    _Base::push_back(__x);
  }

  /**
   *  @brief  Inserts an object in %BoundedVector before specified iterator.
   *  @param  __position  A const_iterator into the %BoundedVector.
   *  @param  __args  Arguments.
   *  @return  An iterator that points to the inserted data.
   *
   *  This function will insert an object of type T constructed
   *  with T(std::forward<Args>(args)...) before the specified location.
   *  Note that this kind of operation could be expensive for a %BoundedVector
   *  and if it is frequently used the user should consider using
   *  std::list.
   */
  template<typename ... _Args>
  typename _Base::iterator
  emplace(
    typename _Base::const_iterator __position,
    _Args && ... __args)
  {
    if (size() >= _UpperBound) {
      throw std::length_error("Exceeded upper bound");
    }
    _Base::emplace(__position, std::forward<_Args>(__args) ...);
  }

  /**
   *  @brief  Inserts given value into %BoundedVector before specified iterator.
   *  @param  __position  A const_iterator into the %BoundedVector.
   *  @param  __x  Data to be inserted.
   *  @return  An iterator that points to the inserted data.
   *
   *  This function will insert a copy of the given value before
   *  the specified location.  Note that this kind of operation
   *  could be expensive for a %BoundedVector and if it is frequently
   *  used the user should consider using std::list.
   */
  typename _Base::iterator
  insert(
    typename _Base::const_iterator __position,
    const typename _Base::value_type & __x)
  {
    if (size() >= _UpperBound) {
      throw std::length_error("Exceeded upper bound");
    }
    return _Base::insert(__position, __x);
  }

  /**
   *  @brief  Inserts given rvalue into %BoundedVector before specified iterator.
   *  @param  __position  A const_iterator into the %BoundedVector.
   *  @param  __x  Data to be inserted.
   *  @return  An iterator that points to the inserted data.
   *
   *  This function will insert a copy of the given rvalue before
   *  the specified location.  Note that this kind of operation
   *  could be expensive for a %BoundedVector and if it is frequently
   *  used the user should consider using std::list.
   */
  typename _Base::iterator
  insert(
    typename _Base::const_iterator __position,
    typename _Base::value_type && __x)
  {
    if (size() >= _UpperBound) {
      throw std::length_error("Exceeded upper bound");
    }
    return _Base::insert(__position, __x);
  }

  /**
   *  @brief  Inserts an initializer_list into the %BoundedVector.
   *  @param  __position  An iterator into the %BoundedVector.
   *  @param  __l  An initializer_list.
   *
   *  This function will insert copies of the data in the
   *  initializer_list @a l into the %BoundedVector before the location
   *  specified by @a position.
   *
   *  Note that this kind of operation could be expensive for a
   *  %BoundedVector and if it is frequently used the user should
   *  consider using std::list.
   */
  typename _Base::iterator
  insert(
    typename _Base::const_iterator __position,
    std::initializer_list<typename _Base::value_type> __l)
  {
    if (size() + __l.size() > _UpperBound) {
      throw std::length_error("Exceeded upper bound");
    }
    return _Base::insert(__position, __l);
  }

  /**
   *  @brief  Inserts a number of copies of given data into the %BoundedVector.
   *  @param  __position  A const_iterator into the %BoundedVector.
   *  @param  __n  Number of elements to be inserted.
   *  @param  __x  Data to be inserted.
   *  @return  An iterator that points to the inserted data.
   *
   *  This function will insert a specified number of copies of
   *  the given data before the location specified by @a position.
   *
   *  Note that this kind of operation could be expensive for a
   *  %BoundedVector and if it is frequently used the user should
   *  consider using std::list.
   */
  typename _Base::iterator
  insert(
    typename _Base::const_iterator __position,
    typename _Base::size_type __n,
    const typename _Base::value_type & __x)
  {
    if (size() + __n > _UpperBound) {
      throw std::length_error("Exceeded upper bound");
    }
    return _Base::insert(__position, __n, __x);
  }

  /**
   *  @brief  Inserts a range into the %BoundedVector.
   *  @param  __position  A const_iterator into the %BoundedVector.
   *  @param  __first  An input iterator.
   *  @param  __last   An input iterator.
   *  @return  An iterator that points to the inserted data.
   *
   *  This function will insert copies of the data in the range
   *  [__first,__last) into the %BoundedVector before the location specified
   *  by @a pos.
   *
   *  Note that this kind of operation could be expensive for a
   *  %BoundedVector and if it is frequently used the user should
   *  consider using std::list.
   */
  template<
    typename _InputIterator
  >
  typename _Base::iterator
  insert(
    typename _Base::const_iterator __position,
    _InputIterator __first,
    _InputIterator __last)
  {
    if (size() + std::distance(__first, __last) > _UpperBound) {
      throw std::length_error("Exceeded upper bound");
    }
    return _Base::insert(__position, __first, __last);
  }

  using _Base::erase;
  using _Base::pop_back;
  using _Base::clear;
};

/**
 *  @brief  Vector equality comparison.
 *  @param  __x  A %BoundedVector.
 *  @param  __y  A %BoundedVector of the same type as @a __x.
 *  @return  True if the size and elements of the vectors are equal.
 *
 *  This is an equivalence relation.  It is linear in the size of the
 *  vectors.  Vectors are considered equivalent if their sizes are equal,
 *  and if corresponding elements compare equal.
*/
template<typename _Tp, std::size_t _UpperBound, typename _Alloc>
inline bool
operator==(
  const BoundedVector<_Tp, _UpperBound, _Alloc> & __x,
  const BoundedVector<_Tp, _UpperBound, _Alloc> & __y)
{
  return operator==(
    *reinterpret_cast<const std::vector<_Tp, _Alloc> *>(&__x),
    *reinterpret_cast<const std::vector<_Tp, _Alloc> *>(&__y));
}

/**
 *  @brief  Vector ordering relation.
 *  @param  __x  A %BoundedVector.
 *  @param  __y  A %BoundedVector of the same type as @a __x.
 *  @return  True if @a __x is lexicographically less than @a __y.
 *
 *  This is a total ordering relation.  It is linear in the size of the
 *  vectors.  The elements must be comparable with @c <.
 *
 *  See std::lexicographical_compare() for how the determination is made.
*/
template<typename _Tp, std::size_t _UpperBound, typename _Alloc>
inline bool
operator<(
  const BoundedVector<_Tp, _UpperBound, _Alloc> & __x,
  const BoundedVector<_Tp, _UpperBound, _Alloc> & __y)
{
  return operator<(
    *reinterpret_cast<const std::vector<_Tp, _Alloc> *>(&__x),
    *reinterpret_cast<const std::vector<_Tp, _Alloc> *>(&__y));
}

/// Based on operator==
template<typename _Tp, std::size_t _UpperBound, typename _Alloc>
inline bool
operator!=(
  const BoundedVector<_Tp, _UpperBound, _Alloc> & __x,
  const BoundedVector<_Tp, _UpperBound, _Alloc> & __y)
{
  return operator!=(
    *reinterpret_cast<const std::vector<_Tp, _Alloc> *>(&__x),
    *reinterpret_cast<const std::vector<_Tp, _Alloc> *>(&__y));
}

/// Based on operator<
template<typename _Tp, std::size_t _UpperBound, typename _Alloc>
inline bool
operator>(
  const BoundedVector<_Tp, _UpperBound, _Alloc> & __x,
  const BoundedVector<_Tp, _UpperBound, _Alloc> & __y)
{
  return operator>(
    *reinterpret_cast<const std::vector<_Tp, _Alloc> *>(&__x),
    *reinterpret_cast<const std::vector<_Tp, _Alloc> *>(&__y));
}

/// Based on operator<
template<typename _Tp, std::size_t _UpperBound, typename _Alloc>
inline bool
operator<=(
  const BoundedVector<_Tp, _UpperBound, _Alloc> & __x,
  const BoundedVector<_Tp, _UpperBound, _Alloc> & __y)
{
  return operator<=(
    *reinterpret_cast<const std::vector<_Tp, _Alloc> *>(&__x),
    *reinterpret_cast<const std::vector<_Tp, _Alloc> *>(&__y));
}

/// Based on operator<
template<typename _Tp, std::size_t _UpperBound, typename _Alloc>
inline bool
operator>=(
  const BoundedVector<_Tp, _UpperBound, _Alloc> & __x,
  const BoundedVector<_Tp, _UpperBound, _Alloc> & __y)
{
  return operator>=(
    *reinterpret_cast<const std::vector<_Tp, _Alloc> *>(&__x),
    *reinterpret_cast<const std::vector<_Tp, _Alloc> *>(&__y));
}

/// See rosidl_generator_cpp::BoundedVector::swap().
template<typename _Tp, std::size_t _UpperBound, typename _Alloc>
inline void
swap(BoundedVector<_Tp, _UpperBound, _Alloc> & __x, BoundedVector<_Tp, _UpperBound, _Alloc> & __y)
{
  __x.swap(__y);
}

}  // namespace rosidl_generator_cpp

#endif  // ROSIDL_GENERATOR_CPP__BOUNDED_VECTOR_HPP_
