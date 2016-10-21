// generated from rosidl_generator_cpp/resource/msg__struct.hpp.em
// generated code does not contain a copyright notice

@#######################################################################
@# EmPy template for generating <msg>__struct.hpp files
@#
@# Context:
@#  - spec (rosidl_parser.MessageSpecification)
@#    Parsed specification of the .msg file
@#  - subfolder (string)
@#    The subfolder / subnamespace of the message
@#    Either 'msg' or 'srv'
@#  - get_header_filename_from_msg_name (function)
@#######################################################################
@
@{
header_guard_parts = [
    spec.base_type.pkg_name, subfolder,
    get_header_filename_from_msg_name(spec.base_type.type) + '__struct_hpp']
header_guard_variable = '__'.join([x.upper() for x in header_guard_parts]) + '_'
}@
#ifndef @(header_guard_variable)
#define @(header_guard_variable)

// Protect against ERROR being predefined on Windows, in case somebody defines a
// constant by that name.
#if defined(_WIN32) && defined(ERROR)
  #undef ERROR
#endif

@{
from rosidl_generator_cpp import escape_string
from rosidl_generator_cpp import msg_type_to_cpp
from rosidl_generator_cpp import MSG_TYPE_TO_CPP
from rosidl_generator_cpp import value_to_cpp

cpp_namespace = '%s::%s::' % (spec.base_type.pkg_name, subfolder)
cpp_class = '%s_' % spec.base_type.type
cpp_full_name = '%s%s' % (cpp_namespace, cpp_class)
cpp_full_name_with_alloc = '%s<ContainerAllocator>' % (cpp_full_name)
}@
#include <rosidl_generator_cpp/bounded_vector.hpp>
#include <array>
#include <memory>
#include <string>
#include <vector>

// include message dependencies
@{
includes = {}
for field in spec.fields:
    if not field.type.is_primitive_type():
        key = '%s/msg/%s.hpp' % \
            (field.type.pkg_name, get_header_filename_from_msg_name(field.type.type))
        if key not in includes:
            includes[key] = set([])
        includes[key].add(field.name)
for key in sorted(includes.keys()):
    print('#include "%s"  // %s' % (key, ', '.join(includes[key])))
}@

@{
deprecated_macro_name = \
    '_'.join(['DEPRECATED', spec.base_type.pkg_name, subfolder, spec.base_type.type])
}@
#ifndef _WIN32
# define @(deprecated_macro_name) __attribute__((deprecated))
#else
# define @(deprecated_macro_name) __declspec(deprecated)
#endif

namespace @(spec.base_type.pkg_name)
{

namespace @(subfolder)
{

// message struct
template<class ContainerAllocator>
struct @(spec.base_type.type)_
{
  using Type = @(spec.base_type.type)_<ContainerAllocator>;

@# constructors (with and without allocator)
@[for (alloc_type, alloc_name) in [['', ''], ['const ContainerAllocator & ', '_alloc']]]@
  @('explicit ' if alloc_type else '')@(spec.base_type.type)_(@(alloc_type + alloc_name))
@# generate initializer lists
@[  if alloc_name == '_alloc']@
// *INDENT-OFF* (prevent uncrustify from making unnecessary indents here)
@[  end if]@
@{
used_alloc_name = False
leading_colon = ':'
for field in spec.fields:
    # dynamic arrays require allocator if available
    if alloc_name and field.type.is_array and \
            (field.type.array_size is None or field.type.is_upper_bound):
        if leading_colon == ' ':
            print(',')
        print('  %s %s(%s)' % (leading_colon, field.name, alloc_name), end='')
        used_alloc_name = True
        leading_colon = ' '

    # if default value is available set it in initializer list
    elif field.type.is_primitive_type() and field.default_value is not None:
        cpp_value = value_to_cpp(field.type, field.default_value)
        if leading_colon == ' ':
            print(',')
        print('  %s %s(%s)' % (leading_colon, field.name, cpp_value), end='')
        leading_colon = ' '
if leading_colon == ' ':
    print()
}@
@[  if alloc_name == '_alloc']@
// *INDENT-ON*
@[  end if]@
  {
@# generate constructor body
@[  if alloc_name and not used_alloc_name]@
    (void)@(alloc_name);
@[  end if]@
@{
for field in spec.fields:
    # default values of dynamic arrays with allocators if available
    if alloc_name and field.type.is_array and \
            (field.type.array_size is None or field.type.is_upper_bound) and \
            field.type.is_primitive_type() and field.default_value is not None:
        cpp_value = value_to_cpp(field.type, field.default_value)
        print('    %s = %s;' % (field.name, cpp_value))
}@
  }
@[end for]@

  // field types and members
@[for field in spec.fields]@
  using _@(field.name)_type =
      @(msg_type_to_cpp(field.type));
  _@(field.name)_type @(field.name);
@[end for]@

  // setters for named parameter idiom
@[for field in spec.fields]@
  Type * set__@(field.name)(
    const @(msg_type_to_cpp(field.type)) & _arg)
  {
    this->@(field.name) = _arg;
    return this;
  }
@[end for]@

  // constants
@[for constant in spec.constants]@
@[  if (constant.type in ['byte', 'int8', 'int16', 'int32', 'int64', 'char'])]@
  enum { @(constant.name) = @(int(constant.value)) };
@[  elif (constant.type in ['uint8', 'uint16', 'uint32', 'uint64'])]@
  enum { @(constant.name) = @(int(constant.value))u };
@[  else]@
  static const @(MSG_TYPE_TO_CPP[constant.type]) @(constant.name);
@[  end if]@
@[end for]@

  // pointer types
  using RawPtr =
      @(cpp_full_name)<ContainerAllocator> *;
  using ConstRawPtr =
      const @(cpp_full_name)<ContainerAllocator> *;
  using SharedPtr =
      std::shared_ptr<@(cpp_full_name)<ContainerAllocator>>;
  using ConstSharedPtr =
      std::shared_ptr<@(cpp_full_name)<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
    @(cpp_full_name)<ContainerAllocator>>>
  using UniquePtrWithDeleter =
      std::unique_ptr<@(cpp_full_name)<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
    @(cpp_full_name)<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
      std::unique_ptr<@(cpp_full_name)<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
      std::weak_ptr<@(cpp_full_name)<ContainerAllocator>>;
  using ConstWeakPtr =
      std::weak_ptr<@(cpp_full_name)<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef @(deprecated_macro_name)
    std::shared_ptr<@(cpp_full_name)<ContainerAllocator>>
    Ptr;
  typedef @(deprecated_macro_name)
    std::shared_ptr<@(cpp_full_name)<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const @(spec.base_type.type)_ & other) const
  {
@[if not spec.fields]@
    (void)other;
@[end if]@
@[for field in spec.fields]@
    if (this->@(field.name) != other.@(field.name)) {
      return false;
    }
@[end for]@
    return true;
  }
  bool operator!=(const @(spec.base_type.type)_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct @(cpp_class)

// alias to use template instance with default allocator
using @(spec.base_type.type) =
    @(cpp_full_name)<std::allocator<void>>;

// constants requiring out of line definition
@[for c in spec.constants]@
@[  if c.type not in ['byte', 'int8', 'int16', 'int32', 'int64', 'char', 'uint8', 'uint16', 'uint32', 'uint64']]@
template<typename ContainerAllocator>
const @(MSG_TYPE_TO_CPP[c.type])
@(spec.base_type.type)_<ContainerAllocator>::@(c.name) =
@[    if c.type == 'string']@
  "@(escape_string(c.value))";
@[    elif c.type == 'bool']@
  @(int(c.value));
@[    else]@
  @(c.value);
@[    end if]@
@[  end if]@
@[end for]@

}  // namespace @(subfolder)

}  // namespace @(spec.base_type.pkg_name)

#endif  // @(header_guard_variable)
