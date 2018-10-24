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
from rosidl_generator_cpp import create_init_alloc_and_member_lists
from rosidl_generator_cpp import escape_string
from rosidl_generator_cpp import msg_type_only_to_cpp
from rosidl_generator_cpp import msg_type_to_cpp
from rosidl_generator_cpp import MSG_TYPE_TO_CPP

cpp_namespace = '%s::%s::' % (spec.base_type.pkg_name, subfolder)
cpp_class = '%s_' % spec.base_type.type
cpp_full_name = '%s%s' % (cpp_namespace, cpp_class)
cpp_full_name_with_alloc = '%s<ContainerAllocator>' % (cpp_full_name)
}@
#include <rosidl_generator_cpp/bounded_vector.hpp>
#include <rosidl_generator_cpp/message_initialization.hpp>
#include <algorithm>
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

@{
# The creation of the constructors for messages is a bit complicated.  The goal
# is to have a constructor where the user can control how the fields of the
# message get initialized via the _init parameter to the constructor.  See
# http://design.ros2.org/articles/generated_interfaces_cpp.html#constructors
# for a detailed explanation of the different _init parameters.
init_list, alloc_list, member_list = create_init_alloc_and_member_lists(spec)

def generate_default_string(membset):
    strlist = []
    for member in membset.members:
        if member.default_value is not None:
            if member.num_prealloc > 0:
                strlist.append('this->%s.resize(%d);' % (member.name, member.num_prealloc))
            if isinstance(member.default_value, list):
                if all(v == member.default_value[0] for v in member.default_value):
                    # Specifying type for std::fill because of MSVC 14.12 warning about casting 'const int' to smaller types (C4244)
                    # For more info, see https://github.com/ros2/rosidl/issues/309
                    # TODO(jacobperron): Investigate reason for build warnings on Windows
                    # TODO(jacobperron): Write test case for this path of execution
                    strlist.append('std::fill<typename %s::iterator, %s>(this->%s.begin(), this->%s.end(), %s);' % (msg_type_to_cpp(member.type), msg_type_only_to_cpp(member.type), member.name, member.name, member.default_value[0]))
                else:
                    for index, val in enumerate(member.default_value):
                        strlist.append('this->%s[%d] = %s;' % (member.name, index, val))
            else:
                strlist.append('this->%s = %s;' % (member.name, member.default_value))

    return strlist

def generate_zero_string(membset, fill_args):
    strlist = []
    for member in membset.members:
        if isinstance(member.zero_value, list):
            if member.num_prealloc > 0:
                strlist.append('this->%s.resize(%d);' % (member.name, member.num_prealloc))
            if member.zero_need_array_override:
                strlist.append('this->%s.fill(%s{%s});' % (member.name, msg_type_only_to_cpp(member.type), fill_args))
            else:
                # Specifying type for std::fill because of MSVC 14.12 warning about casting 'const int' to smaller types (C4244)
                # For more info, see https://github.com/ros2/rosidl/issues/309
                # TODO(jacobperron): Investigate reason for build warnings on Windows
                strlist.append('std::fill<typename %s::iterator, %s>(this->%s.begin(), this->%s.end(), %s);' % (msg_type_to_cpp(member.type), msg_type_only_to_cpp(member.type), member.name, member.name, member.zero_value[0]))
        else:
            strlist.append('this->%s = %s;' % (member.name, member.zero_value))
    return strlist
}@
  explicit @(spec.base_type.type)_(rosidl_generator_cpp::MessageInitialization _init = rosidl_generator_cpp::MessageInitialization::ALL)
@[if init_list]@
  : @(',\n    '.join(init_list))
@[end if]@
  {
@[if not member_list]@
    (void)_init;
@[end if]@
@[for membset in member_list]@
@[ if membset.members[0].default_value is not None]@
    if (rosidl_generator_cpp::MessageInitialization::ALL == _init ||
      rosidl_generator_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
@[  for line in generate_default_string(membset)]@
      @(line)
@[  end for]@
@[  if membset.members[0].zero_value is not None]@
    } else if (rosidl_generator_cpp::MessageInitialization::ZERO == _init) {
@[   for line in generate_zero_string(membset, '_init')]@
      @(line)
@[   end for]@
@[  end if]@
    }
@[ elif membset.members[0].zero_value is not None]@
    if (rosidl_generator_cpp::MessageInitialization::ALL == _init ||
      rosidl_generator_cpp::MessageInitialization::ZERO == _init)
    {
@[  for line in generate_zero_string(membset, '_init')]@
      @(line)
@[  end for]@
    }
@[ end if]@
@[end for]@
  }

  explicit @(spec.base_type.type)_(const ContainerAllocator & _alloc, rosidl_generator_cpp::MessageInitialization _init = rosidl_generator_cpp::MessageInitialization::ALL)
@[if alloc_list]@
  : @(',\n    '.join(alloc_list))
@[end if]@
  {
@[if not member_list]@
    (void)_init;
@[end if]@
@[if not alloc_list]@
    (void)_alloc;
@[end if]@
@[for membset in member_list]@
@[ if membset.members[0].default_value is not None]@
    if (rosidl_generator_cpp::MessageInitialization::ALL == _init ||
      rosidl_generator_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
@[  for line in generate_default_string(membset)]@
      @(line)
@[  end for]@
@[  if membset.members[0].zero_value is not None]@
    } else if (rosidl_generator_cpp::MessageInitialization::ZERO == _init) {
@[   for line in generate_zero_string(membset, '_alloc, _init')]@
      @(line)
@[   end for]@
@[  end if]@
    }
@[ elif membset.members[0].zero_value is not None]@
    if (rosidl_generator_cpp::MessageInitialization::ALL == _init ||
      rosidl_generator_cpp::MessageInitialization::ZERO == _init)
    {
@[  for line in generate_zero_string(membset, '_alloc, _init')]@
      @(line)
@[  end for]@
    }
@[ end if]@
@[end for]@
  }

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

  // constant declarations
@[for constant in spec.constants]@
@[if constant.type == 'string']
  static const @(MSG_TYPE_TO_CPP[constant.type]) @(constant.name);
@[else]@
  static constexpr @(MSG_TYPE_TO_CPP[constant.type]) @(constant.name) =
@[if constant.type in ('bool', 'byte', 'int8', 'int16', 'int32', 'int64', 'char')]@
    @(int(constant.value));
@[elif constant.type in ('uint8', 'uint16', 'uint32', 'uint64')]@
    @(int(constant.value))u;
@[else]@
    @(constant.value);
@[  end if]@
@[end if]@
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

// constant definitions
@[for c in spec.constants]@
@[if c.type == 'string']@
template<typename ContainerAllocator>
const @(MSG_TYPE_TO_CPP[c.type])
@(spec.base_type.type)_<ContainerAllocator>::@(c.name) = "@(escape_string(c.value))";
@[ else ]@
template<typename ContainerAllocator>
constexpr @(MSG_TYPE_TO_CPP[c.type]) @(spec.base_type.type)_<ContainerAllocator>::@(c.name);
@[end if]@
@[end for]@

}  // namespace @(subfolder)

}  // namespace @(spec.base_type.pkg_name)

#endif  // @(header_guard_variable)
