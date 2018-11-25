@# Included from rosidl_generator_cpp/resource/idl__struct.hpp.em
// Protect against ERROR being predefined on Windows, in case somebody defines a
// constant by that name.
#if defined(_WIN32) && defined(ERROR)
  #undef ERROR
#endif
@
@{
from rosidl_generator_cpp import create_init_alloc_and_member_lists
from rosidl_generator_cpp import escape_string
from rosidl_generator_cpp import msg_type_to_cpp
from rosidl_generator_cpp import MSG_TYPE_TO_CPP
from rosidl_parser.definition import BaseString
from rosidl_parser.definition import BasicType
from rosidl_parser.definition import NamespacedType
from rosidl_parser.definition import NestedType

message_typename = '::'.join(message.structure.type.namespaces + [message.structure.type.name])
}@
@
@#<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
@# Collect necessary include directives for all members
@{
from collections import OrderedDict
from rosidl_cmake import convert_camel_case_to_lower_case_underscore
includes = OrderedDict()
for member in message.structure.members:
    print('// member:', member.name)
    type_ = member.type
    if isinstance(type_, NestedType):
        type_ = type_.basetype
    if isinstance(type_, NamespacedType):
        filename_prefix = convert_camel_case_to_lower_case_underscore(type_.name)
        if filename_prefix.endswith('__request'):
            filename_prefix = filename_prefix[:-9]
        elif filename_prefix.endswith('__response'):
            filename_prefix = filename_prefix[:-10]
        if filename_prefix.endswith('__goal'):
            filename_prefix = filename_prefix[:-6]
        elif filename_prefix.endswith('__result'):
            filename_prefix = filename_prefix[:-8]
        elif filename_prefix.endswith('__feedback'):
            filename_prefix = filename_prefix[:-10]
        member_names = includes.setdefault(
            '/'.join((type_.namespaces + [filename_prefix])) + '__struct.hpp', [])
        member_names.append(member.name)
}@
@#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
@
@#<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
@[if includes]@

// Include directives for member types
@[    for header_file, member_names in includes.items()]@
@[        for member_name in member_names]@
// Member '@(member_name)'
@[        end for]@
@[        if header_file in include_directives]@
// already included above
// @
@[        else]@
@{include_directives.add(header_file)}@
@[        end if]@
#include "@(header_file)"
@[    end for]@
@[end if]@
@#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

@{
deprecated_macro_name = \
    '__'.join(['DEPRECATED', package_name] + list(interface_path.parents[0].parts) + [message.structure.type.name])
}@
#ifndef _WIN32
# define @(deprecated_macro_name) __attribute__((deprecated))
#else
# define @(deprecated_macro_name) __declspec(deprecated)
#endif

@[for ns in message.structure.type.namespaces]@
namespace @(ns)
{

@[end for]@
// message struct
template<class ContainerAllocator>
struct @(message.structure.type.name)_
{
  using Type = @(message.structure.type.name)_<ContainerAllocator>;

@{
# The creation of the constructors for messages is a bit complicated.  The goal
# is to have a constructor where the user can control how the fields of the
# message get initialized via the _init parameter to the constructor.  See
# http://design.ros2.org/articles/generated_interfaces_cpp.html#constructors
# for a detailed explanation of the different _init parameters.
init_list, alloc_list, member_list = create_init_alloc_and_member_lists(message)

def generate_default_string(membset):
    strlist = []
    for member in membset.members:
        if member.default_value is not None:
            if member.num_prealloc > 0:
                strlist.append('this->%s.resize(%d);' % (member.name, member.num_prealloc))
            if isinstance(member.default_value, list):
                if all(v == member.default_value[0] for v in member.default_value):
                    strlist.append('std::fill(this->%s.begin(), this->%s.end(), %s);' % (member.name, member.name, member.default_value[0]))
                else:
                    for index, val in enumerate(member.default_value):
                        strlist.append('this->%s[%d] = %s;' % (member.name, index, val))
            else:
                strlist.append('this->%s = %s;' % (member.name, member.default_value))

    return strlist

def generate_zero_string(membset, fill_args):
    from rosidl_generator_cpp import msg_type_only_to_cpp
    strlist = []
    for member in membset.members:
        if isinstance(member.zero_value, list):
            if member.num_prealloc > 0:
                strlist.append('this->%s.resize(%d);' % (member.name, member.num_prealloc))
            if member.zero_need_array_override:
                strlist.append('this->%s.fill(%s{%s});' % (member.name, msg_type_only_to_cpp(member.type), fill_args))
            else:
                strlist.append('std::fill(this->%s.begin(), this->%s.end(), %s);' % (member.name, member.name, member.zero_value[0]))
        else:
            strlist.append('this->%s = %s;' % (member.name, member.zero_value))
    return strlist
}@
  explicit @(message.structure.type.name)_(rosidl_generator_cpp::MessageInitialization _init = rosidl_generator_cpp::MessageInitialization::ALL)
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

  explicit @(message.structure.type.name)_(const ContainerAllocator & _alloc, rosidl_generator_cpp::MessageInitialization _init = rosidl_generator_cpp::MessageInitialization::ALL)
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
@[for member in message.structure.members]@
  using _@(member.name)_type =
    @(msg_type_to_cpp(member.type));
  _@(member.name)_type @(member.name);
@[end for]@

  // setters for named parameter idiom
@[for member in message.structure.members]@
  Type * set__@(member.name)(
    const @(msg_type_to_cpp(member.type)) & _arg)
  {
    this->@(member.name) = _arg;
    return this;
  }
@[end for]@

  // constant declarations
@[for constant in message.constants.values()]@
@[ if isinstance(constant.type, BaseString)]@
  static const @(MSG_TYPE_TO_CPP['string']) @(constant.name);
@[ else]@
  static constexpr @(MSG_TYPE_TO_CPP[constant.type.type]) @(constant.name) =
@[  if isinstance(constant.type, BasicType) and constant.type.type in ['short', 'unsigned short', 'long', 'unsigned long', 'long long', 'unsigned long long', 'char', 'wchar', 'boolean', 'octet', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']]@
    @(int(constant.value))@
@[   if constant.type.type.startswith('u')]@
u@
@[   end if];
@[  else]@
    @(constant.value);
@[  end if]@
@[ end if]@
@[end for]@

  // pointer types
  using RawPtr =
    @(message_typename)_<ContainerAllocator> *;
  using ConstRawPtr =
    const @(message_typename)_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<@(message_typename)_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<@(message_typename)_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      @(message_typename)_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<@(message_typename)_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      @(message_typename)_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<@(message_typename)_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<@(message_typename)_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<@(message_typename)_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef @(deprecated_macro_name)
    std::shared_ptr<@(message_typename)_<ContainerAllocator>>
    Ptr;
  typedef @(deprecated_macro_name)
    std::shared_ptr<@(message_typename)_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const @(message.structure.type.name)_ & other) const
  {
@[if not message.structure.members]@
    (void)other;
@[end if]@
@[for member in message.structure.members]@
    if (this->@(member.name) != other.@(member.name)) {
      return false;
    }
@[end for]@
    return true;
  }
  bool operator!=(const @(message.structure.type.name)_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct @(message.structure.type.name)_

// alias to use template instance with default allocator
using @(message.structure.type.name) =
  @(message_typename)_<std::allocator<void>>;

// constant definitions
@[for c in message.constants.values()]@
@[ if isinstance(c.type, BaseString)]@
template<typename ContainerAllocator>
const @(MSG_TYPE_TO_CPP['string'])
@(message.structure.type.name)_<ContainerAllocator>::@(c.name) = "@(escape_string(c.value))";
@[ else ]@
template<typename ContainerAllocator>
constexpr @(MSG_TYPE_TO_CPP[c.type.type]) @(message.structure.type.name)_<ContainerAllocator>::@(c.name);
@[ end if]@
@[end for]@
@
@[for ns in reversed(message.structure.type.namespaces)]@

}  // namespace @(ns)
@[end for]@