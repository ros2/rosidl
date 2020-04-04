@# Included from rosidl_generator_cpp/resource/idl__builder.hpp.em
@{
message_typename = '::'.join(message.structure.namespaced_type.namespaced_name())
}@

@[for ns in message.structure.namespaced_type.namespaces]@
namespace @(ns)
{

@[end for]@
@[if not message.structure.members]@
::@(message_typename) build_@(message.structure.namespaced_type.name)()
{
return ::@(message_typename)(rosidl_generator_cpp::MessageInitialization::ZERO);
}
@[else]@
namespace builder
{

@[  for index in range(len(message.structure.members) - 1, -1, -1)]@
@{
field_name = message.structure.members[index].name
if index < len(message.structure.members) - 1:
  next_field_name = message.structure.members[index + 1].name
}@
class Init_@(message.structure.namespaced_type.name)_@(field_name)
{
public:
  @
@[    if index != 0]@
explicit @
@[    end if]@
Init_@(message.structure.namespaced_type.name)_@(field_name)@
@[    if index == 0]@
()
  : msg_(::rosidl_generator_cpp::MessageInitialization::SKIP)
@[    else]@
(::@(message_typename) & msg)
  : msg_(msg)
@[    end if]@
  {}
@[    if index == len(message.structure.members) - 1]@
  ::@(message_typename) @
@[    else]@
  Init_@(message.structure.namespaced_type.name)_@(next_field_name) @
@[    end if]@
@(field_name)(::@(message_typename)::_@(field_name)_type arg)
  {
    msg_.@(field_name) = std::move(arg);
@[    if index == len(message.structure.members) - 1]@
    return std::move(msg_);
@[    else]@
    return Init_@(message.structure.namespaced_type.name)_@(next_field_name)(msg_);
@[    end if]@
  }

private:
  ::@(message_typename) msg_;
};

@[  end for]@
}  // namespace builder

builder::Init_@(message.structure.namespaced_type.name)_@(message.structure.members[0].name) build_@(message.structure.namespaced_type.name)()
{
  return builder::Init_@(message.structure.namespaced_type.name)_@(message.structure.members[0].name)();
}
@[end if]@
@
@[for ns in reversed(message.structure.namespaced_type.namespaces)]@

}  // namespace @(ns)
@[end for]@

namespace rosidl_generator_cpp
{

template<>
::@('::'.join(message.structure.namespaced_type.namespaces))::builder::Init_@(message.structure.namespaced_type.name)_@(message.structure.members[0].name)
build<::@(message_typename)>()
{
  return @('::'.join(message.structure.namespaced_type.namespaces))::builder::Init_@(message.structure.namespaced_type.name)_@(message.structure.members[0].name)();
}

}  // namespace rosidl_generator_cpp
