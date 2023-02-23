@# Included from rosidl_generator_cpp/resource/idl__description.hpp.em
@
@#######################################################################
@# EmPy template for generating get_type_description function for types
@#
@# Context:
@#  - namespaced_type (NamespacedType)
@#  - type_description_msg (dict representation of TypeDescription.msg)
@#######################################################################
@{
from rosidl_generator_cpp import escape_string
toplevel_type_description = type_description_msg['type_description']
all_type_descriptions = [toplevel_type_description] + type_description_msg['referenced_type_descriptions']

def typename_to_individual_varname(typename):
  return typename.replace('/', '__') + '__individual'
}@

namespace rosidl_runtime_cpp
{

template<>
inline const rosidl_runtime_cpp::type_description::TypeDescription &
get_type_description<@('::'.join(namespaced_type.namespaced_name()))>() {
  static bool constructed = false;
  static rosidl_runtime_cpp::type_description::TypeDescription instance;
  if (constructed) {
    return instance;
  }
@[  for itd in all_type_descriptions]@
@{ itd_var = typename_to_individual_varname(itd['type_name']) }@
  static rosidl_runtime_cpp::type_description::IndividualTypeDescription @(itd_var);
  @(itd_var).type_name = "@(itd['type_name'])";
  @(itd_var).fields.resize(@(len(itd['fields'])));
@[    for idx, field in enumerate(itd['fields'])]@
  @(itd_var).fields[@(idx)].name = "@(field['name'])";
  @(itd_var).fields[@(idx)].type.type_id = @(field['type']['type_id']);
  @(itd_var).fields[@(idx)].type.capacity = @(field['type']['capacity']);
  @(itd_var).fields[@(idx)].type.string_capacity = @(field['type']['string_capacity']);
  @(itd_var).fields[@(idx)].type.nested_type_name = "@(field['type']['nested_type_name'])";
  @(itd_var).fields[@(idx)].default_value = "@(escape_string(field['default_value']))";
@[    end for]@

@[  end for]@

  instance.type_description = @(typename_to_individual_varname(toplevel_type_description['type_name']));
  instance.referenced_type_descriptions = {
@[  for ref_itd in type_description_msg['referenced_type_descriptions']]@
      @(typename_to_individual_varname(ref_itd['type_name'])),
@[  end for]@
  };

  constructed = true;
  return instance;
}

}  // namespace rosidl_runtime_cpp
