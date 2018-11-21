@# Included from rosidl_generator_c/resource/idl__type_support.h.em
#include "rosidl_generator_c/action_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"

#include "@(package_name)/msg/rosidl_generator_c__visibility_control.h"

// Forward declare the get type support functions for this type.
ROSIDL_GENERATOR_C_PUBLIC_@(package_name)
const rosidl_action_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__ACTION_SYMBOL_NAME(
  rosidl_typesupport_c,
  @(',\n  '.join(action.structure_type.namespaces + [action.structure_type.name]))
)();
