// generated from rosidl_adapter/resource/action.idl.em
// with input from @(pkg_name)/@(relative_input_file)
// generated code does not contain a copyright notice

@{
import re
from rosidl_adapter.msg import get_include_file

include_guard_parts = [
    re.sub(r'[^0-9A-Za-z]+', '_', part).strip('_').upper()
    for part in (pkg_name, *relative_input_file.parts)
]
assert all(include_guard_parts)
include_guard = '__'.join(include_guard_parts)
assert include_guard[0].isalpha()
include_files = set()
fields = action.goal.fields + action.result.fields + action.feedback.fields
for field in fields:
    include_file = get_include_file(field.type)
    if include_file is not None:
        include_files.add(include_file)
}@
#ifndef @(include_guard)
#define @(include_guard)

@[for include_file in sorted(include_files)]@
#include "@(include_file)"
@[end for]@

module @(pkg_name) {
  module action {
@{
TEMPLATE(
    'struct.idl.em',
    msg=action.goal,
)
}@
@{
TEMPLATE(
    'struct.idl.em',
    msg=action.result,
)
}@
@{
TEMPLATE(
    'struct.idl.em',
    msg=action.feedback,
)
}@
  };
};

#endif  // @(include_guard)
