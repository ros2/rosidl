// generated from rosidl_adapter/resource/action.idl.em
// with input from @(pkg_name)/@(relative_input_file)
// generated code does not contain a copyright notice

@{
import re
from rosidl_adapter.msg import get_include_file

include_guard = re.sub(r'[^0-9A-Za-z]+', '_', f'{pkg_name}_{relative_input_file}')
include_guard = include_guard.strip('_').upper()
if not include_guard or not include_guard[0].isalpha():
    include_guard = f'ROSIDL_{include_guard}'
include_files = set()
fields = action.goal.fields + action.result.fields + action.feedback.fields
for field in fields:
    include_file = get_include_file(field.type)
    if include_file is not None:
        include_files.add(include_file)
}@
#ifndef @(include_guard)
#define @(include_guard)

@[if include_files]@
@[for include_file in sorted(include_files)]@
#include "@(include_file)"
@[end for]@

@[end if]@
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
