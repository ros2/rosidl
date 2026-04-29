// generated from rosidl_adapter/resource/msg.idl.em
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
for field in msg.fields:
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
  module msg {
@{
TEMPLATE(
    'struct.idl.em',
    msg=msg,
)
}@
  };
};

#endif  // @(include_guard)
