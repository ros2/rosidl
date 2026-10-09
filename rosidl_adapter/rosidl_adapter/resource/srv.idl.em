// generated from rosidl_adapter/resource/srv.idl.em
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
for field in srv.request.fields + srv.response.fields:
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
  module srv {
@{
TEMPLATE(
    'struct.idl.em',
    msg=srv.request,
)
}@
@{
TEMPLATE(
    'struct.idl.em',
    msg=srv.response,
)
}@
  };
};

#endif  // @(include_guard)
