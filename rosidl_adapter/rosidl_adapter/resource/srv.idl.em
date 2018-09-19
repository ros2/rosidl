// generated from rosidl_adapter/resource/srv.idl.em
// with input from @(pkg_name)/@(relative_input_file)

@{
include_files = set()
for field in srv.request.fields + srv.response.fields:
    include_file = get_include_file(field.type)
    if include_file is not None:
        include_files.add(include_file)
}@
@[for include_file in sorted(include_files)]@
#include "@(include_file)"
@[end for]@

module @(pkg_name) {
  module msg {
@{
TEMPLATE(
    'struct.idl.em',
    msg=srv.request,
    get_idl_type=get_idl_type)
}@
@{
TEMPLATE(
    'struct.idl.em',
    msg=srv.response,
    get_idl_type=get_idl_type)
}@
  };
};
