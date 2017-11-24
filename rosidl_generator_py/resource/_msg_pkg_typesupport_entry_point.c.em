// generated from rosidl_generator_py/resource/_msg_pkg_typesupport_entry_point.c.em
// generated code does not contain a copyright notice

@#######################################################################
@# EmPy template for generating _<msg_pkg>_s.ep.<typesupport_impl>_c.c files
@#
@# Context:
@#  - package_name
@#  - message_specs (list of rosidl_parser.MessageSpecification)
@#    Parsed specification of the .msg files
@#  - service_specs (list of rosidl_parser.ServiceSpecification)
@#    Parsed specification of the .srv files
@#  - typesupport_impl (string identifying the typesupport used)
@#  - convert_camel_case_to_lower_case_underscore (function)
@#######################################################################
@
#include <Python.h>
#include <stdint.h>

@{
static_includes = set([
    '#include <rosidl_generator_c/message_type_support_struct.h>',
    '#include <rosidl_generator_c/visibility_control.h>',
])
for spec, subfolder in message_specs:
  if subfolder == 'msg':
    static_includes.add('#include <rosidl_generator_c/message_type_support_struct.h>')
  elif subfolder == 'srv':
    static_includes.add('#include <rosidl_generator_c/service_type_support_struct.h>')
}@
@[for value in sorted(static_includes)]@
@(value)
@[end for]@

@{
includes = {}
for spec, subfolder in message_specs:
  type_name = spec.base_type.type
  module_name = convert_camel_case_to_lower_case_underscore(type_name)
  key = '%s/%s/%s' % (spec.base_type.pkg_name, subfolder, module_name)
  includes[key] = '#include <%s__type_support.h>' % key

for spec, subfolder in service_specs:
  type_name = convert_camel_case_to_lower_case_underscore(spec.srv_name)
  module_name = convert_camel_case_to_lower_case_underscore(type_name)
  key = '%s/%s/%s' % (spec.pkg_name, subfolder, module_name)
  includes[key] = '#include <%s.h>' % key
}@
@[for v in sorted(includes.values())]@
@(v)
@[end for]@

@[for spec, subfolder in message_specs]@
@{
type_name = spec.base_type.type
module_name = convert_camel_case_to_lower_case_underscore(type_name)
}@
void * @(spec.base_type.pkg_name)_@(module_name)__convert_from_py(PyObject * _pymsg);
void @(spec.base_type.pkg_name)_@(module_name)__destroy_ros_message(void * raw_ros_message);
PyObject * @(spec.base_type.pkg_name)_@(module_name)__convert_to_py(void * raw_ros_message);
@[end for]@

static PyMethodDef @(package_name)__methods[] = {
  {NULL, NULL, 0, NULL}  /* sentinel */
};

static struct PyModuleDef @(package_name)__module = {
  PyModuleDef_HEAD_INIT,
  "_@(package_name)_support",
  "_@(package_name)_doc",
  -1,  /* -1 means that the module keeps state in global variables */
  @(package_name)__methods,
  NULL,
  NULL,
  NULL,
  NULL,
};
@
@[for spec, subfolder in message_specs]@
@{
type_name = convert_camel_case_to_lower_case_underscore(spec.base_type.type)
function_names = ['convert_from_py', 'destroy_ros_message', 'convert_to_py', 'type_support']
}@

ROSIDL_GENERATOR_C_IMPORT
const rosidl_message_type_support_t *
ROSIDL_GET_MSG_TYPE_SUPPORT(@(spec.base_type.pkg_name), @(subfolder), @(spec.msg_name));

int8_t
_register_msg_type__@(type_name)(PyObject * pymodule)
{
  int8_t err;
@[  for function_name in function_names]@

  PyObject * pyobject_@(function_name) = NULL;
  pyobject_@(function_name) = PyCapsule_New(
@[    if function_name != 'type_support']@
    (void *)&@(spec.base_type.pkg_name)_@(type_name)__@(function_name),
@[    else]@
    (void *)ROSIDL_GET_MSG_TYPE_SUPPORT(@(spec.base_type.pkg_name), @(subfolder), @(spec.msg_name)),
@[    end if]@
    NULL, NULL);
  if (!pyobject_@(function_name)) {
    // previously added objects will be removed when the module is destroyed
    return -1;
  }
  err = PyModule_AddObject(
    pymodule,
    "@(function_name)_msg_@(type_name)",
    pyobject_@(function_name));
  if (err) {
    // the created capsule needs to be decremented
    Py_XDECREF(pyobject_@(function_name));
    // previously added objects will be removed when the module is destroyed
    return err;
  }
@[  end for]@
  return 0;
}
@[end for]@
@[for spec, subfolder in service_specs]@
@{
type_name = convert_camel_case_to_lower_case_underscore(spec.srv_name)
function_name = 'type_support'
}@

ROSIDL_GENERATOR_C_IMPORT
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, @(spec.pkg_name), @(spec.srv_name))();

int8_t
_register_srv_type__@(type_name)(PyObject * pymodule)
{
  int8_t err;
  PyObject * pyobject_@(function_name) = NULL;
  pyobject_@(function_name) = PyCapsule_New(
    (void *)ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, @(spec.pkg_name), @(spec.srv_name))(),
    NULL, NULL);
  if (!pyobject_@(function_name)) {
    // previously added objects will be removed when the module is destroyed
    return -1;
  }
  err = PyModule_AddObject(
    pymodule,
    "@(function_name)_srv_@(type_name)",
    pyobject_@(function_name));
  if (err) {
    // the created capsule needs to be decremented
    Py_XDECREF(pyobject_@(function_name));
    // previously added objects will be removed when the module is destroyed
    return err;
  }
  return 0;
}
@[end for]@

PyMODINIT_FUNC
PyInit_@(package_name)_s__@(typesupport_impl)(void)
{
  PyObject * pymodule = NULL;
  pymodule = PyModule_Create(&@(package_name)__module);
  if (!pymodule) {
    return NULL;
  }
  int8_t err;
@[for spec, subfolder in message_specs]@
@{
type_name = convert_camel_case_to_lower_case_underscore(spec.base_type.type)
}@
  err = _register_msg_type__@(type_name)(pymodule);
  if (err) {
    Py_XDECREF(pymodule);
    return NULL;
  }
@[end for]@
@[for spec, subfolder in service_specs]@
@{
type_name = convert_camel_case_to_lower_case_underscore(spec.srv_name)
}@
  err = _register_srv_type__@(type_name)(pymodule);
  if (err) {
    Py_XDECREF(pymodule);
    return NULL;
  }
@[end for]@

  return pymodule;
}
