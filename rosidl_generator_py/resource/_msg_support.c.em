// generated from rosidl_generator_py/resource/_msg_support.c.em
// generated code does not contain a copyright notice

@#######################################################################
@# EmPy template for generating _<msg>_s.c files
@#
@# Context:
@#  - module_name
@#  - spec (rosidl_parser.MessageSpecification)
@#    Parsed specification of the .msg file
@#  - convert_camel_case_to_lower_case_underscore (function)
@#  - primitive_msg_type_to_c (function)
@#######################################################################
@
#include <Python.h>

#include <@(spec.base_type.pkg_name)/@(subfolder)/@(module_name)__struct.h>
#include <@(spec.base_type.pkg_name)/@(subfolder)/@(module_name)__functions.h>

@{
have_not_included_primitive_arrays = True
have_not_included_string = True
nested_array_dict = {}
}@
@[for field in spec.fields]@
@[  if field.type.is_array and have_not_included_primitive_arrays]@
@{have_not_included_primitive_arrays = False}@
#include <rosidl_generator_c/primitives_array.h>
#include <rosidl_generator_c/primitives_array_functions.h>

@[  end if]@
@[  if field.type.type == 'string' and have_not_included_string]@
@{have_not_included_string = False}@
#include <rosidl_generator_c/string.h>
#include <rosidl_generator_c/string_functions.h>

@[  end if]@
@{
if not field.type.is_primitive_type() and field.type.is_array:
    if field.type.type not in nested_array_dict:
        nested_array_dict[field.type.type] = field.type.pkg_name
}@
@[end for]@
@[if nested_array_dict != {}]@
// Nested array functions includes
@[  for key in nested_array_dict]@
#include <@(nested_array_dict[key])/msg/@convert_camel_case_to_lower_case_underscore(key)__functions.h>
@[  end for]@
// end nested array functions include
@[end if]@
@{
msg_typename = '%s__%s__%s' % (spec.base_type.pkg_name, subfolder, spec.base_type.type)
}@

void * @(spec.base_type.pkg_name)_@(module_name)__convert_from_py(PyObject * _pymsg)
{
  @(msg_typename) * ros_message = @(msg_typename)__create();
  (void)ros_message;
@{
full_classname = '%s.%s._%s.%s' % (spec.base_type.pkg_name, subfolder, module_name, spec.base_type.type)
}@
  char full_classname_dest[@(len(full_classname) + 1)];

  char * class_name = NULL;
  char * module_name = NULL;
  {
    PyObject * attr1 = PyObject_GetAttrString(_pymsg, "__class__");
    PyObject * attr2 = PyObject_GetAttrString(attr1, "__name__");
    class_name = (char *)PyUnicode_1BYTE_DATA(attr2);
    Py_DECREF(attr2);
    attr2 = PyObject_GetAttrString(attr1, "__module__");
    module_name = (char *)PyUnicode_1BYTE_DATA(attr2);
    Py_DECREF(attr2);
    Py_DECREF(attr1);
  }

  snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);

  assert(strncmp(
      "@(full_classname)",
      full_classname_dest, @(len(full_classname))) == 0);

@[for field in spec.fields]@
  PyObject * py@(field.name) = PyObject_GetAttrString(_pymsg, "@(field.name)");
@[  if not field.type.is_primitive_type()]@
@{
nested_type = '%s__%s__%s' % (field.type.pkg_name, 'msg', field.type.type)
}@
  typedef PyObject *(* convert_from_py_signature)(void *);
  convert_from_py_signature convert_from_py_@(field.name) = NULL;
  {
    PyObject * py@(field.name)_msg_module = PyImport_ImportModule("@(field.type.pkg_name).msg._@convert_camel_case_to_lower_case_underscore(field.type.type)");
    PyObject * py@(field.name)_msg_class = PyObject_GetAttrString(py@(field.name)_msg_module, "@(field.type.type)");
    Py_DECREF(py@(field.name)_msg_module);
    PyObject * py@(field.name)_msg_metaclass = PyObject_GetAttrString(py@(field.name)_msg_class, "__class__");
    Py_DECREF(py@(field.name)_msg_class);
    PyObject * py@(field.name)_convert_from_py = PyObject_GetAttrString(py@(field.name)_msg_metaclass, "_CONVERT_FROM_PY");
    Py_DECREF(py@(field.name)_msg_metaclass);
    convert_from_py_@(field.name) = (convert_from_py_signature)PyCapsule_GetPointer(py@(field.name)_convert_from_py, NULL);
    Py_DECREF(py@(field.name)_convert_from_py);
  }
@[    if field.type.is_array]@
  assert(PySequence_Check(py@(field.name)));
  PyObject * seq@(field.name) = PySequence_Fast(py@(field.name), "expected a sequence");
  @(nested_type) * item@(field.name);
@[      if field.type.array_size is None or field.type.is_upper_bound]@
  size_t size@(field.name) = PySequence_Size(py@(field.name));
  if (!@(nested_type)__Array__init(&(ros_message->@(field.name)), size@(field.name))) {
    PyErr_SetString(PyExc_RuntimeError, "unable to create @(nested_type)__Array ros_message");
    return NULL;
  }
  @(nested_type) * dest@(field.name) = ros_message->@(field.name).data;
@[      else]@
  size_t size@(field.name) = @(field.type.array_size);
  @(nested_type) * dest@(field.name) = ros_message->@(field.name);
@[      end if]@
  size_t idx@(field.name);
  for (idx@(field.name) = 0; idx@(field.name) < size@(field.name); idx@(field.name)++) {
    item@(field.name) = (@(nested_type) *) convert_from_py_@(field.name)(
      PySequence_Fast_GET_ITEM(seq@(field.name), idx@(field.name)));
    if (!item@(field.name)) {
      return NULL;
    }
    memcpy(&dest@(field.name)[idx@(field.name)], item@(field.name), sizeof(@(nested_type)));
  }
  Py_DECREF(seq@(field.name));
@[    else]@
  @(nested_type) * tmp@(field.name) = (@(nested_type) *) convert_from_py_@(field.name)(py@(field.name));
  if (!tmp@(field.name)) {
    return NULL;
  }
  ros_message->@(field.name) = *tmp@(field.name);
@[    end if]@
@[  elif field.type.is_array]@
  assert(PySequence_Check(py@(field.name)));
  PyObject * seq@(field.name) = PySequence_Fast(py@(field.name), "expected a sequence");
  PyObject * item@(field.name);
@[    if field.type.array_size is None or field.type.is_upper_bound]@
  size_t size@(field.name) = PySequence_Size(py@(field.name));
@[      if field.type.type == 'string']@
  if (!rosidl_generator_c__String__Array__init(&(ros_message->@(field.name)), size@(field.name))) {
    PyErr_SetString(PyExc_RuntimeError, "unable to create String__Array ros_message");
    return NULL;
  }
@[      else]@
  if (!rosidl_generator_c__@(field.type.type)__Array__init(&(ros_message->@(field.name)), size@(field.name))) {
    PyErr_SetString(PyExc_RuntimeError, "unable to create @(field.type.type)__Array ros_message");
    return NULL;
  }
@[      end if]@
  @primitive_msg_type_to_c(field.type.type) * dest@(field.name) = ros_message->@(field.name).data;
@[    else]@
  size_t size@(field.name) = @(field.type.array_size);
  @primitive_msg_type_to_c(field.type.type) * dest@(field.name) = ros_message->@(field.name);
@[    end if]@
@[    if field.type.type != 'string']@
  @primitive_msg_type_to_c(field.type.type) tmp@(field.name);
@[    end if]@
  size_t idx@(field.name);
  for (idx@(field.name) = 0; idx@(field.name) < size@(field.name); idx@(field.name)++) {
    item@(field.name) = PySequence_Fast_GET_ITEM(seq@(field.name), idx@(field.name));
@[    if field.type.type == 'char']@
    assert(PyUnicode_Check(item@(field.name)));
    tmp@(field.name) = PyUnicode_1BYTE_DATA(item@(field.name))[0];
@[    elif field.type.type == 'byte']@
    assert(PyBytes_Check(item@(field.name)));
    tmp@(field.name) = PyBytes_AS_STRING(item@(field.name))[0];
@[    elif field.type.type == 'string']@
    assert(PyUnicode_Check(item@(field.name)));
    rosidl_generator_c__String__assign(
      &dest@(field.name)[idx@(field.name)], (char *)PyUnicode_1BYTE_DATA(item@(field.name)));
@[    elif field.type.type == 'bool']@
    assert(PyBool_Check(item@(field.name)));
    tmp@(field.name) = (item@(field.name) == Py_True);
@[    elif field.type.type in ['float32', 'float64']]@
    assert(PyFloat_Check(item@(field.name)));
@[      if field.type.type == 'float32']@
    tmp@(field.name) = (float)PyFloat_AS_DOUBLE(item@(field.name));
@[      else]@
    tmp@(field.name) = PyFloat_AS_DOUBLE(item@(field.name));
@[      end if]@
@[    elif field.type.type in [
        'int8',
        'int16',
        'int32',
    ]]@
    assert(PyLong_Check(item@(field.name)));
    tmp@(field.name) = (@(primitive_msg_type_to_c(field.type.type)))PyLong_AsLong(item@(field.name));
@[    elif field.type.type in [
        'uint8',
        'uint16',
        'uint32',
    ]]@
    assert(PyLong_Check(item@(field.name)));
@[      if field.type.type == 'uint32']@
    tmp@(field.name) = PyLong_AsUnsignedLong(item@(field.name));
@[      else]@
    tmp@(field.name) = (@(primitive_msg_type_to_c(field.type.type)))PyLong_AsUnsignedLong(item@(field.name));
@[      end if]
@[    elif field.type.type == 'int64']@
    assert(PyLong_Check(item@(field.name)));
    tmp@(field.name) = PyLong_AsLongLong(item@(field.name));
@[    elif field.type.type == 'uint64']@
    assert(PyLong_Check(item@(field.name)));
    tmp@(field.name) = PyLong_AsUnsignedLongLong(item@(field.name));
@[    end if]@
@[    if field.type.type != 'string']@
    memcpy(&dest@(field.name)[idx@(field.name)], &tmp@(field.name), sizeof(@primitive_msg_type_to_c(field.type.type)));
@[    end if]@
  }
  Py_DECREF(seq@(field.name));
@[  elif field.type.type == 'char']@
  assert(PyUnicode_Check(py@(field.name)));
  ros_message->@(field.name) = PyUnicode_1BYTE_DATA(py@(field.name))[0];
@[  elif field.type.type == 'byte']@
  assert(PyBytes_Check(py@(field.name)));
  ros_message->@(field.name) = PyBytes_AS_STRING(py@(field.name))[0];
@[  elif field.type.type == 'string']@
  assert(PyUnicode_Check(py@(field.name)));
  rosidl_generator_c__String__assign(
    &ros_message->@(field.name), (char *)PyUnicode_1BYTE_DATA(py@(field.name)));
@[  elif field.type.type == 'bool']@
  assert(PyBool_Check(py@(field.name)));
  ros_message->@(field.name) = (Py_True == py@(field.name));
@[  elif field.type.type in ['float32', 'float64']]@
  assert(PyFloat_Check(py@(field.name)));
@[    if field.type.type == 'float32']@
  ros_message->@(field.name) = (float)PyFloat_AS_DOUBLE(py@(field.name));
@[    else]@
  ros_message->@(field.name) = PyFloat_AS_DOUBLE(py@(field.name));
@[    end if]@
@[  elif field.type.type in [
        'int8',
        'int16',
        'int32',
    ]]@
  assert(PyLong_Check(py@(field.name)));
  ros_message->@(field.name) = (@(primitive_msg_type_to_c(field.type.type)))PyLong_AsLong(py@(field.name));
@[  elif field.type.type in [
        'uint8',
        'uint16',
        'uint32',
    ]]@
  assert(PyLong_Check(py@(field.name)));
@[    if field.type.type == 'uint32']@
  ros_message->@(field.name) = PyLong_AsUnsignedLong(py@(field.name));
@[    else]@
  ros_message->@(field.name) = (@(primitive_msg_type_to_c(field.type.type)))PyLong_AsUnsignedLong(py@(field.name));
@[    end if]@
@[  elif field.type.type == 'int64']@
  assert(PyLong_Check(py@(field.name)));
  ros_message->@(field.name) = PyLong_AsLongLong(py@(field.name));
@[  elif field.type.type == 'uint64']@
  assert(PyLong_Check(py@(field.name)));
  ros_message->@(field.name) = PyLong_AsUnsignedLongLong(py@(field.name));
@[  else]@
  assert(false);
@[  end if]@
  Py_DECREF(py@(field.name));
@[end for]@

  assert(ros_message != NULL);
  return ros_message;
}

void @(spec.base_type.pkg_name)_@(module_name)__destroy_ros_message(void * raw_ros_message)
{
  @(msg_typename) * ros_message = (@(msg_typename) *)raw_ros_message;
  @(msg_typename)__destroy(ros_message);
}

PyObject * @(spec.base_type.pkg_name)_@(module_name)__convert_to_py(void * raw_ros_message)
{
  @(msg_typename) * ros_message = (@(msg_typename) *)raw_ros_message;
  (void)ros_message;

  /* NOTE(esteve): Call constructor of @(spec.base_type.type) */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("@(spec.base_type.pkg_name).@(subfolder)._@(module_name)");
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "@(spec.base_type.type)");
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
  }
  assert(_pymessage != NULL);

@[for field in spec.fields]@
  PyObject * py@(field.name) = NULL;
@[  if not field.type.is_primitive_type()]@
@{
nested_type = '%s__%s__%s' % (field.type.pkg_name, 'msg', field.type.type)
}@
  PyObject * py@(field.name)_msg_module = PyImport_ImportModule("@(field.type.pkg_name).msg._@convert_camel_case_to_lower_case_underscore(field.type.type)");
  PyObject * py@(field.name)_msg_class = PyObject_GetAttrString(py@(field.name)_msg_module, "@(field.type.type)");
  PyObject * py@(field.name)_msg_metaclass = PyObject_GetAttrString(py@(field.name)_msg_class, "__class__");
  PyObject * py@(field.name)_convert_to_py = PyObject_GetAttrString(py@(field.name)_msg_metaclass, "_CONVERT_TO_PY");
  typedef PyObject *(* convert_to_py_signature)(void *);
  convert_to_py_signature convert_to_py_@(field.name) = (convert_to_py_signature)PyCapsule_GetPointer(py@(field.name)_convert_to_py, NULL);
@[    if field.type.is_array]@
@[      if field.type.array_size is None or field.type.is_upper_bound]@
  size_t size@(field.name) = ros_message->@(field.name).size;
@[      else]@
  size_t size@(field.name) = @(field.type.array_size);
@[      end if]@
  py@(field.name) = PyList_New(size@(field.name));
  @(nested_type) * item@(field.name);
  size_t idx@(field.name);
  for (idx@(field.name) = 0; idx@(field.name) < size@(field.name); idx@(field.name)++) {
@[      if field.type.array_size is None or field.type.is_upper_bound]@
    item@(field.name) = &(ros_message->@(field.name).data[idx@(field.name)]);
@[      else]@
    item@(field.name) = &(ros_message->@(field.name)[idx@(field.name)]);
@[      end if]@
    PyObject * pyitem@(field.name) = convert_to_py_@(field.name)(item@(field.name));
    if (!pyitem@(field.name)) {
      return NULL;
    }
    PyList_SetItem(py@(field.name), idx@(field.name), pyitem@(field.name));
  }
  assert(PySequence_Check(py@(field.name)));
@[    else]@
  @(nested_type) pytmp@(field.name) = ros_message->@(field.name);
  py@(field.name) = convert_to_py_@(field.name)(&pytmp@(field.name));
  if (!py@(field.name)) {
    return NULL;
  }
@[    end if]@
@[  elif field.type.is_array]@
@[    if field.type.array_size is None or field.type.is_upper_bound]@
  size_t size@(field.name) = ros_message->@(field.name).size;
  @primitive_msg_type_to_c(field.type.type) * tmpmessagedata@(field.name) = ros_message->@(field.name).data;
@[    else]@
  size_t size@(field.name) = @(field.type.array_size);
  @primitive_msg_type_to_c(field.type.type) * tmpmessagedata@(field.name) = ros_message->@(field.name);
@[    end if]@
  py@(field.name) = PyList_New(size@(field.name));
  size_t idx@(field.name);
  for (idx@(field.name) = 0; idx@(field.name) < size@(field.name); idx@(field.name)++) {
@[    if field.type.type == 'char']@
    PyList_SetItem(py@(field.name), idx@(field.name),
      Py_BuildValue("C", tmpmessagedata@(field.name)[idx@(field.name)]));
@[    elif field.type.type == 'byte']@
    PyList_SetItem(py@(field.name), idx@(field.name),
      PyBytes_FromStringAndSize((const char *)&tmpmessagedata@(field.name)[idx@(field.name)], 1));
@[    elif field.type.type == 'string']@
    PyList_SetItem(py@(field.name), idx@(field.name), PyUnicode_FromString(tmpmessagedata@(field.name)[idx@(field.name)].data));
@[    elif field.type.type == 'bool']@
@# using PyBool_FromLong because PyList_SetItem will steal ownership of the passed item
    PyList_SetItem(py@(field.name), idx@(field.name), PyBool_FromLong(tmpmessagedata@(field.name)[idx@(field.name)] ? 1 : 0));
@[    elif field.type.type in ['float32', 'float64']]@
    PyList_SetItem(py@(field.name), idx@(field.name), PyFloat_FromDouble(tmpmessagedata@(field.name)[idx@(field.name)]));
@[    elif field.type.type in [
        'int8',
        'int16',
        'int32',
    ]]@
    PyList_SetItem(py@(field.name), idx@(field.name), PyLong_FromLong(tmpmessagedata@(field.name)[idx@(field.name)]));
@[    elif field.type.type in [
        'uint8',
        'uint16',
        'uint32',
    ]]@
    PyList_SetItem(py@(field.name), idx@(field.name), PyLong_FromUnsignedLong(tmpmessagedata@(field.name)[idx@(field.name)]));
@[    elif field.type.type == 'int64']@
    PyList_SetItem(py@(field.name), idx@(field.name), PyLong_FromLongLong(tmpmessagedata@(field.name)[idx@(field.name)]));
@[    elif field.type.type == 'uint64']@
    PyList_SetItem(py@(field.name), idx@(field.name), PyLong_FromUnsignedLongLong(tmpmessagedata@(field.name)[idx@(field.name)]));
@[    end if]@
  }
  assert(PySequence_Check(py@(field.name)));
@[  elif field.type.type == 'char']@
  py@(field.name) = Py_BuildValue("C", ros_message->@(field.name));
@[  elif field.type.type == 'byte']@
  py@(field.name) = PyBytes_FromStringAndSize((const char *)&ros_message->@(field.name), 1);
@[  elif field.type.type == 'string']@
  py@(field.name) = PyUnicode_FromString(ros_message->@(field.name).data);
@[  elif field.type.type == 'bool']@
@# using PyBool_FromLong allows treating the variable uniformly by calling Py_DECREF on it later
  py@(field.name) = PyBool_FromLong(ros_message->@(field.name) ? 1 : 0);
@[  elif field.type.type in ['float32', 'float64']]@
  py@(field.name) = PyFloat_FromDouble(ros_message->@(field.name));
@[  elif field.type.type in [
        'int8',
        'int16',
        'int32',
    ]]@
  py@(field.name) = PyLong_FromLong(ros_message->@(field.name));
@[  elif field.type.type in [
        'uint8',
        'uint16',
        'uint32',
    ]]@
  py@(field.name) = PyLong_FromUnsignedLong(ros_message->@(field.name));
@[  elif field.type.type == 'int64']@
  py@(field.name) = PyLong_FromLongLong(ros_message->@(field.name));
@[  elif field.type.type == 'uint64']@
  py@(field.name) = PyLong_FromUnsignedLongLong(ros_message->@(field.name));
@[  else]@
  assert(false);
@[  end if]@
  assert(py@(field.name) != NULL);
  PyObject_SetAttrString(_pymessage, "@(field.name)", py@(field.name));
  Py_DECREF(py@(field.name));
@[end for]@
  assert(_pymessage != NULL);
  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
