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
@{
full_classname = '%s.%s._%s.%s' % (spec.base_type.pkg_name, subfolder, module_name, spec.base_type.type)
}@
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[@(len(full_classname) + 1)];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return NULL;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp(
        "@(full_classname)",
        full_classname_dest, @(len(full_classname))) == 0);
  }
  @(msg_typename) * ros_message = @(msg_typename)__create();
@[if not spec.fields]@
  (void)ros_message;
@[end if]@
@[for field in spec.fields]@
  {  // @(field.name)
    PyObject * field = PyObject_GetAttrString(_pymsg, "@(field.name)");
    if (!field) {
      return NULL;
    }
@[  if not field.type.is_primitive_type()]@
@{
nested_type = '%s__%s__%s' % (field.type.pkg_name, 'msg', field.type.type)
}@
    typedef PyObject *(* convert_from_py_signature)(void *);
    convert_from_py_signature convert_from_py = NULL;
    // get conversion function
    {
      PyObject * msg_module = PyImport_ImportModule("@(field.type.pkg_name).msg._@convert_camel_case_to_lower_case_underscore(field.type.type)");
      if (!msg_module) {
        Py_DECREF(field);
        return NULL;
      }
      PyObject * msg_class = PyObject_GetAttrString(msg_module, "@(field.type.type)");
      Py_DECREF(msg_module);
      if (!msg_class) {
        Py_DECREF(field);
        return NULL;
      }
      PyObject * msg_metaclass = PyObject_GetAttrString(msg_class, "__class__");
      Py_DECREF(msg_class);
      if (!msg_metaclass) {
        Py_DECREF(field);
        return NULL;
      }
      PyObject * convert_from_py_ = PyObject_GetAttrString(msg_metaclass, "_CONVERT_FROM_PY");
      Py_DECREF(msg_metaclass);
      if (!convert_from_py_) {
        Py_DECREF(field);
        return NULL;
      }
      convert_from_py = (convert_from_py_signature)PyCapsule_GetPointer(convert_from_py_, NULL);
      Py_DECREF(convert_from_py_);
      if (!convert_from_py) {
        Py_DECREF(field);
        return NULL;
      }
    }
@[    if field.type.is_array]@
    PyObject * seq_field = PySequence_Fast(field, "expected a sequence in '@(field.name)'");
    if (!seq_field) {
      Py_DECREF(field);
      return NULL;
    }
@[      if field.type.array_size is None or field.type.is_upper_bound]@
    Py_ssize_t size = PySequence_Size(field);
    if (-1 == size) {
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return NULL;
    }
    if (!@(nested_type)__Array__init(&(ros_message->@(field.name)), size)) {
      PyErr_SetString(PyExc_RuntimeError, "unable to create @(nested_type)__Array ros_message");
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return NULL;
    }
    @(nested_type) * dest = ros_message->@(field.name).data;
@[      else]@
    Py_ssize_t size = @(field.type.array_size);
    @(nested_type) * dest = ros_message->@(field.name);
@[      end if]@
    for (Py_ssize_t i = 0; i < size; ++i) {
      @(nested_type) * item = (@(nested_type) *) convert_from_py(
        PySequence_Fast_GET_ITEM(seq_field, i));
      if (!item) {
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return NULL;
      }
      memcpy(&dest[i], item, sizeof(@(nested_type)));
    }
    Py_DECREF(seq_field);
@[    else]@
    @(nested_type) * tmp = (@(nested_type) *) convert_from_py(field);
    if (!tmp) {
      return NULL;
    }
    ros_message->@(field.name) = *tmp;
@[    end if]@
@[  elif field.type.is_array]@
    PyObject * seq_field = PySequence_Fast(field, "expected a sequence in '@(field.name)'");
    if (!seq_field) {
      Py_DECREF(field);
      return NULL;
    }
@[    if field.type.array_size is None or field.type.is_upper_bound]@
    Py_ssize_t size = PySequence_Size(field);
    if (-1 == size) {
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return NULL;
    }
@[      if field.type.type == 'string']@
    if (!rosidl_generator_c__String__Array__init(&(ros_message->@(field.name)), size)) {
      PyErr_SetString(PyExc_RuntimeError, "unable to create String__Array ros_message");
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return NULL;
    }
@[      else]@
    if (!rosidl_generator_c__@(field.type.type)__Array__init(&(ros_message->@(field.name)), size)) {
      PyErr_SetString(PyExc_RuntimeError, "unable to create @(field.type.type)__Array ros_message");
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return NULL;
    }
@[      end if]@
    @primitive_msg_type_to_c(field.type.type) * dest = ros_message->@(field.name).data;
@[    else]@
    Py_ssize_t size = @(field.type.array_size);
    @primitive_msg_type_to_c(field.type.type) * dest = ros_message->@(field.name);
@[    end if]@
    for (Py_ssize_t i = 0; i < size; ++i) {
      PyObject * item = PySequence_Fast_GET_ITEM(seq_field, i);
      if (!item) {
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return NULL;
      }
@[    if field.type.type == 'char']@
      assert(PyUnicode_Check(item));
      @primitive_msg_type_to_c(field.type.type) tmp = PyUnicode_1BYTE_DATA(item)[0];
@[    elif field.type.type == 'byte']@
      assert(PyBytes_Check(item));
      @primitive_msg_type_to_c(field.type.type) tmp = PyBytes_AS_STRING(item)[0];
@[    elif field.type.type == 'string']@
      assert(PyUnicode_Check(item));
      rosidl_generator_c__String__assign(&dest[i], (char *)PyUnicode_1BYTE_DATA(item));
@[    elif field.type.type == 'bool']@
      assert(PyBool_Check(item));
      @primitive_msg_type_to_c(field.type.type) tmp = (item == Py_True);
@[    elif field.type.type in ['float32', 'float64']]@
      assert(PyFloat_Check(item));
@[      if field.type.type == 'float32']@
      @primitive_msg_type_to_c(field.type.type) tmp = (float)PyFloat_AS_DOUBLE(item);
@[      else]@
      @primitive_msg_type_to_c(field.type.type) tmp = PyFloat_AS_DOUBLE(item);
@[      end if]@
@[    elif field.type.type in [
        'int8',
        'int16',
        'int32',
    ]]@
      assert(PyLong_Check(item));
      @primitive_msg_type_to_c(field.type.type) tmp = (@(primitive_msg_type_to_c(field.type.type)))PyLong_AsLong(item);
@[    elif field.type.type in [
        'uint8',
        'uint16',
        'uint32',
    ]]@
      assert(PyLong_Check(item));
@[      if field.type.type == 'uint32']@
      @primitive_msg_type_to_c(field.type.type) tmp = PyLong_AsUnsignedLong(item);
@[      else]@
      @primitive_msg_type_to_c(field.type.type) tmp = (@(primitive_msg_type_to_c(field.type.type)))PyLong_AsUnsignedLong(item);
@[      end if]
@[    elif field.type.type == 'int64']@
      assert(PyLong_Check(item));
      @primitive_msg_type_to_c(field.type.type) tmp = PyLong_AsLongLong(item);
@[    elif field.type.type == 'uint64']@
      assert(PyLong_Check(item));
      @primitive_msg_type_to_c(field.type.type) tmp = PyLong_AsUnsignedLongLong(item);
@[    end if]@
@[    if field.type.type != 'string']@
      memcpy(&dest[i], &tmp, sizeof(@primitive_msg_type_to_c(field.type.type)));
@[    end if]@
    }
    Py_DECREF(seq_field);
@[  elif field.type.type == 'char']@
    assert(PyUnicode_Check(field));
    ros_message->@(field.name) = PyUnicode_1BYTE_DATA(field)[0];
@[  elif field.type.type == 'byte']@
    assert(PyBytes_Check(field));
    ros_message->@(field.name) = PyBytes_AS_STRING(field)[0];
@[  elif field.type.type == 'string']@
    assert(PyUnicode_Check(field));
    rosidl_generator_c__String__assign(&ros_message->@(field.name), (char *)PyUnicode_1BYTE_DATA(field));
@[  elif field.type.type == 'bool']@
    assert(PyBool_Check(field));
    ros_message->@(field.name) = (Py_True == field);
@[  elif field.type.type in ['float32', 'float64']]@
    assert(PyFloat_Check(field));
@[    if field.type.type == 'float32']@
    ros_message->@(field.name) = (float)PyFloat_AS_DOUBLE(field);
@[    else]@
    ros_message->@(field.name) = PyFloat_AS_DOUBLE(field);
@[    end if]@
@[  elif field.type.type in [
        'int8',
        'int16',
        'int32',
    ]]@
    assert(PyLong_Check(field));
    ros_message->@(field.name) = (@(primitive_msg_type_to_c(field.type.type)))PyLong_AsLong(field);
@[  elif field.type.type in [
        'uint8',
        'uint16',
        'uint32',
    ]]@
    assert(PyLong_Check(field));
@[    if field.type.type == 'uint32']@
    ros_message->@(field.name) = PyLong_AsUnsignedLong(field);
@[    else]@
    ros_message->@(field.name) = (@(primitive_msg_type_to_c(field.type.type)))PyLong_AsUnsignedLong(field);
@[    end if]@
@[  elif field.type.type == 'int64']@
    assert(PyLong_Check(field));
    ros_message->@(field.name) = PyLong_AsLongLong(field);
@[  elif field.type.type == 'uint64']@
    assert(PyLong_Check(field));
    ros_message->@(field.name) = PyLong_AsUnsignedLongLong(field);
@[  else]@
    assert(false);
@[  end if]@
    Py_DECREF(field);
  }
@[end for]@

  return ros_message;
}

void @(spec.base_type.pkg_name)_@(module_name)__destroy_ros_message(void * raw_ros_message)
{
  @(msg_typename) * ros_message = (@(msg_typename) *)raw_ros_message;
  @(msg_typename)__destroy(ros_message);
}

PyObject * @(spec.base_type.pkg_name)_@(module_name)__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of @(spec.base_type.type) */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("@(spec.base_type.pkg_name).@(subfolder)._@(module_name)");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "@(spec.base_type.type)");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  @(msg_typename) * ros_message = (@(msg_typename) *)raw_ros_message;
@[if not spec.fields]@
  (void)ros_message;
@[end if]@
@[for field in spec.fields]@
  {  // @(field.name)
    PyObject * field = NULL;
@[  if not field.type.is_primitive_type()]@
@{
nested_type = '%s__%s__%s' % (field.type.pkg_name, 'msg', field.type.type)
}@
    typedef PyObject *(* convert_to_py_signature)(void *);
    convert_to_py_signature convert_to_py = NULL;
    // get conversion function
    {
      PyObject * msg_module = PyImport_ImportModule("@(field.type.pkg_name).msg._@convert_camel_case_to_lower_case_underscore(field.type.type)");
      if (!msg_module) {
        return NULL;
      }
      PyObject * msg_class = PyObject_GetAttrString(msg_module, "@(field.type.type)");
      Py_DECREF(msg_module);
      if (!msg_class) {
        return NULL;
      }
      PyObject * msg_metaclass = PyObject_GetAttrString(msg_class, "__class__");
      Py_DECREF(msg_class);
      if (!msg_metaclass) {
        return NULL;
      }
      PyObject * convert_to_py_ = PyObject_GetAttrString(msg_metaclass, "_CONVERT_TO_PY");
      Py_DECREF(msg_metaclass);
      if (!convert_to_py_) {
        return NULL;
      }
      convert_to_py = (convert_to_py_signature)PyCapsule_GetPointer(convert_to_py_, NULL);
      Py_DECREF(convert_to_py_);
      if (!convert_to_py) {
        return NULL;
      }
    }
@[    if field.type.is_array]@
@[      if field.type.array_size is None or field.type.is_upper_bound]@
    size_t size = ros_message->@(field.name).size;
@[      else]@
    size_t size = @(field.type.array_size);
@[      end if]@
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    @(nested_type) * item;
    for (size_t i = 0; i < size; ++i) {
@[      if field.type.array_size is None or field.type.is_upper_bound]@
      item = &(ros_message->@(field.name).data[i]);
@[      else]@
      item = &(ros_message->@(field.name)[i]);
@[      end if]@
      PyObject * pyitem = convert_to_py(item);
      if (!pyitem) {
        Py_DECREF(field);
        return NULL;
      }
      int rc = PyList_SetItem(field, i, pyitem);
      (void)rc;
      assert(rc == 0);
    }
    assert(PySequence_Check(field));
@[    else]@
    field = convert_to_py(&ros_message->@(field.name));
    if (!field) {
      return NULL;
    }
@[    end if]@
@[  elif field.type.is_array]@
@[    if field.type.array_size is None or field.type.is_upper_bound]@
    size_t size = ros_message->@(field.name).size;
    @primitive_msg_type_to_c(field.type.type) * src = ros_message->@(field.name).data;
@[    else]@
    size_t size = @(field.type.array_size);
    @primitive_msg_type_to_c(field.type.type) * src = ros_message->@(field.name);
@[    end if]@
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    for (size_t i = 0; i < size; ++i) {
@[    if field.type.type == 'char']@
      int rc = PyList_SetItem(field, i, Py_BuildValue("C", src[i]));
      (void)rc;
      assert(rc == 0);
@[    elif field.type.type == 'byte']@
      int rc = PyList_SetItem(field, i, PyBytes_FromStringAndSize((const char *)&src[i], 1));
      (void)rc;
      assert(rc == 0);
@[    elif field.type.type == 'string']@
      int rc = PyList_SetItem(field, i, PyUnicode_FromString(src[i].data));
      (void)rc;
      assert(rc == 0);
@[    elif field.type.type == 'bool']@
@# using PyBool_FromLong because PyList_SetItem will steal ownership of the passed item
      int rc = PyList_SetItem(field, i, PyBool_FromLong(src[i] ? 1 : 0));
      (void)rc;
      assert(rc == 0);
@[    elif field.type.type in ['float32', 'float64']]@
      int rc = PyList_SetItem(field, i, PyFloat_FromDouble(src[i]));
      (void)rc;
      assert(rc == 0);
@[    elif field.type.type in [
        'int8',
        'int16',
        'int32',
    ]]@
      int rc = PyList_SetItem(field, i, PyLong_FromLong(src[i]));
      (void)rc;
      assert(rc == 0);
@[    elif field.type.type in [
        'uint8',
        'uint16',
        'uint32',
    ]]@
      int rc = PyList_SetItem(field, i, PyLong_FromUnsignedLong(src[i]));
      (void)rc;
      assert(rc == 0);
@[    elif field.type.type == 'int64']@
      int rc = PyList_SetItem(field, i, PyLong_FromLongLong(src[i]));
      (void)rc;
      assert(rc == 0);
@[    elif field.type.type == 'uint64']@
      int rc = PyList_SetItem(field, i, PyLong_FromUnsignedLongLong(src[i]));
      (void)rc;
      assert(rc == 0);
@[    end if]@
    }
    assert(PySequence_Check(field));
@[  elif field.type.type == 'char']@
    field = Py_BuildValue("C", ros_message->@(field.name));
    if (!field) {
      return NULL;
    }
@[  elif field.type.type == 'byte']@
    field = PyBytes_FromStringAndSize((const char *)&ros_message->@(field.name), 1);
    if (!field) {
      return NULL;
    }
@[  elif field.type.type == 'string']@
    field = PyUnicode_FromString(ros_message->@(field.name).data);
@[  elif field.type.type == 'bool']@
@# using PyBool_FromLong allows treating the variable uniformly by calling Py_DECREF on it later
    field = PyBool_FromLong(ros_message->@(field.name) ? 1 : 0);
@[  elif field.type.type in ['float32', 'float64']]@
    field = PyFloat_FromDouble(ros_message->@(field.name));
@[  elif field.type.type in [
        'int8',
        'int16',
        'int32',
    ]]@
    field = PyLong_FromLong(ros_message->@(field.name));
@[  elif field.type.type in [
        'uint8',
        'uint16',
        'uint32',
    ]]@
    field = PyLong_FromUnsignedLong(ros_message->@(field.name));
@[  elif field.type.type == 'int64']@
    field = PyLong_FromLongLong(ros_message->@(field.name));
@[  elif field.type.type == 'uint64']@
    field = PyLong_FromUnsignedLongLong(ros_message->@(field.name));
@[  else]@
    assert(false);
@[  end if]@
    {
      int rc = PyObject_SetAttrString(_pymessage, "@(field.name)", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
@[end for]@

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
