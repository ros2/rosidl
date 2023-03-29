// DO NOT EDIT MANUALLY - this copied file managed by copy_type_description_generated_sources.bash
// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from type_description_interfaces:msg/Field.idl
// generated code does not contain a copyright notice
#include "rosidl_runtime_c/type_description/field__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `name`
// Member `default_value`
#include "rosidl_runtime_c/string_functions.h"
// Member `type`
#include "rosidl_runtime_c/type_description/field_type__functions.h"

bool
rosidl_runtime_c__type_description__Field__init(rosidl_runtime_c__type_description__Field * msg)
{
  if (!msg) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__init(&msg->name)) {
    rosidl_runtime_c__type_description__Field__fini(msg);
    return false;
  }
  // type
  if (!rosidl_runtime_c__type_description__FieldType__init(&msg->type)) {
    rosidl_runtime_c__type_description__Field__fini(msg);
    return false;
  }
  // default_value
  if (!rosidl_runtime_c__String__init(&msg->default_value)) {
    rosidl_runtime_c__type_description__Field__fini(msg);
    return false;
  }
  return true;
}

void
rosidl_runtime_c__type_description__Field__fini(rosidl_runtime_c__type_description__Field * msg)
{
  if (!msg) {
    return;
  }
  // name
  rosidl_runtime_c__String__fini(&msg->name);
  // type
  rosidl_runtime_c__type_description__FieldType__fini(&msg->type);
  // default_value
  rosidl_runtime_c__String__fini(&msg->default_value);
}

bool
rosidl_runtime_c__type_description__Field__are_equal(const rosidl_runtime_c__type_description__Field * lhs, const rosidl_runtime_c__type_description__Field * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->name), &(rhs->name)))
  {
    return false;
  }
  // type
  if (!rosidl_runtime_c__type_description__FieldType__are_equal(
      &(lhs->type), &(rhs->type)))
  {
    return false;
  }
  // default_value
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->default_value), &(rhs->default_value)))
  {
    return false;
  }
  return true;
}

bool
rosidl_runtime_c__type_description__Field__copy(
  const rosidl_runtime_c__type_description__Field * input,
  rosidl_runtime_c__type_description__Field * output)
{
  if (!input || !output) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__copy(
      &(input->name), &(output->name)))
  {
    return false;
  }
  // type
  if (!rosidl_runtime_c__type_description__FieldType__copy(
      &(input->type), &(output->type)))
  {
    return false;
  }
  // default_value
  if (!rosidl_runtime_c__String__copy(
      &(input->default_value), &(output->default_value)))
  {
    return false;
  }
  return true;
}

rosidl_runtime_c__type_description__Field *
rosidl_runtime_c__type_description__Field__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosidl_runtime_c__type_description__Field * msg = (rosidl_runtime_c__type_description__Field *)allocator.allocate(sizeof(rosidl_runtime_c__type_description__Field), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rosidl_runtime_c__type_description__Field));
  bool success = rosidl_runtime_c__type_description__Field__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rosidl_runtime_c__type_description__Field__destroy(rosidl_runtime_c__type_description__Field * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rosidl_runtime_c__type_description__Field__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rosidl_runtime_c__type_description__Field__Sequence__init(rosidl_runtime_c__type_description__Field__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosidl_runtime_c__type_description__Field * data = NULL;

  if (size) {
    data = (rosidl_runtime_c__type_description__Field *)allocator.zero_allocate(size, sizeof(rosidl_runtime_c__type_description__Field), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rosidl_runtime_c__type_description__Field__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rosidl_runtime_c__type_description__Field__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
rosidl_runtime_c__type_description__Field__Sequence__fini(rosidl_runtime_c__type_description__Field__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      rosidl_runtime_c__type_description__Field__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

rosidl_runtime_c__type_description__Field__Sequence *
rosidl_runtime_c__type_description__Field__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosidl_runtime_c__type_description__Field__Sequence * array = (rosidl_runtime_c__type_description__Field__Sequence *)allocator.allocate(sizeof(rosidl_runtime_c__type_description__Field__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rosidl_runtime_c__type_description__Field__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rosidl_runtime_c__type_description__Field__Sequence__destroy(rosidl_runtime_c__type_description__Field__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rosidl_runtime_c__type_description__Field__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rosidl_runtime_c__type_description__Field__Sequence__are_equal(const rosidl_runtime_c__type_description__Field__Sequence * lhs, const rosidl_runtime_c__type_description__Field__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rosidl_runtime_c__type_description__Field__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rosidl_runtime_c__type_description__Field__Sequence__copy(
  const rosidl_runtime_c__type_description__Field__Sequence * input,
  rosidl_runtime_c__type_description__Field__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rosidl_runtime_c__type_description__Field);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rosidl_runtime_c__type_description__Field * data =
      (rosidl_runtime_c__type_description__Field *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rosidl_runtime_c__type_description__Field__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rosidl_runtime_c__type_description__Field__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rosidl_runtime_c__type_description__Field__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
