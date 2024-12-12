// DO NOT EDIT MANUALLY - this copied file managed by copy_type_description_generated_sources.bash
// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from type_description_interfaces:msg/FieldType.idl
// generated code does not contain a copyright notice
#include "rosidl_runtime_c/type_description/field_type__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `nested_type_name`
#include "rosidl_runtime_c/string_functions.h"

bool
rosidl_runtime_c__type_description__FieldType__init(rosidl_runtime_c__type_description__FieldType * msg)
{
  if (!msg) {
    return false;
  }
  // type_id
  msg->type_id = 0;
  // capacity
  // string_capacity
  // nested_type_name
  if (!rosidl_runtime_c__String__init(&msg->nested_type_name)) {
    rosidl_runtime_c__type_description__FieldType__fini(msg);
    return false;
  }
  return true;
}

void
rosidl_runtime_c__type_description__FieldType__fini(rosidl_runtime_c__type_description__FieldType * msg)
{
  if (!msg) {
    return;
  }
  // type_id
  // capacity
  // string_capacity
  // nested_type_name
  rosidl_runtime_c__String__fini(&msg->nested_type_name);
}

bool
rosidl_runtime_c__type_description__FieldType__are_equal(const rosidl_runtime_c__type_description__FieldType * lhs, const rosidl_runtime_c__type_description__FieldType * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // type_id
  if (lhs->type_id != rhs->type_id) {
    return false;
  }
  // capacity
  if (lhs->capacity != rhs->capacity) {
    return false;
  }
  // string_capacity
  if (lhs->string_capacity != rhs->string_capacity) {
    return false;
  }
  // nested_type_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->nested_type_name), &(rhs->nested_type_name)))
  {
    return false;
  }
  return true;
}

bool
rosidl_runtime_c__type_description__FieldType__copy(
  const rosidl_runtime_c__type_description__FieldType * input,
  rosidl_runtime_c__type_description__FieldType * output)
{
  if (!input || !output) {
    return false;
  }
  // type_id
  output->type_id = input->type_id;
  // capacity
  output->capacity = input->capacity;
  // string_capacity
  output->string_capacity = input->string_capacity;
  // nested_type_name
  if (!rosidl_runtime_c__String__copy(
      &(input->nested_type_name), &(output->nested_type_name)))
  {
    return false;
  }
  return true;
}

rosidl_runtime_c__type_description__FieldType *
rosidl_runtime_c__type_description__FieldType__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosidl_runtime_c__type_description__FieldType * msg = (rosidl_runtime_c__type_description__FieldType *)allocator.allocate(sizeof(rosidl_runtime_c__type_description__FieldType), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rosidl_runtime_c__type_description__FieldType));
  bool success = rosidl_runtime_c__type_description__FieldType__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rosidl_runtime_c__type_description__FieldType__destroy(rosidl_runtime_c__type_description__FieldType * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rosidl_runtime_c__type_description__FieldType__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rosidl_runtime_c__type_description__FieldType__Sequence__init(rosidl_runtime_c__type_description__FieldType__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosidl_runtime_c__type_description__FieldType * data = NULL;

  if (size) {
    data = (rosidl_runtime_c__type_description__FieldType *)allocator.zero_allocate(size, sizeof(rosidl_runtime_c__type_description__FieldType), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rosidl_runtime_c__type_description__FieldType__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rosidl_runtime_c__type_description__FieldType__fini(&data[i - 1]);
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
rosidl_runtime_c__type_description__FieldType__Sequence__fini(rosidl_runtime_c__type_description__FieldType__Sequence * array)
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
      rosidl_runtime_c__type_description__FieldType__fini(&array->data[i]);
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

rosidl_runtime_c__type_description__FieldType__Sequence *
rosidl_runtime_c__type_description__FieldType__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosidl_runtime_c__type_description__FieldType__Sequence * array = (rosidl_runtime_c__type_description__FieldType__Sequence *)allocator.allocate(sizeof(rosidl_runtime_c__type_description__FieldType__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rosidl_runtime_c__type_description__FieldType__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rosidl_runtime_c__type_description__FieldType__Sequence__destroy(rosidl_runtime_c__type_description__FieldType__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rosidl_runtime_c__type_description__FieldType__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rosidl_runtime_c__type_description__FieldType__Sequence__are_equal(const rosidl_runtime_c__type_description__FieldType__Sequence * lhs, const rosidl_runtime_c__type_description__FieldType__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rosidl_runtime_c__type_description__FieldType__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rosidl_runtime_c__type_description__FieldType__Sequence__copy(
  const rosidl_runtime_c__type_description__FieldType__Sequence * input,
  rosidl_runtime_c__type_description__FieldType__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rosidl_runtime_c__type_description__FieldType);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rosidl_runtime_c__type_description__FieldType * data =
      (rosidl_runtime_c__type_description__FieldType *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rosidl_runtime_c__type_description__FieldType__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rosidl_runtime_c__type_description__FieldType__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rosidl_runtime_c__type_description__FieldType__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
