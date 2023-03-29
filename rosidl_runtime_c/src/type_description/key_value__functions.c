// DO NOT EDIT MANUALLY - this copied file managed by copy_type_description_generated_sources.bash
// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from type_description_interfaces:msg/KeyValue.idl
// generated code does not contain a copyright notice
#include "rosidl_runtime_c/type_description/key_value__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `key`
// Member `value`
#include "rosidl_runtime_c/string_functions.h"

bool
rosidl_runtime_c__type_description__KeyValue__init(rosidl_runtime_c__type_description__KeyValue * msg)
{
  if (!msg) {
    return false;
  }
  // key
  if (!rosidl_runtime_c__String__init(&msg->key)) {
    rosidl_runtime_c__type_description__KeyValue__fini(msg);
    return false;
  }
  // value
  if (!rosidl_runtime_c__String__init(&msg->value)) {
    rosidl_runtime_c__type_description__KeyValue__fini(msg);
    return false;
  }
  return true;
}

void
rosidl_runtime_c__type_description__KeyValue__fini(rosidl_runtime_c__type_description__KeyValue * msg)
{
  if (!msg) {
    return;
  }
  // key
  rosidl_runtime_c__String__fini(&msg->key);
  // value
  rosidl_runtime_c__String__fini(&msg->value);
}

bool
rosidl_runtime_c__type_description__KeyValue__are_equal(const rosidl_runtime_c__type_description__KeyValue * lhs, const rosidl_runtime_c__type_description__KeyValue * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // key
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->key), &(rhs->key)))
  {
    return false;
  }
  // value
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->value), &(rhs->value)))
  {
    return false;
  }
  return true;
}

bool
rosidl_runtime_c__type_description__KeyValue__copy(
  const rosidl_runtime_c__type_description__KeyValue * input,
  rosidl_runtime_c__type_description__KeyValue * output)
{
  if (!input || !output) {
    return false;
  }
  // key
  if (!rosidl_runtime_c__String__copy(
      &(input->key), &(output->key)))
  {
    return false;
  }
  // value
  if (!rosidl_runtime_c__String__copy(
      &(input->value), &(output->value)))
  {
    return false;
  }
  return true;
}

rosidl_runtime_c__type_description__KeyValue *
rosidl_runtime_c__type_description__KeyValue__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosidl_runtime_c__type_description__KeyValue * msg = (rosidl_runtime_c__type_description__KeyValue *)allocator.allocate(sizeof(rosidl_runtime_c__type_description__KeyValue), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rosidl_runtime_c__type_description__KeyValue));
  bool success = rosidl_runtime_c__type_description__KeyValue__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rosidl_runtime_c__type_description__KeyValue__destroy(rosidl_runtime_c__type_description__KeyValue * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rosidl_runtime_c__type_description__KeyValue__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rosidl_runtime_c__type_description__KeyValue__Sequence__init(rosidl_runtime_c__type_description__KeyValue__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosidl_runtime_c__type_description__KeyValue * data = NULL;

  if (size) {
    data = (rosidl_runtime_c__type_description__KeyValue *)allocator.zero_allocate(size, sizeof(rosidl_runtime_c__type_description__KeyValue), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rosidl_runtime_c__type_description__KeyValue__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rosidl_runtime_c__type_description__KeyValue__fini(&data[i - 1]);
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
rosidl_runtime_c__type_description__KeyValue__Sequence__fini(rosidl_runtime_c__type_description__KeyValue__Sequence * array)
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
      rosidl_runtime_c__type_description__KeyValue__fini(&array->data[i]);
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

rosidl_runtime_c__type_description__KeyValue__Sequence *
rosidl_runtime_c__type_description__KeyValue__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosidl_runtime_c__type_description__KeyValue__Sequence * array = (rosidl_runtime_c__type_description__KeyValue__Sequence *)allocator.allocate(sizeof(rosidl_runtime_c__type_description__KeyValue__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rosidl_runtime_c__type_description__KeyValue__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rosidl_runtime_c__type_description__KeyValue__Sequence__destroy(rosidl_runtime_c__type_description__KeyValue__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rosidl_runtime_c__type_description__KeyValue__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rosidl_runtime_c__type_description__KeyValue__Sequence__are_equal(const rosidl_runtime_c__type_description__KeyValue__Sequence * lhs, const rosidl_runtime_c__type_description__KeyValue__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rosidl_runtime_c__type_description__KeyValue__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rosidl_runtime_c__type_description__KeyValue__Sequence__copy(
  const rosidl_runtime_c__type_description__KeyValue__Sequence * input,
  rosidl_runtime_c__type_description__KeyValue__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rosidl_runtime_c__type_description__KeyValue);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rosidl_runtime_c__type_description__KeyValue * data =
      (rosidl_runtime_c__type_description__KeyValue *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rosidl_runtime_c__type_description__KeyValue__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rosidl_runtime_c__type_description__KeyValue__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rosidl_runtime_c__type_description__KeyValue__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
