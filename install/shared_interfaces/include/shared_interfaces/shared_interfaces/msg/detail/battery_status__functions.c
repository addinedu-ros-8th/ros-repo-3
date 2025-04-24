// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from shared_interfaces:msg/BatteryStatus.idl
// generated code does not contain a copyright notice
#include "shared_interfaces/msg/detail/battery_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
shared_interfaces__msg__BatteryStatus__init(shared_interfaces__msg__BatteryStatus * msg)
{
  if (!msg) {
    return false;
  }
  // voltage
  // current
  // percentage
  return true;
}

void
shared_interfaces__msg__BatteryStatus__fini(shared_interfaces__msg__BatteryStatus * msg)
{
  if (!msg) {
    return;
  }
  // voltage
  // current
  // percentage
}

bool
shared_interfaces__msg__BatteryStatus__are_equal(const shared_interfaces__msg__BatteryStatus * lhs, const shared_interfaces__msg__BatteryStatus * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // voltage
  if (lhs->voltage != rhs->voltage) {
    return false;
  }
  // current
  if (lhs->current != rhs->current) {
    return false;
  }
  // percentage
  if (lhs->percentage != rhs->percentage) {
    return false;
  }
  return true;
}

bool
shared_interfaces__msg__BatteryStatus__copy(
  const shared_interfaces__msg__BatteryStatus * input,
  shared_interfaces__msg__BatteryStatus * output)
{
  if (!input || !output) {
    return false;
  }
  // voltage
  output->voltage = input->voltage;
  // current
  output->current = input->current;
  // percentage
  output->percentage = input->percentage;
  return true;
}

shared_interfaces__msg__BatteryStatus *
shared_interfaces__msg__BatteryStatus__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  shared_interfaces__msg__BatteryStatus * msg = (shared_interfaces__msg__BatteryStatus *)allocator.allocate(sizeof(shared_interfaces__msg__BatteryStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(shared_interfaces__msg__BatteryStatus));
  bool success = shared_interfaces__msg__BatteryStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
shared_interfaces__msg__BatteryStatus__destroy(shared_interfaces__msg__BatteryStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    shared_interfaces__msg__BatteryStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
shared_interfaces__msg__BatteryStatus__Sequence__init(shared_interfaces__msg__BatteryStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  shared_interfaces__msg__BatteryStatus * data = NULL;

  if (size) {
    data = (shared_interfaces__msg__BatteryStatus *)allocator.zero_allocate(size, sizeof(shared_interfaces__msg__BatteryStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = shared_interfaces__msg__BatteryStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        shared_interfaces__msg__BatteryStatus__fini(&data[i - 1]);
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
shared_interfaces__msg__BatteryStatus__Sequence__fini(shared_interfaces__msg__BatteryStatus__Sequence * array)
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
      shared_interfaces__msg__BatteryStatus__fini(&array->data[i]);
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

shared_interfaces__msg__BatteryStatus__Sequence *
shared_interfaces__msg__BatteryStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  shared_interfaces__msg__BatteryStatus__Sequence * array = (shared_interfaces__msg__BatteryStatus__Sequence *)allocator.allocate(sizeof(shared_interfaces__msg__BatteryStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = shared_interfaces__msg__BatteryStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
shared_interfaces__msg__BatteryStatus__Sequence__destroy(shared_interfaces__msg__BatteryStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    shared_interfaces__msg__BatteryStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
shared_interfaces__msg__BatteryStatus__Sequence__are_equal(const shared_interfaces__msg__BatteryStatus__Sequence * lhs, const shared_interfaces__msg__BatteryStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!shared_interfaces__msg__BatteryStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
shared_interfaces__msg__BatteryStatus__Sequence__copy(
  const shared_interfaces__msg__BatteryStatus__Sequence * input,
  shared_interfaces__msg__BatteryStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(shared_interfaces__msg__BatteryStatus);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    shared_interfaces__msg__BatteryStatus * data =
      (shared_interfaces__msg__BatteryStatus *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!shared_interfaces__msg__BatteryStatus__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          shared_interfaces__msg__BatteryStatus__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!shared_interfaces__msg__BatteryStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
