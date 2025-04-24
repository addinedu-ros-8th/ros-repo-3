// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from shared_interfaces:srv/ManualOverride.idl
// generated code does not contain a copyright notice
#include "shared_interfaces/srv/detail/manual_override__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `command`
#include "rosidl_runtime_c/string_functions.h"

bool
shared_interfaces__srv__ManualOverride_Request__init(shared_interfaces__srv__ManualOverride_Request * msg)
{
  if (!msg) {
    return false;
  }
  // command
  if (!rosidl_runtime_c__String__init(&msg->command)) {
    shared_interfaces__srv__ManualOverride_Request__fini(msg);
    return false;
  }
  return true;
}

void
shared_interfaces__srv__ManualOverride_Request__fini(shared_interfaces__srv__ManualOverride_Request * msg)
{
  if (!msg) {
    return;
  }
  // command
  rosidl_runtime_c__String__fini(&msg->command);
}

bool
shared_interfaces__srv__ManualOverride_Request__are_equal(const shared_interfaces__srv__ManualOverride_Request * lhs, const shared_interfaces__srv__ManualOverride_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // command
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->command), &(rhs->command)))
  {
    return false;
  }
  return true;
}

bool
shared_interfaces__srv__ManualOverride_Request__copy(
  const shared_interfaces__srv__ManualOverride_Request * input,
  shared_interfaces__srv__ManualOverride_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // command
  if (!rosidl_runtime_c__String__copy(
      &(input->command), &(output->command)))
  {
    return false;
  }
  return true;
}

shared_interfaces__srv__ManualOverride_Request *
shared_interfaces__srv__ManualOverride_Request__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  shared_interfaces__srv__ManualOverride_Request * msg = (shared_interfaces__srv__ManualOverride_Request *)allocator.allocate(sizeof(shared_interfaces__srv__ManualOverride_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(shared_interfaces__srv__ManualOverride_Request));
  bool success = shared_interfaces__srv__ManualOverride_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
shared_interfaces__srv__ManualOverride_Request__destroy(shared_interfaces__srv__ManualOverride_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    shared_interfaces__srv__ManualOverride_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
shared_interfaces__srv__ManualOverride_Request__Sequence__init(shared_interfaces__srv__ManualOverride_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  shared_interfaces__srv__ManualOverride_Request * data = NULL;

  if (size) {
    data = (shared_interfaces__srv__ManualOverride_Request *)allocator.zero_allocate(size, sizeof(shared_interfaces__srv__ManualOverride_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = shared_interfaces__srv__ManualOverride_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        shared_interfaces__srv__ManualOverride_Request__fini(&data[i - 1]);
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
shared_interfaces__srv__ManualOverride_Request__Sequence__fini(shared_interfaces__srv__ManualOverride_Request__Sequence * array)
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
      shared_interfaces__srv__ManualOverride_Request__fini(&array->data[i]);
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

shared_interfaces__srv__ManualOverride_Request__Sequence *
shared_interfaces__srv__ManualOverride_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  shared_interfaces__srv__ManualOverride_Request__Sequence * array = (shared_interfaces__srv__ManualOverride_Request__Sequence *)allocator.allocate(sizeof(shared_interfaces__srv__ManualOverride_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = shared_interfaces__srv__ManualOverride_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
shared_interfaces__srv__ManualOverride_Request__Sequence__destroy(shared_interfaces__srv__ManualOverride_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    shared_interfaces__srv__ManualOverride_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
shared_interfaces__srv__ManualOverride_Request__Sequence__are_equal(const shared_interfaces__srv__ManualOverride_Request__Sequence * lhs, const shared_interfaces__srv__ManualOverride_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!shared_interfaces__srv__ManualOverride_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
shared_interfaces__srv__ManualOverride_Request__Sequence__copy(
  const shared_interfaces__srv__ManualOverride_Request__Sequence * input,
  shared_interfaces__srv__ManualOverride_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(shared_interfaces__srv__ManualOverride_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    shared_interfaces__srv__ManualOverride_Request * data =
      (shared_interfaces__srv__ManualOverride_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!shared_interfaces__srv__ManualOverride_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          shared_interfaces__srv__ManualOverride_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!shared_interfaces__srv__ManualOverride_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
shared_interfaces__srv__ManualOverride_Response__init(shared_interfaces__srv__ManualOverride_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  return true;
}

void
shared_interfaces__srv__ManualOverride_Response__fini(shared_interfaces__srv__ManualOverride_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
}

bool
shared_interfaces__srv__ManualOverride_Response__are_equal(const shared_interfaces__srv__ManualOverride_Response * lhs, const shared_interfaces__srv__ManualOverride_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  return true;
}

bool
shared_interfaces__srv__ManualOverride_Response__copy(
  const shared_interfaces__srv__ManualOverride_Response * input,
  shared_interfaces__srv__ManualOverride_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  return true;
}

shared_interfaces__srv__ManualOverride_Response *
shared_interfaces__srv__ManualOverride_Response__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  shared_interfaces__srv__ManualOverride_Response * msg = (shared_interfaces__srv__ManualOverride_Response *)allocator.allocate(sizeof(shared_interfaces__srv__ManualOverride_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(shared_interfaces__srv__ManualOverride_Response));
  bool success = shared_interfaces__srv__ManualOverride_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
shared_interfaces__srv__ManualOverride_Response__destroy(shared_interfaces__srv__ManualOverride_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    shared_interfaces__srv__ManualOverride_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
shared_interfaces__srv__ManualOverride_Response__Sequence__init(shared_interfaces__srv__ManualOverride_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  shared_interfaces__srv__ManualOverride_Response * data = NULL;

  if (size) {
    data = (shared_interfaces__srv__ManualOverride_Response *)allocator.zero_allocate(size, sizeof(shared_interfaces__srv__ManualOverride_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = shared_interfaces__srv__ManualOverride_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        shared_interfaces__srv__ManualOverride_Response__fini(&data[i - 1]);
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
shared_interfaces__srv__ManualOverride_Response__Sequence__fini(shared_interfaces__srv__ManualOverride_Response__Sequence * array)
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
      shared_interfaces__srv__ManualOverride_Response__fini(&array->data[i]);
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

shared_interfaces__srv__ManualOverride_Response__Sequence *
shared_interfaces__srv__ManualOverride_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  shared_interfaces__srv__ManualOverride_Response__Sequence * array = (shared_interfaces__srv__ManualOverride_Response__Sequence *)allocator.allocate(sizeof(shared_interfaces__srv__ManualOverride_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = shared_interfaces__srv__ManualOverride_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
shared_interfaces__srv__ManualOverride_Response__Sequence__destroy(shared_interfaces__srv__ManualOverride_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    shared_interfaces__srv__ManualOverride_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
shared_interfaces__srv__ManualOverride_Response__Sequence__are_equal(const shared_interfaces__srv__ManualOverride_Response__Sequence * lhs, const shared_interfaces__srv__ManualOverride_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!shared_interfaces__srv__ManualOverride_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
shared_interfaces__srv__ManualOverride_Response__Sequence__copy(
  const shared_interfaces__srv__ManualOverride_Response__Sequence * input,
  shared_interfaces__srv__ManualOverride_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(shared_interfaces__srv__ManualOverride_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    shared_interfaces__srv__ManualOverride_Response * data =
      (shared_interfaces__srv__ManualOverride_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!shared_interfaces__srv__ManualOverride_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          shared_interfaces__srv__ManualOverride_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!shared_interfaces__srv__ManualOverride_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `info`
#include "service_msgs/msg/detail/service_event_info__functions.h"
// Member `request`
// Member `response`
// already included above
// #include "shared_interfaces/srv/detail/manual_override__functions.h"

bool
shared_interfaces__srv__ManualOverride_Event__init(shared_interfaces__srv__ManualOverride_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    shared_interfaces__srv__ManualOverride_Event__fini(msg);
    return false;
  }
  // request
  if (!shared_interfaces__srv__ManualOverride_Request__Sequence__init(&msg->request, 0)) {
    shared_interfaces__srv__ManualOverride_Event__fini(msg);
    return false;
  }
  // response
  if (!shared_interfaces__srv__ManualOverride_Response__Sequence__init(&msg->response, 0)) {
    shared_interfaces__srv__ManualOverride_Event__fini(msg);
    return false;
  }
  return true;
}

void
shared_interfaces__srv__ManualOverride_Event__fini(shared_interfaces__srv__ManualOverride_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  shared_interfaces__srv__ManualOverride_Request__Sequence__fini(&msg->request);
  // response
  shared_interfaces__srv__ManualOverride_Response__Sequence__fini(&msg->response);
}

bool
shared_interfaces__srv__ManualOverride_Event__are_equal(const shared_interfaces__srv__ManualOverride_Event * lhs, const shared_interfaces__srv__ManualOverride_Event * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__are_equal(
      &(lhs->info), &(rhs->info)))
  {
    return false;
  }
  // request
  if (!shared_interfaces__srv__ManualOverride_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!shared_interfaces__srv__ManualOverride_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
shared_interfaces__srv__ManualOverride_Event__copy(
  const shared_interfaces__srv__ManualOverride_Event * input,
  shared_interfaces__srv__ManualOverride_Event * output)
{
  if (!input || !output) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__copy(
      &(input->info), &(output->info)))
  {
    return false;
  }
  // request
  if (!shared_interfaces__srv__ManualOverride_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!shared_interfaces__srv__ManualOverride_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

shared_interfaces__srv__ManualOverride_Event *
shared_interfaces__srv__ManualOverride_Event__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  shared_interfaces__srv__ManualOverride_Event * msg = (shared_interfaces__srv__ManualOverride_Event *)allocator.allocate(sizeof(shared_interfaces__srv__ManualOverride_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(shared_interfaces__srv__ManualOverride_Event));
  bool success = shared_interfaces__srv__ManualOverride_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
shared_interfaces__srv__ManualOverride_Event__destroy(shared_interfaces__srv__ManualOverride_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    shared_interfaces__srv__ManualOverride_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
shared_interfaces__srv__ManualOverride_Event__Sequence__init(shared_interfaces__srv__ManualOverride_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  shared_interfaces__srv__ManualOverride_Event * data = NULL;

  if (size) {
    data = (shared_interfaces__srv__ManualOverride_Event *)allocator.zero_allocate(size, sizeof(shared_interfaces__srv__ManualOverride_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = shared_interfaces__srv__ManualOverride_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        shared_interfaces__srv__ManualOverride_Event__fini(&data[i - 1]);
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
shared_interfaces__srv__ManualOverride_Event__Sequence__fini(shared_interfaces__srv__ManualOverride_Event__Sequence * array)
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
      shared_interfaces__srv__ManualOverride_Event__fini(&array->data[i]);
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

shared_interfaces__srv__ManualOverride_Event__Sequence *
shared_interfaces__srv__ManualOverride_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  shared_interfaces__srv__ManualOverride_Event__Sequence * array = (shared_interfaces__srv__ManualOverride_Event__Sequence *)allocator.allocate(sizeof(shared_interfaces__srv__ManualOverride_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = shared_interfaces__srv__ManualOverride_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
shared_interfaces__srv__ManualOverride_Event__Sequence__destroy(shared_interfaces__srv__ManualOverride_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    shared_interfaces__srv__ManualOverride_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
shared_interfaces__srv__ManualOverride_Event__Sequence__are_equal(const shared_interfaces__srv__ManualOverride_Event__Sequence * lhs, const shared_interfaces__srv__ManualOverride_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!shared_interfaces__srv__ManualOverride_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
shared_interfaces__srv__ManualOverride_Event__Sequence__copy(
  const shared_interfaces__srv__ManualOverride_Event__Sequence * input,
  shared_interfaces__srv__ManualOverride_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(shared_interfaces__srv__ManualOverride_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    shared_interfaces__srv__ManualOverride_Event * data =
      (shared_interfaces__srv__ManualOverride_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!shared_interfaces__srv__ManualOverride_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          shared_interfaces__srv__ManualOverride_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!shared_interfaces__srv__ManualOverride_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
