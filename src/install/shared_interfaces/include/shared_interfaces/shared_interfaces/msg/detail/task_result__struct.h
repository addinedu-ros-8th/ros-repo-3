// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from shared_interfaces:msg/TaskResult.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "shared_interfaces/msg/task_result.h"


#ifndef SHARED_INTERFACES__MSG__DETAIL__TASK_RESULT__STRUCT_H_
#define SHARED_INTERFACES__MSG__DETAIL__TASK_RESULT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'task_id'
// Member 'result'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/TaskResult in the package shared_interfaces.
typedef struct shared_interfaces__msg__TaskResult
{
  rosidl_runtime_c__String task_id;
  rosidl_runtime_c__String result;
} shared_interfaces__msg__TaskResult;

// Struct for a sequence of shared_interfaces__msg__TaskResult.
typedef struct shared_interfaces__msg__TaskResult__Sequence
{
  shared_interfaces__msg__TaskResult * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} shared_interfaces__msg__TaskResult__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SHARED_INTERFACES__MSG__DETAIL__TASK_RESULT__STRUCT_H_
