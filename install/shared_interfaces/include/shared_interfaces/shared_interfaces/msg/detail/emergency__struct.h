// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from shared_interfaces:msg/Emergency.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "shared_interfaces/msg/emergency.h"


#ifndef SHARED_INTERFACES__MSG__DETAIL__EMERGENCY__STRUCT_H_
#define SHARED_INTERFACES__MSG__DETAIL__EMERGENCY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'description'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/Emergency in the package shared_interfaces.
typedef struct shared_interfaces__msg__Emergency
{
  bool emergency;
  rosidl_runtime_c__String description;
} shared_interfaces__msg__Emergency;

// Struct for a sequence of shared_interfaces__msg__Emergency.
typedef struct shared_interfaces__msg__Emergency__Sequence
{
  shared_interfaces__msg__Emergency * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} shared_interfaces__msg__Emergency__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SHARED_INTERFACES__MSG__DETAIL__EMERGENCY__STRUCT_H_
