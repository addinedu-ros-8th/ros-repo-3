// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from shared_interfaces:msg/BatteryStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "shared_interfaces/msg/battery_status.h"


#ifndef SHARED_INTERFACES__MSG__DETAIL__BATTERY_STATUS__STRUCT_H_
#define SHARED_INTERFACES__MSG__DETAIL__BATTERY_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/BatteryStatus in the package shared_interfaces.
typedef struct shared_interfaces__msg__BatteryStatus
{
  float voltage;
  float current;
  float percentage;
} shared_interfaces__msg__BatteryStatus;

// Struct for a sequence of shared_interfaces__msg__BatteryStatus.
typedef struct shared_interfaces__msg__BatteryStatus__Sequence
{
  shared_interfaces__msg__BatteryStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} shared_interfaces__msg__BatteryStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SHARED_INTERFACES__MSG__DETAIL__BATTERY_STATUS__STRUCT_H_
