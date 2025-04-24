// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from shared_interfaces:msg/BatteryStatus.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "shared_interfaces/msg/detail/battery_status__rosidl_typesupport_introspection_c.h"
#include "shared_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "shared_interfaces/msg/detail/battery_status__functions.h"
#include "shared_interfaces/msg/detail/battery_status__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void shared_interfaces__msg__BatteryStatus__rosidl_typesupport_introspection_c__BatteryStatus_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  shared_interfaces__msg__BatteryStatus__init(message_memory);
}

void shared_interfaces__msg__BatteryStatus__rosidl_typesupport_introspection_c__BatteryStatus_fini_function(void * message_memory)
{
  shared_interfaces__msg__BatteryStatus__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember shared_interfaces__msg__BatteryStatus__rosidl_typesupport_introspection_c__BatteryStatus_message_member_array[3] = {
  {
    "voltage",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(shared_interfaces__msg__BatteryStatus, voltage),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "current",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(shared_interfaces__msg__BatteryStatus, current),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "percentage",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(shared_interfaces__msg__BatteryStatus, percentage),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers shared_interfaces__msg__BatteryStatus__rosidl_typesupport_introspection_c__BatteryStatus_message_members = {
  "shared_interfaces__msg",  // message namespace
  "BatteryStatus",  // message name
  3,  // number of fields
  sizeof(shared_interfaces__msg__BatteryStatus),
  false,  // has_any_key_member_
  shared_interfaces__msg__BatteryStatus__rosidl_typesupport_introspection_c__BatteryStatus_message_member_array,  // message members
  shared_interfaces__msg__BatteryStatus__rosidl_typesupport_introspection_c__BatteryStatus_init_function,  // function to initialize message memory (memory has to be allocated)
  shared_interfaces__msg__BatteryStatus__rosidl_typesupport_introspection_c__BatteryStatus_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t shared_interfaces__msg__BatteryStatus__rosidl_typesupport_introspection_c__BatteryStatus_message_type_support_handle = {
  0,
  &shared_interfaces__msg__BatteryStatus__rosidl_typesupport_introspection_c__BatteryStatus_message_members,
  get_message_typesupport_handle_function,
  &shared_interfaces__msg__BatteryStatus__get_type_hash,
  &shared_interfaces__msg__BatteryStatus__get_type_description,
  &shared_interfaces__msg__BatteryStatus__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_shared_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, shared_interfaces, msg, BatteryStatus)() {
  if (!shared_interfaces__msg__BatteryStatus__rosidl_typesupport_introspection_c__BatteryStatus_message_type_support_handle.typesupport_identifier) {
    shared_interfaces__msg__BatteryStatus__rosidl_typesupport_introspection_c__BatteryStatus_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &shared_interfaces__msg__BatteryStatus__rosidl_typesupport_introspection_c__BatteryStatus_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
