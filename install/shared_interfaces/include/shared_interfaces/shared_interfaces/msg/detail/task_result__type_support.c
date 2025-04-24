// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from shared_interfaces:msg/TaskResult.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "shared_interfaces/msg/detail/task_result__rosidl_typesupport_introspection_c.h"
#include "shared_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "shared_interfaces/msg/detail/task_result__functions.h"
#include "shared_interfaces/msg/detail/task_result__struct.h"


// Include directives for member types
// Member `task_id`
// Member `result`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void shared_interfaces__msg__TaskResult__rosidl_typesupport_introspection_c__TaskResult_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  shared_interfaces__msg__TaskResult__init(message_memory);
}

void shared_interfaces__msg__TaskResult__rosidl_typesupport_introspection_c__TaskResult_fini_function(void * message_memory)
{
  shared_interfaces__msg__TaskResult__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember shared_interfaces__msg__TaskResult__rosidl_typesupport_introspection_c__TaskResult_message_member_array[2] = {
  {
    "task_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(shared_interfaces__msg__TaskResult, task_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "result",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(shared_interfaces__msg__TaskResult, result),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers shared_interfaces__msg__TaskResult__rosidl_typesupport_introspection_c__TaskResult_message_members = {
  "shared_interfaces__msg",  // message namespace
  "TaskResult",  // message name
  2,  // number of fields
  sizeof(shared_interfaces__msg__TaskResult),
  false,  // has_any_key_member_
  shared_interfaces__msg__TaskResult__rosidl_typesupport_introspection_c__TaskResult_message_member_array,  // message members
  shared_interfaces__msg__TaskResult__rosidl_typesupport_introspection_c__TaskResult_init_function,  // function to initialize message memory (memory has to be allocated)
  shared_interfaces__msg__TaskResult__rosidl_typesupport_introspection_c__TaskResult_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t shared_interfaces__msg__TaskResult__rosidl_typesupport_introspection_c__TaskResult_message_type_support_handle = {
  0,
  &shared_interfaces__msg__TaskResult__rosidl_typesupport_introspection_c__TaskResult_message_members,
  get_message_typesupport_handle_function,
  &shared_interfaces__msg__TaskResult__get_type_hash,
  &shared_interfaces__msg__TaskResult__get_type_description,
  &shared_interfaces__msg__TaskResult__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_shared_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, shared_interfaces, msg, TaskResult)() {
  if (!shared_interfaces__msg__TaskResult__rosidl_typesupport_introspection_c__TaskResult_message_type_support_handle.typesupport_identifier) {
    shared_interfaces__msg__TaskResult__rosidl_typesupport_introspection_c__TaskResult_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &shared_interfaces__msg__TaskResult__rosidl_typesupport_introspection_c__TaskResult_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
