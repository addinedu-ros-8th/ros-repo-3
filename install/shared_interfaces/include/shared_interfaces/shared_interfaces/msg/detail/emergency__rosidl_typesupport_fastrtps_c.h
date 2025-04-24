// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from shared_interfaces:msg/Emergency.idl
// generated code does not contain a copyright notice
#ifndef SHARED_INTERFACES__MSG__DETAIL__EMERGENCY__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define SHARED_INTERFACES__MSG__DETAIL__EMERGENCY__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "shared_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "shared_interfaces/msg/detail/emergency__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_shared_interfaces
bool cdr_serialize_shared_interfaces__msg__Emergency(
  const shared_interfaces__msg__Emergency * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_shared_interfaces
bool cdr_deserialize_shared_interfaces__msg__Emergency(
  eprosima::fastcdr::Cdr &,
  shared_interfaces__msg__Emergency * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_shared_interfaces
size_t get_serialized_size_shared_interfaces__msg__Emergency(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_shared_interfaces
size_t max_serialized_size_shared_interfaces__msg__Emergency(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_shared_interfaces
bool cdr_serialize_key_shared_interfaces__msg__Emergency(
  const shared_interfaces__msg__Emergency * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_shared_interfaces
size_t get_serialized_size_key_shared_interfaces__msg__Emergency(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_shared_interfaces
size_t max_serialized_size_key_shared_interfaces__msg__Emergency(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_shared_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, shared_interfaces, msg, Emergency)();

#ifdef __cplusplus
}
#endif

#endif  // SHARED_INTERFACES__MSG__DETAIL__EMERGENCY__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
