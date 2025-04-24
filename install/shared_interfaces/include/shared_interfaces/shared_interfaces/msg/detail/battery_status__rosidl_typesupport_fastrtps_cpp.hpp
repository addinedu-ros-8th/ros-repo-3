// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from shared_interfaces:msg/BatteryStatus.idl
// generated code does not contain a copyright notice

#ifndef SHARED_INTERFACES__MSG__DETAIL__BATTERY_STATUS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define SHARED_INTERFACES__MSG__DETAIL__BATTERY_STATUS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include <cstddef>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "shared_interfaces/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "shared_interfaces/msg/detail/battery_status__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace shared_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_shared_interfaces
cdr_serialize(
  const shared_interfaces::msg::BatteryStatus & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_shared_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  shared_interfaces::msg::BatteryStatus & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_shared_interfaces
get_serialized_size(
  const shared_interfaces::msg::BatteryStatus & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_shared_interfaces
max_serialized_size_BatteryStatus(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_shared_interfaces
cdr_serialize_key(
  const shared_interfaces::msg::BatteryStatus & ros_message,
  eprosima::fastcdr::Cdr &);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_shared_interfaces
get_serialized_size_key(
  const shared_interfaces::msg::BatteryStatus & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_shared_interfaces
max_serialized_size_key_BatteryStatus(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace shared_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_shared_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, shared_interfaces, msg, BatteryStatus)();

#ifdef __cplusplus
}
#endif

#endif  // SHARED_INTERFACES__MSG__DETAIL__BATTERY_STATUS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
