// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from shared_interfaces:msg/BatteryStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "shared_interfaces/msg/battery_status.hpp"


#ifndef SHARED_INTERFACES__MSG__DETAIL__BATTERY_STATUS__TRAITS_HPP_
#define SHARED_INTERFACES__MSG__DETAIL__BATTERY_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "shared_interfaces/msg/detail/battery_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace shared_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const BatteryStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: voltage
  {
    out << "voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.voltage, out);
    out << ", ";
  }

  // member: current
  {
    out << "current: ";
    rosidl_generator_traits::value_to_yaml(msg.current, out);
    out << ", ";
  }

  // member: percentage
  {
    out << "percentage: ";
    rosidl_generator_traits::value_to_yaml(msg.percentage, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const BatteryStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: voltage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.voltage, out);
    out << "\n";
  }

  // member: current
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current: ";
    rosidl_generator_traits::value_to_yaml(msg.current, out);
    out << "\n";
  }

  // member: percentage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "percentage: ";
    rosidl_generator_traits::value_to_yaml(msg.percentage, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const BatteryStatus & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace shared_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use shared_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const shared_interfaces::msg::BatteryStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  shared_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use shared_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const shared_interfaces::msg::BatteryStatus & msg)
{
  return shared_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<shared_interfaces::msg::BatteryStatus>()
{
  return "shared_interfaces::msg::BatteryStatus";
}

template<>
inline const char * name<shared_interfaces::msg::BatteryStatus>()
{
  return "shared_interfaces/msg/BatteryStatus";
}

template<>
struct has_fixed_size<shared_interfaces::msg::BatteryStatus>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<shared_interfaces::msg::BatteryStatus>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<shared_interfaces::msg::BatteryStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SHARED_INTERFACES__MSG__DETAIL__BATTERY_STATUS__TRAITS_HPP_
