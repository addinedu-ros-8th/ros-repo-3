// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from shared_interfaces:msg/Emergency.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "shared_interfaces/msg/emergency.hpp"


#ifndef SHARED_INTERFACES__MSG__DETAIL__EMERGENCY__TRAITS_HPP_
#define SHARED_INTERFACES__MSG__DETAIL__EMERGENCY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "shared_interfaces/msg/detail/emergency__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace shared_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Emergency & msg,
  std::ostream & out)
{
  out << "{";
  // member: emergency
  {
    out << "emergency: ";
    rosidl_generator_traits::value_to_yaml(msg.emergency, out);
    out << ", ";
  }

  // member: description
  {
    out << "description: ";
    rosidl_generator_traits::value_to_yaml(msg.description, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Emergency & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: emergency
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "emergency: ";
    rosidl_generator_traits::value_to_yaml(msg.emergency, out);
    out << "\n";
  }

  // member: description
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "description: ";
    rosidl_generator_traits::value_to_yaml(msg.description, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Emergency & msg, bool use_flow_style = false)
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
  const shared_interfaces::msg::Emergency & msg,
  std::ostream & out, size_t indentation = 0)
{
  shared_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use shared_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const shared_interfaces::msg::Emergency & msg)
{
  return shared_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<shared_interfaces::msg::Emergency>()
{
  return "shared_interfaces::msg::Emergency";
}

template<>
inline const char * name<shared_interfaces::msg::Emergency>()
{
  return "shared_interfaces/msg/Emergency";
}

template<>
struct has_fixed_size<shared_interfaces::msg::Emergency>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<shared_interfaces::msg::Emergency>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<shared_interfaces::msg::Emergency>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SHARED_INTERFACES__MSG__DETAIL__EMERGENCY__TRAITS_HPP_
