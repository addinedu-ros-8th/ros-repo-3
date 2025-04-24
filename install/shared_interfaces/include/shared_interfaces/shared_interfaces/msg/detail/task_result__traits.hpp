// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from shared_interfaces:msg/TaskResult.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "shared_interfaces/msg/task_result.hpp"


#ifndef SHARED_INTERFACES__MSG__DETAIL__TASK_RESULT__TRAITS_HPP_
#define SHARED_INTERFACES__MSG__DETAIL__TASK_RESULT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "shared_interfaces/msg/detail/task_result__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace shared_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const TaskResult & msg,
  std::ostream & out)
{
  out << "{";
  // member: task_id
  {
    out << "task_id: ";
    rosidl_generator_traits::value_to_yaml(msg.task_id, out);
    out << ", ";
  }

  // member: result
  {
    out << "result: ";
    rosidl_generator_traits::value_to_yaml(msg.result, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TaskResult & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: task_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "task_id: ";
    rosidl_generator_traits::value_to_yaml(msg.task_id, out);
    out << "\n";
  }

  // member: result
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "result: ";
    rosidl_generator_traits::value_to_yaml(msg.result, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TaskResult & msg, bool use_flow_style = false)
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
  const shared_interfaces::msg::TaskResult & msg,
  std::ostream & out, size_t indentation = 0)
{
  shared_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use shared_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const shared_interfaces::msg::TaskResult & msg)
{
  return shared_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<shared_interfaces::msg::TaskResult>()
{
  return "shared_interfaces::msg::TaskResult";
}

template<>
inline const char * name<shared_interfaces::msg::TaskResult>()
{
  return "shared_interfaces/msg/TaskResult";
}

template<>
struct has_fixed_size<shared_interfaces::msg::TaskResult>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<shared_interfaces::msg::TaskResult>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<shared_interfaces::msg::TaskResult>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SHARED_INTERFACES__MSG__DETAIL__TASK_RESULT__TRAITS_HPP_
