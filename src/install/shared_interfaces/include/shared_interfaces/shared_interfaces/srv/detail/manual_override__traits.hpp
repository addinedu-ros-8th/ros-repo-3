// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from shared_interfaces:srv/ManualOverride.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "shared_interfaces/srv/manual_override.hpp"


#ifndef SHARED_INTERFACES__SRV__DETAIL__MANUAL_OVERRIDE__TRAITS_HPP_
#define SHARED_INTERFACES__SRV__DETAIL__MANUAL_OVERRIDE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "shared_interfaces/srv/detail/manual_override__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace shared_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const ManualOverride_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: command
  {
    out << "command: ";
    rosidl_generator_traits::value_to_yaml(msg.command, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ManualOverride_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: command
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "command: ";
    rosidl_generator_traits::value_to_yaml(msg.command, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ManualOverride_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace shared_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use shared_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const shared_interfaces::srv::ManualOverride_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  shared_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use shared_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const shared_interfaces::srv::ManualOverride_Request & msg)
{
  return shared_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<shared_interfaces::srv::ManualOverride_Request>()
{
  return "shared_interfaces::srv::ManualOverride_Request";
}

template<>
inline const char * name<shared_interfaces::srv::ManualOverride_Request>()
{
  return "shared_interfaces/srv/ManualOverride_Request";
}

template<>
struct has_fixed_size<shared_interfaces::srv::ManualOverride_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<shared_interfaces::srv::ManualOverride_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<shared_interfaces::srv::ManualOverride_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace shared_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const ManualOverride_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ManualOverride_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ManualOverride_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace shared_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use shared_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const shared_interfaces::srv::ManualOverride_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  shared_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use shared_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const shared_interfaces::srv::ManualOverride_Response & msg)
{
  return shared_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<shared_interfaces::srv::ManualOverride_Response>()
{
  return "shared_interfaces::srv::ManualOverride_Response";
}

template<>
inline const char * name<shared_interfaces::srv::ManualOverride_Response>()
{
  return "shared_interfaces/srv/ManualOverride_Response";
}

template<>
struct has_fixed_size<shared_interfaces::srv::ManualOverride_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<shared_interfaces::srv::ManualOverride_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<shared_interfaces::srv::ManualOverride_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__traits.hpp"

namespace shared_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const ManualOverride_Event & msg,
  std::ostream & out)
{
  out << "{";
  // member: info
  {
    out << "info: ";
    to_flow_style_yaml(msg.info, out);
    out << ", ";
  }

  // member: request
  {
    if (msg.request.size() == 0) {
      out << "request: []";
    } else {
      out << "request: [";
      size_t pending_items = msg.request.size();
      for (auto item : msg.request) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: response
  {
    if (msg.response.size() == 0) {
      out << "response: []";
    } else {
      out << "response: [";
      size_t pending_items = msg.response.size();
      for (auto item : msg.response) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ManualOverride_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "info:\n";
    to_block_style_yaml(msg.info, out, indentation + 2);
  }

  // member: request
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.request.size() == 0) {
      out << "request: []\n";
    } else {
      out << "request:\n";
      for (auto item : msg.request) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: response
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.response.size() == 0) {
      out << "response: []\n";
    } else {
      out << "response:\n";
      for (auto item : msg.response) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ManualOverride_Event & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace shared_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use shared_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const shared_interfaces::srv::ManualOverride_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  shared_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use shared_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const shared_interfaces::srv::ManualOverride_Event & msg)
{
  return shared_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<shared_interfaces::srv::ManualOverride_Event>()
{
  return "shared_interfaces::srv::ManualOverride_Event";
}

template<>
inline const char * name<shared_interfaces::srv::ManualOverride_Event>()
{
  return "shared_interfaces/srv/ManualOverride_Event";
}

template<>
struct has_fixed_size<shared_interfaces::srv::ManualOverride_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<shared_interfaces::srv::ManualOverride_Event>
  : std::integral_constant<bool, has_bounded_size<service_msgs::msg::ServiceEventInfo>::value && has_bounded_size<shared_interfaces::srv::ManualOverride_Request>::value && has_bounded_size<shared_interfaces::srv::ManualOverride_Response>::value> {};

template<>
struct is_message<shared_interfaces::srv::ManualOverride_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<shared_interfaces::srv::ManualOverride>()
{
  return "shared_interfaces::srv::ManualOverride";
}

template<>
inline const char * name<shared_interfaces::srv::ManualOverride>()
{
  return "shared_interfaces/srv/ManualOverride";
}

template<>
struct has_fixed_size<shared_interfaces::srv::ManualOverride>
  : std::integral_constant<
    bool,
    has_fixed_size<shared_interfaces::srv::ManualOverride_Request>::value &&
    has_fixed_size<shared_interfaces::srv::ManualOverride_Response>::value
  >
{
};

template<>
struct has_bounded_size<shared_interfaces::srv::ManualOverride>
  : std::integral_constant<
    bool,
    has_bounded_size<shared_interfaces::srv::ManualOverride_Request>::value &&
    has_bounded_size<shared_interfaces::srv::ManualOverride_Response>::value
  >
{
};

template<>
struct is_service<shared_interfaces::srv::ManualOverride>
  : std::true_type
{
};

template<>
struct is_service_request<shared_interfaces::srv::ManualOverride_Request>
  : std::true_type
{
};

template<>
struct is_service_response<shared_interfaces::srv::ManualOverride_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // SHARED_INTERFACES__SRV__DETAIL__MANUAL_OVERRIDE__TRAITS_HPP_
