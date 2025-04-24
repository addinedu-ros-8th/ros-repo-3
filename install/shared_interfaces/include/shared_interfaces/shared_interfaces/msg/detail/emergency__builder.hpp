// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from shared_interfaces:msg/Emergency.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "shared_interfaces/msg/emergency.hpp"


#ifndef SHARED_INTERFACES__MSG__DETAIL__EMERGENCY__BUILDER_HPP_
#define SHARED_INTERFACES__MSG__DETAIL__EMERGENCY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "shared_interfaces/msg/detail/emergency__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace shared_interfaces
{

namespace msg
{

namespace builder
{

class Init_Emergency_description
{
public:
  explicit Init_Emergency_description(::shared_interfaces::msg::Emergency & msg)
  : msg_(msg)
  {}
  ::shared_interfaces::msg::Emergency description(::shared_interfaces::msg::Emergency::_description_type arg)
  {
    msg_.description = std::move(arg);
    return std::move(msg_);
  }

private:
  ::shared_interfaces::msg::Emergency msg_;
};

class Init_Emergency_emergency
{
public:
  Init_Emergency_emergency()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Emergency_description emergency(::shared_interfaces::msg::Emergency::_emergency_type arg)
  {
    msg_.emergency = std::move(arg);
    return Init_Emergency_description(msg_);
  }

private:
  ::shared_interfaces::msg::Emergency msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::shared_interfaces::msg::Emergency>()
{
  return shared_interfaces::msg::builder::Init_Emergency_emergency();
}

}  // namespace shared_interfaces

#endif  // SHARED_INTERFACES__MSG__DETAIL__EMERGENCY__BUILDER_HPP_
