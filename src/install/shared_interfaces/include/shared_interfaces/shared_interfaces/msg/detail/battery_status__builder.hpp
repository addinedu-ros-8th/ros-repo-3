// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from shared_interfaces:msg/BatteryStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "shared_interfaces/msg/battery_status.hpp"


#ifndef SHARED_INTERFACES__MSG__DETAIL__BATTERY_STATUS__BUILDER_HPP_
#define SHARED_INTERFACES__MSG__DETAIL__BATTERY_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "shared_interfaces/msg/detail/battery_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace shared_interfaces
{

namespace msg
{

namespace builder
{

class Init_BatteryStatus_percentage
{
public:
  explicit Init_BatteryStatus_percentage(::shared_interfaces::msg::BatteryStatus & msg)
  : msg_(msg)
  {}
  ::shared_interfaces::msg::BatteryStatus percentage(::shared_interfaces::msg::BatteryStatus::_percentage_type arg)
  {
    msg_.percentage = std::move(arg);
    return std::move(msg_);
  }

private:
  ::shared_interfaces::msg::BatteryStatus msg_;
};

class Init_BatteryStatus_current
{
public:
  explicit Init_BatteryStatus_current(::shared_interfaces::msg::BatteryStatus & msg)
  : msg_(msg)
  {}
  Init_BatteryStatus_percentage current(::shared_interfaces::msg::BatteryStatus::_current_type arg)
  {
    msg_.current = std::move(arg);
    return Init_BatteryStatus_percentage(msg_);
  }

private:
  ::shared_interfaces::msg::BatteryStatus msg_;
};

class Init_BatteryStatus_voltage
{
public:
  Init_BatteryStatus_voltage()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BatteryStatus_current voltage(::shared_interfaces::msg::BatteryStatus::_voltage_type arg)
  {
    msg_.voltage = std::move(arg);
    return Init_BatteryStatus_current(msg_);
  }

private:
  ::shared_interfaces::msg::BatteryStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::shared_interfaces::msg::BatteryStatus>()
{
  return shared_interfaces::msg::builder::Init_BatteryStatus_voltage();
}

}  // namespace shared_interfaces

#endif  // SHARED_INTERFACES__MSG__DETAIL__BATTERY_STATUS__BUILDER_HPP_
