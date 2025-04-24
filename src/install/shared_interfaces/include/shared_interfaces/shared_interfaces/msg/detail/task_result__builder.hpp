// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from shared_interfaces:msg/TaskResult.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "shared_interfaces/msg/task_result.hpp"


#ifndef SHARED_INTERFACES__MSG__DETAIL__TASK_RESULT__BUILDER_HPP_
#define SHARED_INTERFACES__MSG__DETAIL__TASK_RESULT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "shared_interfaces/msg/detail/task_result__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace shared_interfaces
{

namespace msg
{

namespace builder
{

class Init_TaskResult_result
{
public:
  explicit Init_TaskResult_result(::shared_interfaces::msg::TaskResult & msg)
  : msg_(msg)
  {}
  ::shared_interfaces::msg::TaskResult result(::shared_interfaces::msg::TaskResult::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::shared_interfaces::msg::TaskResult msg_;
};

class Init_TaskResult_task_id
{
public:
  Init_TaskResult_task_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TaskResult_result task_id(::shared_interfaces::msg::TaskResult::_task_id_type arg)
  {
    msg_.task_id = std::move(arg);
    return Init_TaskResult_result(msg_);
  }

private:
  ::shared_interfaces::msg::TaskResult msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::shared_interfaces::msg::TaskResult>()
{
  return shared_interfaces::msg::builder::Init_TaskResult_task_id();
}

}  // namespace shared_interfaces

#endif  // SHARED_INTERFACES__MSG__DETAIL__TASK_RESULT__BUILDER_HPP_
