// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from shared_interfaces:msg/Emergency.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "shared_interfaces/msg/emergency.hpp"


#ifndef SHARED_INTERFACES__MSG__DETAIL__EMERGENCY__STRUCT_HPP_
#define SHARED_INTERFACES__MSG__DETAIL__EMERGENCY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__shared_interfaces__msg__Emergency __attribute__((deprecated))
#else
# define DEPRECATED__shared_interfaces__msg__Emergency __declspec(deprecated)
#endif

namespace shared_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Emergency_
{
  using Type = Emergency_<ContainerAllocator>;

  explicit Emergency_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->emergency = false;
      this->description = "";
    }
  }

  explicit Emergency_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : description(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->emergency = false;
      this->description = "";
    }
  }

  // field types and members
  using _emergency_type =
    bool;
  _emergency_type emergency;
  using _description_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _description_type description;

  // setters for named parameter idiom
  Type & set__emergency(
    const bool & _arg)
  {
    this->emergency = _arg;
    return *this;
  }
  Type & set__description(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->description = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    shared_interfaces::msg::Emergency_<ContainerAllocator> *;
  using ConstRawPtr =
    const shared_interfaces::msg::Emergency_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<shared_interfaces::msg::Emergency_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<shared_interfaces::msg::Emergency_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      shared_interfaces::msg::Emergency_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<shared_interfaces::msg::Emergency_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      shared_interfaces::msg::Emergency_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<shared_interfaces::msg::Emergency_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<shared_interfaces::msg::Emergency_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<shared_interfaces::msg::Emergency_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__shared_interfaces__msg__Emergency
    std::shared_ptr<shared_interfaces::msg::Emergency_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__shared_interfaces__msg__Emergency
    std::shared_ptr<shared_interfaces::msg::Emergency_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Emergency_ & other) const
  {
    if (this->emergency != other.emergency) {
      return false;
    }
    if (this->description != other.description) {
      return false;
    }
    return true;
  }
  bool operator!=(const Emergency_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Emergency_

// alias to use template instance with default allocator
using Emergency =
  shared_interfaces::msg::Emergency_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace shared_interfaces

#endif  // SHARED_INTERFACES__MSG__DETAIL__EMERGENCY__STRUCT_HPP_
