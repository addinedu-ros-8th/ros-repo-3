// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from shared_interfaces:srv/ManualOverride.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "shared_interfaces/srv/manual_override.hpp"


#ifndef SHARED_INTERFACES__SRV__DETAIL__MANUAL_OVERRIDE__STRUCT_HPP_
#define SHARED_INTERFACES__SRV__DETAIL__MANUAL_OVERRIDE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__shared_interfaces__srv__ManualOverride_Request __attribute__((deprecated))
#else
# define DEPRECATED__shared_interfaces__srv__ManualOverride_Request __declspec(deprecated)
#endif

namespace shared_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ManualOverride_Request_
{
  using Type = ManualOverride_Request_<ContainerAllocator>;

  explicit ManualOverride_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_id = 0l;
      this->command = "";
    }
  }

  explicit ManualOverride_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : command(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_id = 0l;
      this->command = "";
    }
  }

  // field types and members
  using _robot_id_type =
    int32_t;
  _robot_id_type robot_id;
  using _command_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _command_type command;

  // setters for named parameter idiom
  Type & set__robot_id(
    const int32_t & _arg)
  {
    this->robot_id = _arg;
    return *this;
  }
  Type & set__command(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->command = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    shared_interfaces::srv::ManualOverride_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const shared_interfaces::srv::ManualOverride_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<shared_interfaces::srv::ManualOverride_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<shared_interfaces::srv::ManualOverride_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      shared_interfaces::srv::ManualOverride_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<shared_interfaces::srv::ManualOverride_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      shared_interfaces::srv::ManualOverride_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<shared_interfaces::srv::ManualOverride_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<shared_interfaces::srv::ManualOverride_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<shared_interfaces::srv::ManualOverride_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__shared_interfaces__srv__ManualOverride_Request
    std::shared_ptr<shared_interfaces::srv::ManualOverride_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__shared_interfaces__srv__ManualOverride_Request
    std::shared_ptr<shared_interfaces::srv::ManualOverride_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ManualOverride_Request_ & other) const
  {
    if (this->robot_id != other.robot_id) {
      return false;
    }
    if (this->command != other.command) {
      return false;
    }
    return true;
  }
  bool operator!=(const ManualOverride_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ManualOverride_Request_

// alias to use template instance with default allocator
using ManualOverride_Request =
  shared_interfaces::srv::ManualOverride_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace shared_interfaces


#ifndef _WIN32
# define DEPRECATED__shared_interfaces__srv__ManualOverride_Response __attribute__((deprecated))
#else
# define DEPRECATED__shared_interfaces__srv__ManualOverride_Response __declspec(deprecated)
#endif

namespace shared_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ManualOverride_Response_
{
  using Type = ManualOverride_Response_<ContainerAllocator>;

  explicit ManualOverride_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit ManualOverride_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    shared_interfaces::srv::ManualOverride_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const shared_interfaces::srv::ManualOverride_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<shared_interfaces::srv::ManualOverride_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<shared_interfaces::srv::ManualOverride_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      shared_interfaces::srv::ManualOverride_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<shared_interfaces::srv::ManualOverride_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      shared_interfaces::srv::ManualOverride_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<shared_interfaces::srv::ManualOverride_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<shared_interfaces::srv::ManualOverride_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<shared_interfaces::srv::ManualOverride_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__shared_interfaces__srv__ManualOverride_Response
    std::shared_ptr<shared_interfaces::srv::ManualOverride_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__shared_interfaces__srv__ManualOverride_Response
    std::shared_ptr<shared_interfaces::srv::ManualOverride_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ManualOverride_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const ManualOverride_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ManualOverride_Response_

// alias to use template instance with default allocator
using ManualOverride_Response =
  shared_interfaces::srv::ManualOverride_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace shared_interfaces


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__shared_interfaces__srv__ManualOverride_Event __attribute__((deprecated))
#else
# define DEPRECATED__shared_interfaces__srv__ManualOverride_Event __declspec(deprecated)
#endif

namespace shared_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ManualOverride_Event_
{
  using Type = ManualOverride_Event_<ContainerAllocator>;

  explicit ManualOverride_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit ManualOverride_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<shared_interfaces::srv::ManualOverride_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<shared_interfaces::srv::ManualOverride_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<shared_interfaces::srv::ManualOverride_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<shared_interfaces::srv::ManualOverride_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<shared_interfaces::srv::ManualOverride_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<shared_interfaces::srv::ManualOverride_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<shared_interfaces::srv::ManualOverride_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<shared_interfaces::srv::ManualOverride_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    shared_interfaces::srv::ManualOverride_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const shared_interfaces::srv::ManualOverride_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<shared_interfaces::srv::ManualOverride_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<shared_interfaces::srv::ManualOverride_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      shared_interfaces::srv::ManualOverride_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<shared_interfaces::srv::ManualOverride_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      shared_interfaces::srv::ManualOverride_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<shared_interfaces::srv::ManualOverride_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<shared_interfaces::srv::ManualOverride_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<shared_interfaces::srv::ManualOverride_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__shared_interfaces__srv__ManualOverride_Event
    std::shared_ptr<shared_interfaces::srv::ManualOverride_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__shared_interfaces__srv__ManualOverride_Event
    std::shared_ptr<shared_interfaces::srv::ManualOverride_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ManualOverride_Event_ & other) const
  {
    if (this->info != other.info) {
      return false;
    }
    if (this->request != other.request) {
      return false;
    }
    if (this->response != other.response) {
      return false;
    }
    return true;
  }
  bool operator!=(const ManualOverride_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ManualOverride_Event_

// alias to use template instance with default allocator
using ManualOverride_Event =
  shared_interfaces::srv::ManualOverride_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace shared_interfaces

namespace shared_interfaces
{

namespace srv
{

struct ManualOverride
{
  using Request = shared_interfaces::srv::ManualOverride_Request;
  using Response = shared_interfaces::srv::ManualOverride_Response;
  using Event = shared_interfaces::srv::ManualOverride_Event;
};

}  // namespace srv

}  // namespace shared_interfaces

#endif  // SHARED_INTERFACES__SRV__DETAIL__MANUAL_OVERRIDE__STRUCT_HPP_
