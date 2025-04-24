// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from shared_interfaces:msg/Emergency.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "shared_interfaces/msg/detail/emergency__functions.h"
#include "shared_interfaces/msg/detail/emergency__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace shared_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void Emergency_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) shared_interfaces::msg::Emergency(_init);
}

void Emergency_fini_function(void * message_memory)
{
  auto typed_message = static_cast<shared_interfaces::msg::Emergency *>(message_memory);
  typed_message->~Emergency();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember Emergency_message_member_array[2] = {
  {
    "emergency",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(shared_interfaces::msg::Emergency, emergency),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "description",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(shared_interfaces::msg::Emergency, description),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers Emergency_message_members = {
  "shared_interfaces::msg",  // message namespace
  "Emergency",  // message name
  2,  // number of fields
  sizeof(shared_interfaces::msg::Emergency),
  false,  // has_any_key_member_
  Emergency_message_member_array,  // message members
  Emergency_init_function,  // function to initialize message memory (memory has to be allocated)
  Emergency_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t Emergency_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &Emergency_message_members,
  get_message_typesupport_handle_function,
  &shared_interfaces__msg__Emergency__get_type_hash,
  &shared_interfaces__msg__Emergency__get_type_description,
  &shared_interfaces__msg__Emergency__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace shared_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<shared_interfaces::msg::Emergency>()
{
  return &::shared_interfaces::msg::rosidl_typesupport_introspection_cpp::Emergency_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, shared_interfaces, msg, Emergency)() {
  return &::shared_interfaces::msg::rosidl_typesupport_introspection_cpp::Emergency_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
