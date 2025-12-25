// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from moveit_msgs:srv/GetMultiStateValidity.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "moveit_msgs/srv/detail/get_multi_state_validity__functions.h"
#include "moveit_msgs/srv/detail/get_multi_state_validity__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace moveit_msgs
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void GetMultiStateValidity_Request_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) moveit_msgs::srv::GetMultiStateValidity_Request(_init);
}

void GetMultiStateValidity_Request_fini_function(void * message_memory)
{
  auto typed_message = static_cast<moveit_msgs::srv::GetMultiStateValidity_Request *>(message_memory);
  typed_message->~GetMultiStateValidity_Request();
}

size_t size_function__GetMultiStateValidity_Request__joint_states(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<sensor_msgs::msg::JointState> *>(untyped_member);
  return member->size();
}

const void * get_const_function__GetMultiStateValidity_Request__joint_states(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<sensor_msgs::msg::JointState> *>(untyped_member);
  return &member[index];
}

void * get_function__GetMultiStateValidity_Request__joint_states(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<sensor_msgs::msg::JointState> *>(untyped_member);
  return &member[index];
}

void fetch_function__GetMultiStateValidity_Request__joint_states(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const sensor_msgs::msg::JointState *>(
    get_const_function__GetMultiStateValidity_Request__joint_states(untyped_member, index));
  auto & value = *reinterpret_cast<sensor_msgs::msg::JointState *>(untyped_value);
  value = item;
}

void assign_function__GetMultiStateValidity_Request__joint_states(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<sensor_msgs::msg::JointState *>(
    get_function__GetMultiStateValidity_Request__joint_states(untyped_member, index));
  const auto & value = *reinterpret_cast<const sensor_msgs::msg::JointState *>(untyped_value);
  item = value;
}

void resize_function__GetMultiStateValidity_Request__joint_states(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<sensor_msgs::msg::JointState> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember GetMultiStateValidity_Request_message_member_array[4] = {
  {
    "robot_state",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<moveit_msgs::msg::RobotState>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(moveit_msgs::srv::GetMultiStateValidity_Request, robot_state),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "joint_states",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<sensor_msgs::msg::JointState>(),  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(moveit_msgs::srv::GetMultiStateValidity_Request, joint_states),  // bytes offset in struct
    nullptr,  // default value
    size_function__GetMultiStateValidity_Request__joint_states,  // size() function pointer
    get_const_function__GetMultiStateValidity_Request__joint_states,  // get_const(index) function pointer
    get_function__GetMultiStateValidity_Request__joint_states,  // get(index) function pointer
    fetch_function__GetMultiStateValidity_Request__joint_states,  // fetch(index, &value) function pointer
    assign_function__GetMultiStateValidity_Request__joint_states,  // assign(index, value) function pointer
    resize_function__GetMultiStateValidity_Request__joint_states  // resize(index) function pointer
  },
  {
    "group_name",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(moveit_msgs::srv::GetMultiStateValidity_Request, group_name),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "constraints",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<moveit_msgs::msg::Constraints>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(moveit_msgs::srv::GetMultiStateValidity_Request, constraints),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers GetMultiStateValidity_Request_message_members = {
  "moveit_msgs::srv",  // message namespace
  "GetMultiStateValidity_Request",  // message name
  4,  // number of fields
  sizeof(moveit_msgs::srv::GetMultiStateValidity_Request),
  false,  // has_any_key_member_
  GetMultiStateValidity_Request_message_member_array,  // message members
  GetMultiStateValidity_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  GetMultiStateValidity_Request_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t GetMultiStateValidity_Request_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &GetMultiStateValidity_Request_message_members,
  get_message_typesupport_handle_function,
  &moveit_msgs__srv__GetMultiStateValidity_Request__get_type_hash,
  &moveit_msgs__srv__GetMultiStateValidity_Request__get_type_description,
  &moveit_msgs__srv__GetMultiStateValidity_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace moveit_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<moveit_msgs::srv::GetMultiStateValidity_Request>()
{
  return &::moveit_msgs::srv::rosidl_typesupport_introspection_cpp::GetMultiStateValidity_Request_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, moveit_msgs, srv, GetMultiStateValidity_Request)() {
  return &::moveit_msgs::srv::rosidl_typesupport_introspection_cpp::GetMultiStateValidity_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "moveit_msgs/srv/detail/get_multi_state_validity__functions.h"
// already included above
// #include "moveit_msgs/srv/detail/get_multi_state_validity__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace moveit_msgs
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void GetMultiStateValidity_Response_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) moveit_msgs::srv::GetMultiStateValidity_Response(_init);
}

void GetMultiStateValidity_Response_fini_function(void * message_memory)
{
  auto typed_message = static_cast<moveit_msgs::srv::GetMultiStateValidity_Response *>(message_memory);
  typed_message->~GetMultiStateValidity_Response();
}

size_t size_function__GetMultiStateValidity_Response__contacts(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<moveit_msgs::msg::ContactInformation> *>(untyped_member);
  return member->size();
}

const void * get_const_function__GetMultiStateValidity_Response__contacts(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<moveit_msgs::msg::ContactInformation> *>(untyped_member);
  return &member[index];
}

void * get_function__GetMultiStateValidity_Response__contacts(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<moveit_msgs::msg::ContactInformation> *>(untyped_member);
  return &member[index];
}

void fetch_function__GetMultiStateValidity_Response__contacts(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const moveit_msgs::msg::ContactInformation *>(
    get_const_function__GetMultiStateValidity_Response__contacts(untyped_member, index));
  auto & value = *reinterpret_cast<moveit_msgs::msg::ContactInformation *>(untyped_value);
  value = item;
}

void assign_function__GetMultiStateValidity_Response__contacts(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<moveit_msgs::msg::ContactInformation *>(
    get_function__GetMultiStateValidity_Response__contacts(untyped_member, index));
  const auto & value = *reinterpret_cast<const moveit_msgs::msg::ContactInformation *>(untyped_value);
  item = value;
}

void resize_function__GetMultiStateValidity_Response__contacts(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<moveit_msgs::msg::ContactInformation> *>(untyped_member);
  member->resize(size);
}

size_t size_function__GetMultiStateValidity_Response__cost_sources(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<moveit_msgs::msg::CostSource> *>(untyped_member);
  return member->size();
}

const void * get_const_function__GetMultiStateValidity_Response__cost_sources(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<moveit_msgs::msg::CostSource> *>(untyped_member);
  return &member[index];
}

void * get_function__GetMultiStateValidity_Response__cost_sources(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<moveit_msgs::msg::CostSource> *>(untyped_member);
  return &member[index];
}

void fetch_function__GetMultiStateValidity_Response__cost_sources(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const moveit_msgs::msg::CostSource *>(
    get_const_function__GetMultiStateValidity_Response__cost_sources(untyped_member, index));
  auto & value = *reinterpret_cast<moveit_msgs::msg::CostSource *>(untyped_value);
  value = item;
}

void assign_function__GetMultiStateValidity_Response__cost_sources(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<moveit_msgs::msg::CostSource *>(
    get_function__GetMultiStateValidity_Response__cost_sources(untyped_member, index));
  const auto & value = *reinterpret_cast<const moveit_msgs::msg::CostSource *>(untyped_value);
  item = value;
}

void resize_function__GetMultiStateValidity_Response__cost_sources(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<moveit_msgs::msg::CostSource> *>(untyped_member);
  member->resize(size);
}

size_t size_function__GetMultiStateValidity_Response__constraint_result(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<moveit_msgs::msg::ConstraintEvalResult> *>(untyped_member);
  return member->size();
}

const void * get_const_function__GetMultiStateValidity_Response__constraint_result(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<moveit_msgs::msg::ConstraintEvalResult> *>(untyped_member);
  return &member[index];
}

void * get_function__GetMultiStateValidity_Response__constraint_result(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<moveit_msgs::msg::ConstraintEvalResult> *>(untyped_member);
  return &member[index];
}

void fetch_function__GetMultiStateValidity_Response__constraint_result(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const moveit_msgs::msg::ConstraintEvalResult *>(
    get_const_function__GetMultiStateValidity_Response__constraint_result(untyped_member, index));
  auto & value = *reinterpret_cast<moveit_msgs::msg::ConstraintEvalResult *>(untyped_value);
  value = item;
}

void assign_function__GetMultiStateValidity_Response__constraint_result(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<moveit_msgs::msg::ConstraintEvalResult *>(
    get_function__GetMultiStateValidity_Response__constraint_result(untyped_member, index));
  const auto & value = *reinterpret_cast<const moveit_msgs::msg::ConstraintEvalResult *>(untyped_value);
  item = value;
}

void resize_function__GetMultiStateValidity_Response__constraint_result(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<moveit_msgs::msg::ConstraintEvalResult> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember GetMultiStateValidity_Response_message_member_array[4] = {
  {
    "valid",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(moveit_msgs::srv::GetMultiStateValidity_Response, valid),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "contacts",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<moveit_msgs::msg::ContactInformation>(),  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(moveit_msgs::srv::GetMultiStateValidity_Response, contacts),  // bytes offset in struct
    nullptr,  // default value
    size_function__GetMultiStateValidity_Response__contacts,  // size() function pointer
    get_const_function__GetMultiStateValidity_Response__contacts,  // get_const(index) function pointer
    get_function__GetMultiStateValidity_Response__contacts,  // get(index) function pointer
    fetch_function__GetMultiStateValidity_Response__contacts,  // fetch(index, &value) function pointer
    assign_function__GetMultiStateValidity_Response__contacts,  // assign(index, value) function pointer
    resize_function__GetMultiStateValidity_Response__contacts  // resize(index) function pointer
  },
  {
    "cost_sources",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<moveit_msgs::msg::CostSource>(),  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(moveit_msgs::srv::GetMultiStateValidity_Response, cost_sources),  // bytes offset in struct
    nullptr,  // default value
    size_function__GetMultiStateValidity_Response__cost_sources,  // size() function pointer
    get_const_function__GetMultiStateValidity_Response__cost_sources,  // get_const(index) function pointer
    get_function__GetMultiStateValidity_Response__cost_sources,  // get(index) function pointer
    fetch_function__GetMultiStateValidity_Response__cost_sources,  // fetch(index, &value) function pointer
    assign_function__GetMultiStateValidity_Response__cost_sources,  // assign(index, value) function pointer
    resize_function__GetMultiStateValidity_Response__cost_sources  // resize(index) function pointer
  },
  {
    "constraint_result",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<moveit_msgs::msg::ConstraintEvalResult>(),  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(moveit_msgs::srv::GetMultiStateValidity_Response, constraint_result),  // bytes offset in struct
    nullptr,  // default value
    size_function__GetMultiStateValidity_Response__constraint_result,  // size() function pointer
    get_const_function__GetMultiStateValidity_Response__constraint_result,  // get_const(index) function pointer
    get_function__GetMultiStateValidity_Response__constraint_result,  // get(index) function pointer
    fetch_function__GetMultiStateValidity_Response__constraint_result,  // fetch(index, &value) function pointer
    assign_function__GetMultiStateValidity_Response__constraint_result,  // assign(index, value) function pointer
    resize_function__GetMultiStateValidity_Response__constraint_result  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers GetMultiStateValidity_Response_message_members = {
  "moveit_msgs::srv",  // message namespace
  "GetMultiStateValidity_Response",  // message name
  4,  // number of fields
  sizeof(moveit_msgs::srv::GetMultiStateValidity_Response),
  false,  // has_any_key_member_
  GetMultiStateValidity_Response_message_member_array,  // message members
  GetMultiStateValidity_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  GetMultiStateValidity_Response_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t GetMultiStateValidity_Response_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &GetMultiStateValidity_Response_message_members,
  get_message_typesupport_handle_function,
  &moveit_msgs__srv__GetMultiStateValidity_Response__get_type_hash,
  &moveit_msgs__srv__GetMultiStateValidity_Response__get_type_description,
  &moveit_msgs__srv__GetMultiStateValidity_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace moveit_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<moveit_msgs::srv::GetMultiStateValidity_Response>()
{
  return &::moveit_msgs::srv::rosidl_typesupport_introspection_cpp::GetMultiStateValidity_Response_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, moveit_msgs, srv, GetMultiStateValidity_Response)() {
  return &::moveit_msgs::srv::rosidl_typesupport_introspection_cpp::GetMultiStateValidity_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "moveit_msgs/srv/detail/get_multi_state_validity__functions.h"
// already included above
// #include "moveit_msgs/srv/detail/get_multi_state_validity__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace moveit_msgs
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void GetMultiStateValidity_Event_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) moveit_msgs::srv::GetMultiStateValidity_Event(_init);
}

void GetMultiStateValidity_Event_fini_function(void * message_memory)
{
  auto typed_message = static_cast<moveit_msgs::srv::GetMultiStateValidity_Event *>(message_memory);
  typed_message->~GetMultiStateValidity_Event();
}

size_t size_function__GetMultiStateValidity_Event__request(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<moveit_msgs::srv::GetMultiStateValidity_Request> *>(untyped_member);
  return member->size();
}

const void * get_const_function__GetMultiStateValidity_Event__request(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<moveit_msgs::srv::GetMultiStateValidity_Request> *>(untyped_member);
  return &member[index];
}

void * get_function__GetMultiStateValidity_Event__request(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<moveit_msgs::srv::GetMultiStateValidity_Request> *>(untyped_member);
  return &member[index];
}

void fetch_function__GetMultiStateValidity_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const moveit_msgs::srv::GetMultiStateValidity_Request *>(
    get_const_function__GetMultiStateValidity_Event__request(untyped_member, index));
  auto & value = *reinterpret_cast<moveit_msgs::srv::GetMultiStateValidity_Request *>(untyped_value);
  value = item;
}

void assign_function__GetMultiStateValidity_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<moveit_msgs::srv::GetMultiStateValidity_Request *>(
    get_function__GetMultiStateValidity_Event__request(untyped_member, index));
  const auto & value = *reinterpret_cast<const moveit_msgs::srv::GetMultiStateValidity_Request *>(untyped_value);
  item = value;
}

void resize_function__GetMultiStateValidity_Event__request(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<moveit_msgs::srv::GetMultiStateValidity_Request> *>(untyped_member);
  member->resize(size);
}

size_t size_function__GetMultiStateValidity_Event__response(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<moveit_msgs::srv::GetMultiStateValidity_Response> *>(untyped_member);
  return member->size();
}

const void * get_const_function__GetMultiStateValidity_Event__response(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<moveit_msgs::srv::GetMultiStateValidity_Response> *>(untyped_member);
  return &member[index];
}

void * get_function__GetMultiStateValidity_Event__response(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<moveit_msgs::srv::GetMultiStateValidity_Response> *>(untyped_member);
  return &member[index];
}

void fetch_function__GetMultiStateValidity_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const moveit_msgs::srv::GetMultiStateValidity_Response *>(
    get_const_function__GetMultiStateValidity_Event__response(untyped_member, index));
  auto & value = *reinterpret_cast<moveit_msgs::srv::GetMultiStateValidity_Response *>(untyped_value);
  value = item;
}

void assign_function__GetMultiStateValidity_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<moveit_msgs::srv::GetMultiStateValidity_Response *>(
    get_function__GetMultiStateValidity_Event__response(untyped_member, index));
  const auto & value = *reinterpret_cast<const moveit_msgs::srv::GetMultiStateValidity_Response *>(untyped_value);
  item = value;
}

void resize_function__GetMultiStateValidity_Event__response(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<moveit_msgs::srv::GetMultiStateValidity_Response> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember GetMultiStateValidity_Event_message_member_array[3] = {
  {
    "info",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<service_msgs::msg::ServiceEventInfo>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(moveit_msgs::srv::GetMultiStateValidity_Event, info),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "request",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<moveit_msgs::srv::GetMultiStateValidity_Request>(),  // members of sub message
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(moveit_msgs::srv::GetMultiStateValidity_Event, request),  // bytes offset in struct
    nullptr,  // default value
    size_function__GetMultiStateValidity_Event__request,  // size() function pointer
    get_const_function__GetMultiStateValidity_Event__request,  // get_const(index) function pointer
    get_function__GetMultiStateValidity_Event__request,  // get(index) function pointer
    fetch_function__GetMultiStateValidity_Event__request,  // fetch(index, &value) function pointer
    assign_function__GetMultiStateValidity_Event__request,  // assign(index, value) function pointer
    resize_function__GetMultiStateValidity_Event__request  // resize(index) function pointer
  },
  {
    "response",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<moveit_msgs::srv::GetMultiStateValidity_Response>(),  // members of sub message
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(moveit_msgs::srv::GetMultiStateValidity_Event, response),  // bytes offset in struct
    nullptr,  // default value
    size_function__GetMultiStateValidity_Event__response,  // size() function pointer
    get_const_function__GetMultiStateValidity_Event__response,  // get_const(index) function pointer
    get_function__GetMultiStateValidity_Event__response,  // get(index) function pointer
    fetch_function__GetMultiStateValidity_Event__response,  // fetch(index, &value) function pointer
    assign_function__GetMultiStateValidity_Event__response,  // assign(index, value) function pointer
    resize_function__GetMultiStateValidity_Event__response  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers GetMultiStateValidity_Event_message_members = {
  "moveit_msgs::srv",  // message namespace
  "GetMultiStateValidity_Event",  // message name
  3,  // number of fields
  sizeof(moveit_msgs::srv::GetMultiStateValidity_Event),
  false,  // has_any_key_member_
  GetMultiStateValidity_Event_message_member_array,  // message members
  GetMultiStateValidity_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  GetMultiStateValidity_Event_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t GetMultiStateValidity_Event_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &GetMultiStateValidity_Event_message_members,
  get_message_typesupport_handle_function,
  &moveit_msgs__srv__GetMultiStateValidity_Event__get_type_hash,
  &moveit_msgs__srv__GetMultiStateValidity_Event__get_type_description,
  &moveit_msgs__srv__GetMultiStateValidity_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace moveit_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<moveit_msgs::srv::GetMultiStateValidity_Event>()
{
  return &::moveit_msgs::srv::rosidl_typesupport_introspection_cpp::GetMultiStateValidity_Event_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, moveit_msgs, srv, GetMultiStateValidity_Event)() {
  return &::moveit_msgs::srv::rosidl_typesupport_introspection_cpp::GetMultiStateValidity_Event_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"
// already included above
// #include "moveit_msgs/srv/detail/get_multi_state_validity__functions.h"
// already included above
// #include "moveit_msgs/srv/detail/get_multi_state_validity__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/service_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/service_type_support_decl.hpp"

namespace moveit_msgs
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

// this is intentionally not const to allow initialization later to prevent an initialization race
static ::rosidl_typesupport_introspection_cpp::ServiceMembers GetMultiStateValidity_service_members = {
  "moveit_msgs::srv",  // service namespace
  "GetMultiStateValidity",  // service name
  // the following fields are initialized below on first access
  // see get_service_type_support_handle<moveit_msgs::srv::GetMultiStateValidity>()
  nullptr,  // request message
  nullptr,  // response message
  nullptr,  // event message
};

static const rosidl_service_type_support_t GetMultiStateValidity_service_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &GetMultiStateValidity_service_members,
  get_service_typesupport_handle_function,
  ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<moveit_msgs::srv::GetMultiStateValidity_Request>(),
  ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<moveit_msgs::srv::GetMultiStateValidity_Response>(),
  ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<moveit_msgs::srv::GetMultiStateValidity_Event>(),
  &::rosidl_typesupport_cpp::service_create_event_message<moveit_msgs::srv::GetMultiStateValidity>,
  &::rosidl_typesupport_cpp::service_destroy_event_message<moveit_msgs::srv::GetMultiStateValidity>,
  &moveit_msgs__srv__GetMultiStateValidity__get_type_hash,
  &moveit_msgs__srv__GetMultiStateValidity__get_type_description,
  &moveit_msgs__srv__GetMultiStateValidity__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace moveit_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<moveit_msgs::srv::GetMultiStateValidity>()
{
  // get a handle to the value to be returned
  auto service_type_support =
    &::moveit_msgs::srv::rosidl_typesupport_introspection_cpp::GetMultiStateValidity_service_type_support_handle;
  // get a non-const and properly typed version of the data void *
  auto service_members = const_cast<::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
    static_cast<const ::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
      service_type_support->data));
  // make sure all of the service_members are initialized
  // if they are not, initialize them
  if (
    service_members->request_members_ == nullptr ||
    service_members->response_members_ == nullptr ||
    service_members->event_members_ == nullptr)
  {
    // initialize the request_members_ with the static function from the external library
    service_members->request_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::moveit_msgs::srv::GetMultiStateValidity_Request
      >()->data
      );
    // initialize the response_members_ with the static function from the external library
    service_members->response_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::moveit_msgs::srv::GetMultiStateValidity_Response
      >()->data
      );
    // initialize the event_members_ with the static function from the external library
    service_members->event_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::moveit_msgs::srv::GetMultiStateValidity_Event
      >()->data
      );
  }
  // finally return the properly initialized service_type_support handle
  return service_type_support;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, moveit_msgs, srv, GetMultiStateValidity)() {
  return ::rosidl_typesupport_introspection_cpp::get_service_type_support_handle<moveit_msgs::srv::GetMultiStateValidity>();
}

#ifdef __cplusplus
}
#endif
