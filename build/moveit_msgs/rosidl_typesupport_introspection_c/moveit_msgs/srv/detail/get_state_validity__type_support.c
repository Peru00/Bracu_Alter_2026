// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from moveit_msgs:srv/GetStateValidity.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "moveit_msgs/srv/detail/get_state_validity__rosidl_typesupport_introspection_c.h"
#include "moveit_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "moveit_msgs/srv/detail/get_state_validity__functions.h"
#include "moveit_msgs/srv/detail/get_state_validity__struct.h"


// Include directives for member types
// Member `robot_state`
#include "moveit_msgs/msg/robot_state.h"
// Member `robot_state`
#include "moveit_msgs/msg/detail/robot_state__rosidl_typesupport_introspection_c.h"
// Member `group_name`
#include "rosidl_runtime_c/string_functions.h"
// Member `constraints`
#include "moveit_msgs/msg/constraints.h"
// Member `constraints`
#include "moveit_msgs/msg/detail/constraints__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void moveit_msgs__srv__GetStateValidity_Request__rosidl_typesupport_introspection_c__GetStateValidity_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  moveit_msgs__srv__GetStateValidity_Request__init(message_memory);
}

void moveit_msgs__srv__GetStateValidity_Request__rosidl_typesupport_introspection_c__GetStateValidity_Request_fini_function(void * message_memory)
{
  moveit_msgs__srv__GetStateValidity_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember moveit_msgs__srv__GetStateValidity_Request__rosidl_typesupport_introspection_c__GetStateValidity_Request_message_member_array[3] = {
  {
    "robot_state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(moveit_msgs__srv__GetStateValidity_Request, robot_state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "group_name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(moveit_msgs__srv__GetStateValidity_Request, group_name),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "constraints",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(moveit_msgs__srv__GetStateValidity_Request, constraints),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers moveit_msgs__srv__GetStateValidity_Request__rosidl_typesupport_introspection_c__GetStateValidity_Request_message_members = {
  "moveit_msgs__srv",  // message namespace
  "GetStateValidity_Request",  // message name
  3,  // number of fields
  sizeof(moveit_msgs__srv__GetStateValidity_Request),
  false,  // has_any_key_member_
  moveit_msgs__srv__GetStateValidity_Request__rosidl_typesupport_introspection_c__GetStateValidity_Request_message_member_array,  // message members
  moveit_msgs__srv__GetStateValidity_Request__rosidl_typesupport_introspection_c__GetStateValidity_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  moveit_msgs__srv__GetStateValidity_Request__rosidl_typesupport_introspection_c__GetStateValidity_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t moveit_msgs__srv__GetStateValidity_Request__rosidl_typesupport_introspection_c__GetStateValidity_Request_message_type_support_handle = {
  0,
  &moveit_msgs__srv__GetStateValidity_Request__rosidl_typesupport_introspection_c__GetStateValidity_Request_message_members,
  get_message_typesupport_handle_function,
  &moveit_msgs__srv__GetStateValidity_Request__get_type_hash,
  &moveit_msgs__srv__GetStateValidity_Request__get_type_description,
  &moveit_msgs__srv__GetStateValidity_Request__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_moveit_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, moveit_msgs, srv, GetStateValidity_Request)() {
  moveit_msgs__srv__GetStateValidity_Request__rosidl_typesupport_introspection_c__GetStateValidity_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, moveit_msgs, msg, RobotState)();
  moveit_msgs__srv__GetStateValidity_Request__rosidl_typesupport_introspection_c__GetStateValidity_Request_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, moveit_msgs, msg, Constraints)();
  if (!moveit_msgs__srv__GetStateValidity_Request__rosidl_typesupport_introspection_c__GetStateValidity_Request_message_type_support_handle.typesupport_identifier) {
    moveit_msgs__srv__GetStateValidity_Request__rosidl_typesupport_introspection_c__GetStateValidity_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &moveit_msgs__srv__GetStateValidity_Request__rosidl_typesupport_introspection_c__GetStateValidity_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "moveit_msgs/srv/detail/get_state_validity__rosidl_typesupport_introspection_c.h"
// already included above
// #include "moveit_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "moveit_msgs/srv/detail/get_state_validity__functions.h"
// already included above
// #include "moveit_msgs/srv/detail/get_state_validity__struct.h"


// Include directives for member types
// Member `contacts`
#include "moveit_msgs/msg/contact_information.h"
// Member `contacts`
#include "moveit_msgs/msg/detail/contact_information__rosidl_typesupport_introspection_c.h"
// Member `cost_sources`
#include "moveit_msgs/msg/cost_source.h"
// Member `cost_sources`
#include "moveit_msgs/msg/detail/cost_source__rosidl_typesupport_introspection_c.h"
// Member `constraint_result`
#include "moveit_msgs/msg/constraint_eval_result.h"
// Member `constraint_result`
#include "moveit_msgs/msg/detail/constraint_eval_result__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void moveit_msgs__srv__GetStateValidity_Response__rosidl_typesupport_introspection_c__GetStateValidity_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  moveit_msgs__srv__GetStateValidity_Response__init(message_memory);
}

void moveit_msgs__srv__GetStateValidity_Response__rosidl_typesupport_introspection_c__GetStateValidity_Response_fini_function(void * message_memory)
{
  moveit_msgs__srv__GetStateValidity_Response__fini(message_memory);
}

size_t moveit_msgs__srv__GetStateValidity_Response__rosidl_typesupport_introspection_c__size_function__GetStateValidity_Response__contacts(
  const void * untyped_member)
{
  const moveit_msgs__msg__ContactInformation__Sequence * member =
    (const moveit_msgs__msg__ContactInformation__Sequence *)(untyped_member);
  return member->size;
}

const void * moveit_msgs__srv__GetStateValidity_Response__rosidl_typesupport_introspection_c__get_const_function__GetStateValidity_Response__contacts(
  const void * untyped_member, size_t index)
{
  const moveit_msgs__msg__ContactInformation__Sequence * member =
    (const moveit_msgs__msg__ContactInformation__Sequence *)(untyped_member);
  return &member->data[index];
}

void * moveit_msgs__srv__GetStateValidity_Response__rosidl_typesupport_introspection_c__get_function__GetStateValidity_Response__contacts(
  void * untyped_member, size_t index)
{
  moveit_msgs__msg__ContactInformation__Sequence * member =
    (moveit_msgs__msg__ContactInformation__Sequence *)(untyped_member);
  return &member->data[index];
}

void moveit_msgs__srv__GetStateValidity_Response__rosidl_typesupport_introspection_c__fetch_function__GetStateValidity_Response__contacts(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const moveit_msgs__msg__ContactInformation * item =
    ((const moveit_msgs__msg__ContactInformation *)
    moveit_msgs__srv__GetStateValidity_Response__rosidl_typesupport_introspection_c__get_const_function__GetStateValidity_Response__contacts(untyped_member, index));
  moveit_msgs__msg__ContactInformation * value =
    (moveit_msgs__msg__ContactInformation *)(untyped_value);
  *value = *item;
}

void moveit_msgs__srv__GetStateValidity_Response__rosidl_typesupport_introspection_c__assign_function__GetStateValidity_Response__contacts(
  void * untyped_member, size_t index, const void * untyped_value)
{
  moveit_msgs__msg__ContactInformation * item =
    ((moveit_msgs__msg__ContactInformation *)
    moveit_msgs__srv__GetStateValidity_Response__rosidl_typesupport_introspection_c__get_function__GetStateValidity_Response__contacts(untyped_member, index));
  const moveit_msgs__msg__ContactInformation * value =
    (const moveit_msgs__msg__ContactInformation *)(untyped_value);
  *item = *value;
}

bool moveit_msgs__srv__GetStateValidity_Response__rosidl_typesupport_introspection_c__resize_function__GetStateValidity_Response__contacts(
  void * untyped_member, size_t size)
{
  moveit_msgs__msg__ContactInformation__Sequence * member =
    (moveit_msgs__msg__ContactInformation__Sequence *)(untyped_member);
  moveit_msgs__msg__ContactInformation__Sequence__fini(member);
  return moveit_msgs__msg__ContactInformation__Sequence__init(member, size);
}

size_t moveit_msgs__srv__GetStateValidity_Response__rosidl_typesupport_introspection_c__size_function__GetStateValidity_Response__cost_sources(
  const void * untyped_member)
{
  const moveit_msgs__msg__CostSource__Sequence * member =
    (const moveit_msgs__msg__CostSource__Sequence *)(untyped_member);
  return member->size;
}

const void * moveit_msgs__srv__GetStateValidity_Response__rosidl_typesupport_introspection_c__get_const_function__GetStateValidity_Response__cost_sources(
  const void * untyped_member, size_t index)
{
  const moveit_msgs__msg__CostSource__Sequence * member =
    (const moveit_msgs__msg__CostSource__Sequence *)(untyped_member);
  return &member->data[index];
}

void * moveit_msgs__srv__GetStateValidity_Response__rosidl_typesupport_introspection_c__get_function__GetStateValidity_Response__cost_sources(
  void * untyped_member, size_t index)
{
  moveit_msgs__msg__CostSource__Sequence * member =
    (moveit_msgs__msg__CostSource__Sequence *)(untyped_member);
  return &member->data[index];
}

void moveit_msgs__srv__GetStateValidity_Response__rosidl_typesupport_introspection_c__fetch_function__GetStateValidity_Response__cost_sources(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const moveit_msgs__msg__CostSource * item =
    ((const moveit_msgs__msg__CostSource *)
    moveit_msgs__srv__GetStateValidity_Response__rosidl_typesupport_introspection_c__get_const_function__GetStateValidity_Response__cost_sources(untyped_member, index));
  moveit_msgs__msg__CostSource * value =
    (moveit_msgs__msg__CostSource *)(untyped_value);
  *value = *item;
}

void moveit_msgs__srv__GetStateValidity_Response__rosidl_typesupport_introspection_c__assign_function__GetStateValidity_Response__cost_sources(
  void * untyped_member, size_t index, const void * untyped_value)
{
  moveit_msgs__msg__CostSource * item =
    ((moveit_msgs__msg__CostSource *)
    moveit_msgs__srv__GetStateValidity_Response__rosidl_typesupport_introspection_c__get_function__GetStateValidity_Response__cost_sources(untyped_member, index));
  const moveit_msgs__msg__CostSource * value =
    (const moveit_msgs__msg__CostSource *)(untyped_value);
  *item = *value;
}

bool moveit_msgs__srv__GetStateValidity_Response__rosidl_typesupport_introspection_c__resize_function__GetStateValidity_Response__cost_sources(
  void * untyped_member, size_t size)
{
  moveit_msgs__msg__CostSource__Sequence * member =
    (moveit_msgs__msg__CostSource__Sequence *)(untyped_member);
  moveit_msgs__msg__CostSource__Sequence__fini(member);
  return moveit_msgs__msg__CostSource__Sequence__init(member, size);
}

size_t moveit_msgs__srv__GetStateValidity_Response__rosidl_typesupport_introspection_c__size_function__GetStateValidity_Response__constraint_result(
  const void * untyped_member)
{
  const moveit_msgs__msg__ConstraintEvalResult__Sequence * member =
    (const moveit_msgs__msg__ConstraintEvalResult__Sequence *)(untyped_member);
  return member->size;
}

const void * moveit_msgs__srv__GetStateValidity_Response__rosidl_typesupport_introspection_c__get_const_function__GetStateValidity_Response__constraint_result(
  const void * untyped_member, size_t index)
{
  const moveit_msgs__msg__ConstraintEvalResult__Sequence * member =
    (const moveit_msgs__msg__ConstraintEvalResult__Sequence *)(untyped_member);
  return &member->data[index];
}

void * moveit_msgs__srv__GetStateValidity_Response__rosidl_typesupport_introspection_c__get_function__GetStateValidity_Response__constraint_result(
  void * untyped_member, size_t index)
{
  moveit_msgs__msg__ConstraintEvalResult__Sequence * member =
    (moveit_msgs__msg__ConstraintEvalResult__Sequence *)(untyped_member);
  return &member->data[index];
}

void moveit_msgs__srv__GetStateValidity_Response__rosidl_typesupport_introspection_c__fetch_function__GetStateValidity_Response__constraint_result(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const moveit_msgs__msg__ConstraintEvalResult * item =
    ((const moveit_msgs__msg__ConstraintEvalResult *)
    moveit_msgs__srv__GetStateValidity_Response__rosidl_typesupport_introspection_c__get_const_function__GetStateValidity_Response__constraint_result(untyped_member, index));
  moveit_msgs__msg__ConstraintEvalResult * value =
    (moveit_msgs__msg__ConstraintEvalResult *)(untyped_value);
  *value = *item;
}

void moveit_msgs__srv__GetStateValidity_Response__rosidl_typesupport_introspection_c__assign_function__GetStateValidity_Response__constraint_result(
  void * untyped_member, size_t index, const void * untyped_value)
{
  moveit_msgs__msg__ConstraintEvalResult * item =
    ((moveit_msgs__msg__ConstraintEvalResult *)
    moveit_msgs__srv__GetStateValidity_Response__rosidl_typesupport_introspection_c__get_function__GetStateValidity_Response__constraint_result(untyped_member, index));
  const moveit_msgs__msg__ConstraintEvalResult * value =
    (const moveit_msgs__msg__ConstraintEvalResult *)(untyped_value);
  *item = *value;
}

bool moveit_msgs__srv__GetStateValidity_Response__rosidl_typesupport_introspection_c__resize_function__GetStateValidity_Response__constraint_result(
  void * untyped_member, size_t size)
{
  moveit_msgs__msg__ConstraintEvalResult__Sequence * member =
    (moveit_msgs__msg__ConstraintEvalResult__Sequence *)(untyped_member);
  moveit_msgs__msg__ConstraintEvalResult__Sequence__fini(member);
  return moveit_msgs__msg__ConstraintEvalResult__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember moveit_msgs__srv__GetStateValidity_Response__rosidl_typesupport_introspection_c__GetStateValidity_Response_message_member_array[4] = {
  {
    "valid",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(moveit_msgs__srv__GetStateValidity_Response, valid),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "contacts",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(moveit_msgs__srv__GetStateValidity_Response, contacts),  // bytes offset in struct
    NULL,  // default value
    moveit_msgs__srv__GetStateValidity_Response__rosidl_typesupport_introspection_c__size_function__GetStateValidity_Response__contacts,  // size() function pointer
    moveit_msgs__srv__GetStateValidity_Response__rosidl_typesupport_introspection_c__get_const_function__GetStateValidity_Response__contacts,  // get_const(index) function pointer
    moveit_msgs__srv__GetStateValidity_Response__rosidl_typesupport_introspection_c__get_function__GetStateValidity_Response__contacts,  // get(index) function pointer
    moveit_msgs__srv__GetStateValidity_Response__rosidl_typesupport_introspection_c__fetch_function__GetStateValidity_Response__contacts,  // fetch(index, &value) function pointer
    moveit_msgs__srv__GetStateValidity_Response__rosidl_typesupport_introspection_c__assign_function__GetStateValidity_Response__contacts,  // assign(index, value) function pointer
    moveit_msgs__srv__GetStateValidity_Response__rosidl_typesupport_introspection_c__resize_function__GetStateValidity_Response__contacts  // resize(index) function pointer
  },
  {
    "cost_sources",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(moveit_msgs__srv__GetStateValidity_Response, cost_sources),  // bytes offset in struct
    NULL,  // default value
    moveit_msgs__srv__GetStateValidity_Response__rosidl_typesupport_introspection_c__size_function__GetStateValidity_Response__cost_sources,  // size() function pointer
    moveit_msgs__srv__GetStateValidity_Response__rosidl_typesupport_introspection_c__get_const_function__GetStateValidity_Response__cost_sources,  // get_const(index) function pointer
    moveit_msgs__srv__GetStateValidity_Response__rosidl_typesupport_introspection_c__get_function__GetStateValidity_Response__cost_sources,  // get(index) function pointer
    moveit_msgs__srv__GetStateValidity_Response__rosidl_typesupport_introspection_c__fetch_function__GetStateValidity_Response__cost_sources,  // fetch(index, &value) function pointer
    moveit_msgs__srv__GetStateValidity_Response__rosidl_typesupport_introspection_c__assign_function__GetStateValidity_Response__cost_sources,  // assign(index, value) function pointer
    moveit_msgs__srv__GetStateValidity_Response__rosidl_typesupport_introspection_c__resize_function__GetStateValidity_Response__cost_sources  // resize(index) function pointer
  },
  {
    "constraint_result",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(moveit_msgs__srv__GetStateValidity_Response, constraint_result),  // bytes offset in struct
    NULL,  // default value
    moveit_msgs__srv__GetStateValidity_Response__rosidl_typesupport_introspection_c__size_function__GetStateValidity_Response__constraint_result,  // size() function pointer
    moveit_msgs__srv__GetStateValidity_Response__rosidl_typesupport_introspection_c__get_const_function__GetStateValidity_Response__constraint_result,  // get_const(index) function pointer
    moveit_msgs__srv__GetStateValidity_Response__rosidl_typesupport_introspection_c__get_function__GetStateValidity_Response__constraint_result,  // get(index) function pointer
    moveit_msgs__srv__GetStateValidity_Response__rosidl_typesupport_introspection_c__fetch_function__GetStateValidity_Response__constraint_result,  // fetch(index, &value) function pointer
    moveit_msgs__srv__GetStateValidity_Response__rosidl_typesupport_introspection_c__assign_function__GetStateValidity_Response__constraint_result,  // assign(index, value) function pointer
    moveit_msgs__srv__GetStateValidity_Response__rosidl_typesupport_introspection_c__resize_function__GetStateValidity_Response__constraint_result  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers moveit_msgs__srv__GetStateValidity_Response__rosidl_typesupport_introspection_c__GetStateValidity_Response_message_members = {
  "moveit_msgs__srv",  // message namespace
  "GetStateValidity_Response",  // message name
  4,  // number of fields
  sizeof(moveit_msgs__srv__GetStateValidity_Response),
  false,  // has_any_key_member_
  moveit_msgs__srv__GetStateValidity_Response__rosidl_typesupport_introspection_c__GetStateValidity_Response_message_member_array,  // message members
  moveit_msgs__srv__GetStateValidity_Response__rosidl_typesupport_introspection_c__GetStateValidity_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  moveit_msgs__srv__GetStateValidity_Response__rosidl_typesupport_introspection_c__GetStateValidity_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t moveit_msgs__srv__GetStateValidity_Response__rosidl_typesupport_introspection_c__GetStateValidity_Response_message_type_support_handle = {
  0,
  &moveit_msgs__srv__GetStateValidity_Response__rosidl_typesupport_introspection_c__GetStateValidity_Response_message_members,
  get_message_typesupport_handle_function,
  &moveit_msgs__srv__GetStateValidity_Response__get_type_hash,
  &moveit_msgs__srv__GetStateValidity_Response__get_type_description,
  &moveit_msgs__srv__GetStateValidity_Response__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_moveit_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, moveit_msgs, srv, GetStateValidity_Response)() {
  moveit_msgs__srv__GetStateValidity_Response__rosidl_typesupport_introspection_c__GetStateValidity_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, moveit_msgs, msg, ContactInformation)();
  moveit_msgs__srv__GetStateValidity_Response__rosidl_typesupport_introspection_c__GetStateValidity_Response_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, moveit_msgs, msg, CostSource)();
  moveit_msgs__srv__GetStateValidity_Response__rosidl_typesupport_introspection_c__GetStateValidity_Response_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, moveit_msgs, msg, ConstraintEvalResult)();
  if (!moveit_msgs__srv__GetStateValidity_Response__rosidl_typesupport_introspection_c__GetStateValidity_Response_message_type_support_handle.typesupport_identifier) {
    moveit_msgs__srv__GetStateValidity_Response__rosidl_typesupport_introspection_c__GetStateValidity_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &moveit_msgs__srv__GetStateValidity_Response__rosidl_typesupport_introspection_c__GetStateValidity_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "moveit_msgs/srv/detail/get_state_validity__rosidl_typesupport_introspection_c.h"
// already included above
// #include "moveit_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "moveit_msgs/srv/detail/get_state_validity__functions.h"
// already included above
// #include "moveit_msgs/srv/detail/get_state_validity__struct.h"


// Include directives for member types
// Member `info`
#include "service_msgs/msg/service_event_info.h"
// Member `info`
#include "service_msgs/msg/detail/service_event_info__rosidl_typesupport_introspection_c.h"
// Member `request`
// Member `response`
#include "moveit_msgs/srv/get_state_validity.h"
// Member `request`
// Member `response`
// already included above
// #include "moveit_msgs/srv/detail/get_state_validity__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void moveit_msgs__srv__GetStateValidity_Event__rosidl_typesupport_introspection_c__GetStateValidity_Event_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  moveit_msgs__srv__GetStateValidity_Event__init(message_memory);
}

void moveit_msgs__srv__GetStateValidity_Event__rosidl_typesupport_introspection_c__GetStateValidity_Event_fini_function(void * message_memory)
{
  moveit_msgs__srv__GetStateValidity_Event__fini(message_memory);
}

size_t moveit_msgs__srv__GetStateValidity_Event__rosidl_typesupport_introspection_c__size_function__GetStateValidity_Event__request(
  const void * untyped_member)
{
  const moveit_msgs__srv__GetStateValidity_Request__Sequence * member =
    (const moveit_msgs__srv__GetStateValidity_Request__Sequence *)(untyped_member);
  return member->size;
}

const void * moveit_msgs__srv__GetStateValidity_Event__rosidl_typesupport_introspection_c__get_const_function__GetStateValidity_Event__request(
  const void * untyped_member, size_t index)
{
  const moveit_msgs__srv__GetStateValidity_Request__Sequence * member =
    (const moveit_msgs__srv__GetStateValidity_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void * moveit_msgs__srv__GetStateValidity_Event__rosidl_typesupport_introspection_c__get_function__GetStateValidity_Event__request(
  void * untyped_member, size_t index)
{
  moveit_msgs__srv__GetStateValidity_Request__Sequence * member =
    (moveit_msgs__srv__GetStateValidity_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void moveit_msgs__srv__GetStateValidity_Event__rosidl_typesupport_introspection_c__fetch_function__GetStateValidity_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const moveit_msgs__srv__GetStateValidity_Request * item =
    ((const moveit_msgs__srv__GetStateValidity_Request *)
    moveit_msgs__srv__GetStateValidity_Event__rosidl_typesupport_introspection_c__get_const_function__GetStateValidity_Event__request(untyped_member, index));
  moveit_msgs__srv__GetStateValidity_Request * value =
    (moveit_msgs__srv__GetStateValidity_Request *)(untyped_value);
  *value = *item;
}

void moveit_msgs__srv__GetStateValidity_Event__rosidl_typesupport_introspection_c__assign_function__GetStateValidity_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  moveit_msgs__srv__GetStateValidity_Request * item =
    ((moveit_msgs__srv__GetStateValidity_Request *)
    moveit_msgs__srv__GetStateValidity_Event__rosidl_typesupport_introspection_c__get_function__GetStateValidity_Event__request(untyped_member, index));
  const moveit_msgs__srv__GetStateValidity_Request * value =
    (const moveit_msgs__srv__GetStateValidity_Request *)(untyped_value);
  *item = *value;
}

bool moveit_msgs__srv__GetStateValidity_Event__rosidl_typesupport_introspection_c__resize_function__GetStateValidity_Event__request(
  void * untyped_member, size_t size)
{
  moveit_msgs__srv__GetStateValidity_Request__Sequence * member =
    (moveit_msgs__srv__GetStateValidity_Request__Sequence *)(untyped_member);
  moveit_msgs__srv__GetStateValidity_Request__Sequence__fini(member);
  return moveit_msgs__srv__GetStateValidity_Request__Sequence__init(member, size);
}

size_t moveit_msgs__srv__GetStateValidity_Event__rosidl_typesupport_introspection_c__size_function__GetStateValidity_Event__response(
  const void * untyped_member)
{
  const moveit_msgs__srv__GetStateValidity_Response__Sequence * member =
    (const moveit_msgs__srv__GetStateValidity_Response__Sequence *)(untyped_member);
  return member->size;
}

const void * moveit_msgs__srv__GetStateValidity_Event__rosidl_typesupport_introspection_c__get_const_function__GetStateValidity_Event__response(
  const void * untyped_member, size_t index)
{
  const moveit_msgs__srv__GetStateValidity_Response__Sequence * member =
    (const moveit_msgs__srv__GetStateValidity_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void * moveit_msgs__srv__GetStateValidity_Event__rosidl_typesupport_introspection_c__get_function__GetStateValidity_Event__response(
  void * untyped_member, size_t index)
{
  moveit_msgs__srv__GetStateValidity_Response__Sequence * member =
    (moveit_msgs__srv__GetStateValidity_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void moveit_msgs__srv__GetStateValidity_Event__rosidl_typesupport_introspection_c__fetch_function__GetStateValidity_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const moveit_msgs__srv__GetStateValidity_Response * item =
    ((const moveit_msgs__srv__GetStateValidity_Response *)
    moveit_msgs__srv__GetStateValidity_Event__rosidl_typesupport_introspection_c__get_const_function__GetStateValidity_Event__response(untyped_member, index));
  moveit_msgs__srv__GetStateValidity_Response * value =
    (moveit_msgs__srv__GetStateValidity_Response *)(untyped_value);
  *value = *item;
}

void moveit_msgs__srv__GetStateValidity_Event__rosidl_typesupport_introspection_c__assign_function__GetStateValidity_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  moveit_msgs__srv__GetStateValidity_Response * item =
    ((moveit_msgs__srv__GetStateValidity_Response *)
    moveit_msgs__srv__GetStateValidity_Event__rosidl_typesupport_introspection_c__get_function__GetStateValidity_Event__response(untyped_member, index));
  const moveit_msgs__srv__GetStateValidity_Response * value =
    (const moveit_msgs__srv__GetStateValidity_Response *)(untyped_value);
  *item = *value;
}

bool moveit_msgs__srv__GetStateValidity_Event__rosidl_typesupport_introspection_c__resize_function__GetStateValidity_Event__response(
  void * untyped_member, size_t size)
{
  moveit_msgs__srv__GetStateValidity_Response__Sequence * member =
    (moveit_msgs__srv__GetStateValidity_Response__Sequence *)(untyped_member);
  moveit_msgs__srv__GetStateValidity_Response__Sequence__fini(member);
  return moveit_msgs__srv__GetStateValidity_Response__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember moveit_msgs__srv__GetStateValidity_Event__rosidl_typesupport_introspection_c__GetStateValidity_Event_message_member_array[3] = {
  {
    "info",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(moveit_msgs__srv__GetStateValidity_Event, info),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "request",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(moveit_msgs__srv__GetStateValidity_Event, request),  // bytes offset in struct
    NULL,  // default value
    moveit_msgs__srv__GetStateValidity_Event__rosidl_typesupport_introspection_c__size_function__GetStateValidity_Event__request,  // size() function pointer
    moveit_msgs__srv__GetStateValidity_Event__rosidl_typesupport_introspection_c__get_const_function__GetStateValidity_Event__request,  // get_const(index) function pointer
    moveit_msgs__srv__GetStateValidity_Event__rosidl_typesupport_introspection_c__get_function__GetStateValidity_Event__request,  // get(index) function pointer
    moveit_msgs__srv__GetStateValidity_Event__rosidl_typesupport_introspection_c__fetch_function__GetStateValidity_Event__request,  // fetch(index, &value) function pointer
    moveit_msgs__srv__GetStateValidity_Event__rosidl_typesupport_introspection_c__assign_function__GetStateValidity_Event__request,  // assign(index, value) function pointer
    moveit_msgs__srv__GetStateValidity_Event__rosidl_typesupport_introspection_c__resize_function__GetStateValidity_Event__request  // resize(index) function pointer
  },
  {
    "response",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(moveit_msgs__srv__GetStateValidity_Event, response),  // bytes offset in struct
    NULL,  // default value
    moveit_msgs__srv__GetStateValidity_Event__rosidl_typesupport_introspection_c__size_function__GetStateValidity_Event__response,  // size() function pointer
    moveit_msgs__srv__GetStateValidity_Event__rosidl_typesupport_introspection_c__get_const_function__GetStateValidity_Event__response,  // get_const(index) function pointer
    moveit_msgs__srv__GetStateValidity_Event__rosidl_typesupport_introspection_c__get_function__GetStateValidity_Event__response,  // get(index) function pointer
    moveit_msgs__srv__GetStateValidity_Event__rosidl_typesupport_introspection_c__fetch_function__GetStateValidity_Event__response,  // fetch(index, &value) function pointer
    moveit_msgs__srv__GetStateValidity_Event__rosidl_typesupport_introspection_c__assign_function__GetStateValidity_Event__response,  // assign(index, value) function pointer
    moveit_msgs__srv__GetStateValidity_Event__rosidl_typesupport_introspection_c__resize_function__GetStateValidity_Event__response  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers moveit_msgs__srv__GetStateValidity_Event__rosidl_typesupport_introspection_c__GetStateValidity_Event_message_members = {
  "moveit_msgs__srv",  // message namespace
  "GetStateValidity_Event",  // message name
  3,  // number of fields
  sizeof(moveit_msgs__srv__GetStateValidity_Event),
  false,  // has_any_key_member_
  moveit_msgs__srv__GetStateValidity_Event__rosidl_typesupport_introspection_c__GetStateValidity_Event_message_member_array,  // message members
  moveit_msgs__srv__GetStateValidity_Event__rosidl_typesupport_introspection_c__GetStateValidity_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  moveit_msgs__srv__GetStateValidity_Event__rosidl_typesupport_introspection_c__GetStateValidity_Event_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t moveit_msgs__srv__GetStateValidity_Event__rosidl_typesupport_introspection_c__GetStateValidity_Event_message_type_support_handle = {
  0,
  &moveit_msgs__srv__GetStateValidity_Event__rosidl_typesupport_introspection_c__GetStateValidity_Event_message_members,
  get_message_typesupport_handle_function,
  &moveit_msgs__srv__GetStateValidity_Event__get_type_hash,
  &moveit_msgs__srv__GetStateValidity_Event__get_type_description,
  &moveit_msgs__srv__GetStateValidity_Event__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_moveit_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, moveit_msgs, srv, GetStateValidity_Event)() {
  moveit_msgs__srv__GetStateValidity_Event__rosidl_typesupport_introspection_c__GetStateValidity_Event_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, service_msgs, msg, ServiceEventInfo)();
  moveit_msgs__srv__GetStateValidity_Event__rosidl_typesupport_introspection_c__GetStateValidity_Event_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, moveit_msgs, srv, GetStateValidity_Request)();
  moveit_msgs__srv__GetStateValidity_Event__rosidl_typesupport_introspection_c__GetStateValidity_Event_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, moveit_msgs, srv, GetStateValidity_Response)();
  if (!moveit_msgs__srv__GetStateValidity_Event__rosidl_typesupport_introspection_c__GetStateValidity_Event_message_type_support_handle.typesupport_identifier) {
    moveit_msgs__srv__GetStateValidity_Event__rosidl_typesupport_introspection_c__GetStateValidity_Event_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &moveit_msgs__srv__GetStateValidity_Event__rosidl_typesupport_introspection_c__GetStateValidity_Event_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "moveit_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "moveit_msgs/srv/detail/get_state_validity__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers moveit_msgs__srv__detail__get_state_validity__rosidl_typesupport_introspection_c__GetStateValidity_service_members = {
  "moveit_msgs__srv",  // service namespace
  "GetStateValidity",  // service name
  // the following fields are initialized below on first access
  NULL,  // request message
  // moveit_msgs__srv__detail__get_state_validity__rosidl_typesupport_introspection_c__GetStateValidity_Request_message_type_support_handle,
  NULL,  // response message
  // moveit_msgs__srv__detail__get_state_validity__rosidl_typesupport_introspection_c__GetStateValidity_Response_message_type_support_handle
  NULL  // event_message
  // moveit_msgs__srv__detail__get_state_validity__rosidl_typesupport_introspection_c__GetStateValidity_Response_message_type_support_handle
};


static rosidl_service_type_support_t moveit_msgs__srv__detail__get_state_validity__rosidl_typesupport_introspection_c__GetStateValidity_service_type_support_handle = {
  0,
  &moveit_msgs__srv__detail__get_state_validity__rosidl_typesupport_introspection_c__GetStateValidity_service_members,
  get_service_typesupport_handle_function,
  &moveit_msgs__srv__GetStateValidity_Request__rosidl_typesupport_introspection_c__GetStateValidity_Request_message_type_support_handle,
  &moveit_msgs__srv__GetStateValidity_Response__rosidl_typesupport_introspection_c__GetStateValidity_Response_message_type_support_handle,
  &moveit_msgs__srv__GetStateValidity_Event__rosidl_typesupport_introspection_c__GetStateValidity_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    moveit_msgs,
    srv,
    GetStateValidity
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    moveit_msgs,
    srv,
    GetStateValidity
  ),
  &moveit_msgs__srv__GetStateValidity__get_type_hash,
  &moveit_msgs__srv__GetStateValidity__get_type_description,
  &moveit_msgs__srv__GetStateValidity__get_type_description_sources,
};

// Forward declaration of message type support functions for service members
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, moveit_msgs, srv, GetStateValidity_Request)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, moveit_msgs, srv, GetStateValidity_Response)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, moveit_msgs, srv, GetStateValidity_Event)(void);

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_moveit_msgs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, moveit_msgs, srv, GetStateValidity)(void) {
  if (!moveit_msgs__srv__detail__get_state_validity__rosidl_typesupport_introspection_c__GetStateValidity_service_type_support_handle.typesupport_identifier) {
    moveit_msgs__srv__detail__get_state_validity__rosidl_typesupport_introspection_c__GetStateValidity_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)moveit_msgs__srv__detail__get_state_validity__rosidl_typesupport_introspection_c__GetStateValidity_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, moveit_msgs, srv, GetStateValidity_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, moveit_msgs, srv, GetStateValidity_Response)()->data;
  }
  if (!service_members->event_members_) {
    service_members->event_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, moveit_msgs, srv, GetStateValidity_Event)()->data;
  }

  return &moveit_msgs__srv__detail__get_state_validity__rosidl_typesupport_introspection_c__GetStateValidity_service_type_support_handle;
}
