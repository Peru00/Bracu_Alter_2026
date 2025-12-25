// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from moveit_msgs:srv/GetMultiStateValidity.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "moveit_msgs/srv/get_multi_state_validity.h"


#ifndef MOVEIT_MSGS__SRV__DETAIL__GET_MULTI_STATE_VALIDITY__STRUCT_H_
#define MOVEIT_MSGS__SRV__DETAIL__GET_MULTI_STATE_VALIDITY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'robot_state'
#include "moveit_msgs/msg/detail/robot_state__struct.h"
// Member 'joint_states'
#include "sensor_msgs/msg/detail/joint_state__struct.h"
// Member 'group_name'
#include "rosidl_runtime_c/string.h"
// Member 'constraints'
#include "moveit_msgs/msg/detail/constraints__struct.h"

/// Struct defined in srv/GetMultiStateValidity in the package moveit_msgs.
typedef struct moveit_msgs__srv__GetMultiStateValidity_Request
{
  moveit_msgs__msg__RobotState robot_state;
  sensor_msgs__msg__JointState__Sequence joint_states;
  rosidl_runtime_c__String group_name;
  moveit_msgs__msg__Constraints constraints;
} moveit_msgs__srv__GetMultiStateValidity_Request;

// Struct for a sequence of moveit_msgs__srv__GetMultiStateValidity_Request.
typedef struct moveit_msgs__srv__GetMultiStateValidity_Request__Sequence
{
  moveit_msgs__srv__GetMultiStateValidity_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} moveit_msgs__srv__GetMultiStateValidity_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'contacts'
#include "moveit_msgs/msg/detail/contact_information__struct.h"
// Member 'cost_sources'
#include "moveit_msgs/msg/detail/cost_source__struct.h"
// Member 'constraint_result'
#include "moveit_msgs/msg/detail/constraint_eval_result__struct.h"

/// Struct defined in srv/GetMultiStateValidity in the package moveit_msgs.
typedef struct moveit_msgs__srv__GetMultiStateValidity_Response
{
  bool valid;
  moveit_msgs__msg__ContactInformation__Sequence contacts;
  moveit_msgs__msg__CostSource__Sequence cost_sources;
  moveit_msgs__msg__ConstraintEvalResult__Sequence constraint_result;
} moveit_msgs__srv__GetMultiStateValidity_Response;

// Struct for a sequence of moveit_msgs__srv__GetMultiStateValidity_Response.
typedef struct moveit_msgs__srv__GetMultiStateValidity_Response__Sequence
{
  moveit_msgs__srv__GetMultiStateValidity_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} moveit_msgs__srv__GetMultiStateValidity_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  moveit_msgs__srv__GetMultiStateValidity_Event__request__MAX_SIZE = 1
};
// response
enum
{
  moveit_msgs__srv__GetMultiStateValidity_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/GetMultiStateValidity in the package moveit_msgs.
typedef struct moveit_msgs__srv__GetMultiStateValidity_Event
{
  service_msgs__msg__ServiceEventInfo info;
  moveit_msgs__srv__GetMultiStateValidity_Request__Sequence request;
  moveit_msgs__srv__GetMultiStateValidity_Response__Sequence response;
} moveit_msgs__srv__GetMultiStateValidity_Event;

// Struct for a sequence of moveit_msgs__srv__GetMultiStateValidity_Event.
typedef struct moveit_msgs__srv__GetMultiStateValidity_Event__Sequence
{
  moveit_msgs__srv__GetMultiStateValidity_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} moveit_msgs__srv__GetMultiStateValidity_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MOVEIT_MSGS__SRV__DETAIL__GET_MULTI_STATE_VALIDITY__STRUCT_H_
