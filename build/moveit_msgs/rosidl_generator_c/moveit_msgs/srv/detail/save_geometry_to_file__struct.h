// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from moveit_msgs:srv/SaveGeometryToFile.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "moveit_msgs/srv/save_geometry_to_file.h"


#ifndef MOVEIT_MSGS__SRV__DETAIL__SAVE_GEOMETRY_TO_FILE__STRUCT_H_
#define MOVEIT_MSGS__SRV__DETAIL__SAVE_GEOMETRY_TO_FILE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'file_path_and_name'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/SaveGeometryToFile in the package moveit_msgs.
typedef struct moveit_msgs__srv__SaveGeometryToFile_Request
{
  rosidl_runtime_c__String file_path_and_name;
} moveit_msgs__srv__SaveGeometryToFile_Request;

// Struct for a sequence of moveit_msgs__srv__SaveGeometryToFile_Request.
typedef struct moveit_msgs__srv__SaveGeometryToFile_Request__Sequence
{
  moveit_msgs__srv__SaveGeometryToFile_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} moveit_msgs__srv__SaveGeometryToFile_Request__Sequence;

// Constants defined in the message

/// Struct defined in srv/SaveGeometryToFile in the package moveit_msgs.
typedef struct moveit_msgs__srv__SaveGeometryToFile_Response
{
  bool success;
} moveit_msgs__srv__SaveGeometryToFile_Response;

// Struct for a sequence of moveit_msgs__srv__SaveGeometryToFile_Response.
typedef struct moveit_msgs__srv__SaveGeometryToFile_Response__Sequence
{
  moveit_msgs__srv__SaveGeometryToFile_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} moveit_msgs__srv__SaveGeometryToFile_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  moveit_msgs__srv__SaveGeometryToFile_Event__request__MAX_SIZE = 1
};
// response
enum
{
  moveit_msgs__srv__SaveGeometryToFile_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/SaveGeometryToFile in the package moveit_msgs.
typedef struct moveit_msgs__srv__SaveGeometryToFile_Event
{
  service_msgs__msg__ServiceEventInfo info;
  moveit_msgs__srv__SaveGeometryToFile_Request__Sequence request;
  moveit_msgs__srv__SaveGeometryToFile_Response__Sequence response;
} moveit_msgs__srv__SaveGeometryToFile_Event;

// Struct for a sequence of moveit_msgs__srv__SaveGeometryToFile_Event.
typedef struct moveit_msgs__srv__SaveGeometryToFile_Event__Sequence
{
  moveit_msgs__srv__SaveGeometryToFile_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} moveit_msgs__srv__SaveGeometryToFile_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MOVEIT_MSGS__SRV__DETAIL__SAVE_GEOMETRY_TO_FILE__STRUCT_H_
