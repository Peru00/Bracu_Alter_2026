// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from moveit_msgs:srv/GetGroupUrdf.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "moveit_msgs/srv/detail/get_group_urdf__struct.h"
#include "moveit_msgs/srv/detail/get_group_urdf__type_support.h"
#include "moveit_msgs/srv/detail/get_group_urdf__functions.h"
#include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/message_type_support_dispatch.h"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_c/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace moveit_msgs
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _GetGroupUrdf_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _GetGroupUrdf_Request_type_support_ids_t;

static const _GetGroupUrdf_Request_type_support_ids_t _GetGroupUrdf_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _GetGroupUrdf_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _GetGroupUrdf_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _GetGroupUrdf_Request_type_support_symbol_names_t _GetGroupUrdf_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, moveit_msgs, srv, GetGroupUrdf_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, moveit_msgs, srv, GetGroupUrdf_Request)),
  }
};

typedef struct _GetGroupUrdf_Request_type_support_data_t
{
  void * data[2];
} _GetGroupUrdf_Request_type_support_data_t;

static _GetGroupUrdf_Request_type_support_data_t _GetGroupUrdf_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _GetGroupUrdf_Request_message_typesupport_map = {
  2,
  "moveit_msgs",
  &_GetGroupUrdf_Request_message_typesupport_ids.typesupport_identifier[0],
  &_GetGroupUrdf_Request_message_typesupport_symbol_names.symbol_name[0],
  &_GetGroupUrdf_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t GetGroupUrdf_Request_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_GetGroupUrdf_Request_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &moveit_msgs__srv__GetGroupUrdf_Request__get_type_hash,
  &moveit_msgs__srv__GetGroupUrdf_Request__get_type_description,
  &moveit_msgs__srv__GetGroupUrdf_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace moveit_msgs

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, moveit_msgs, srv, GetGroupUrdf_Request)() {
  return &::moveit_msgs::srv::rosidl_typesupport_c::GetGroupUrdf_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "moveit_msgs/srv/detail/get_group_urdf__struct.h"
// already included above
// #include "moveit_msgs/srv/detail/get_group_urdf__type_support.h"
// already included above
// #include "moveit_msgs/srv/detail/get_group_urdf__functions.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace moveit_msgs
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _GetGroupUrdf_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _GetGroupUrdf_Response_type_support_ids_t;

static const _GetGroupUrdf_Response_type_support_ids_t _GetGroupUrdf_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _GetGroupUrdf_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _GetGroupUrdf_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _GetGroupUrdf_Response_type_support_symbol_names_t _GetGroupUrdf_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, moveit_msgs, srv, GetGroupUrdf_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, moveit_msgs, srv, GetGroupUrdf_Response)),
  }
};

typedef struct _GetGroupUrdf_Response_type_support_data_t
{
  void * data[2];
} _GetGroupUrdf_Response_type_support_data_t;

static _GetGroupUrdf_Response_type_support_data_t _GetGroupUrdf_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _GetGroupUrdf_Response_message_typesupport_map = {
  2,
  "moveit_msgs",
  &_GetGroupUrdf_Response_message_typesupport_ids.typesupport_identifier[0],
  &_GetGroupUrdf_Response_message_typesupport_symbol_names.symbol_name[0],
  &_GetGroupUrdf_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t GetGroupUrdf_Response_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_GetGroupUrdf_Response_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &moveit_msgs__srv__GetGroupUrdf_Response__get_type_hash,
  &moveit_msgs__srv__GetGroupUrdf_Response__get_type_description,
  &moveit_msgs__srv__GetGroupUrdf_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace moveit_msgs

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, moveit_msgs, srv, GetGroupUrdf_Response)() {
  return &::moveit_msgs::srv::rosidl_typesupport_c::GetGroupUrdf_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "moveit_msgs/srv/detail/get_group_urdf__struct.h"
// already included above
// #include "moveit_msgs/srv/detail/get_group_urdf__type_support.h"
// already included above
// #include "moveit_msgs/srv/detail/get_group_urdf__functions.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace moveit_msgs
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _GetGroupUrdf_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _GetGroupUrdf_Event_type_support_ids_t;

static const _GetGroupUrdf_Event_type_support_ids_t _GetGroupUrdf_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _GetGroupUrdf_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _GetGroupUrdf_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _GetGroupUrdf_Event_type_support_symbol_names_t _GetGroupUrdf_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, moveit_msgs, srv, GetGroupUrdf_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, moveit_msgs, srv, GetGroupUrdf_Event)),
  }
};

typedef struct _GetGroupUrdf_Event_type_support_data_t
{
  void * data[2];
} _GetGroupUrdf_Event_type_support_data_t;

static _GetGroupUrdf_Event_type_support_data_t _GetGroupUrdf_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _GetGroupUrdf_Event_message_typesupport_map = {
  2,
  "moveit_msgs",
  &_GetGroupUrdf_Event_message_typesupport_ids.typesupport_identifier[0],
  &_GetGroupUrdf_Event_message_typesupport_symbol_names.symbol_name[0],
  &_GetGroupUrdf_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t GetGroupUrdf_Event_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_GetGroupUrdf_Event_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &moveit_msgs__srv__GetGroupUrdf_Event__get_type_hash,
  &moveit_msgs__srv__GetGroupUrdf_Event__get_type_description,
  &moveit_msgs__srv__GetGroupUrdf_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace moveit_msgs

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, moveit_msgs, srv, GetGroupUrdf_Event)() {
  return &::moveit_msgs::srv::rosidl_typesupport_c::GetGroupUrdf_Event_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "moveit_msgs/srv/detail/get_group_urdf__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/service_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
#include "service_msgs/msg/service_event_info.h"
#include "builtin_interfaces/msg/time.h"

namespace moveit_msgs
{

namespace srv
{

namespace rosidl_typesupport_c
{
typedef struct _GetGroupUrdf_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _GetGroupUrdf_type_support_ids_t;

static const _GetGroupUrdf_type_support_ids_t _GetGroupUrdf_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _GetGroupUrdf_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _GetGroupUrdf_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _GetGroupUrdf_type_support_symbol_names_t _GetGroupUrdf_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, moveit_msgs, srv, GetGroupUrdf)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, moveit_msgs, srv, GetGroupUrdf)),
  }
};

typedef struct _GetGroupUrdf_type_support_data_t
{
  void * data[2];
} _GetGroupUrdf_type_support_data_t;

static _GetGroupUrdf_type_support_data_t _GetGroupUrdf_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _GetGroupUrdf_service_typesupport_map = {
  2,
  "moveit_msgs",
  &_GetGroupUrdf_service_typesupport_ids.typesupport_identifier[0],
  &_GetGroupUrdf_service_typesupport_symbol_names.symbol_name[0],
  &_GetGroupUrdf_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t GetGroupUrdf_service_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_GetGroupUrdf_service_typesupport_map),
  rosidl_typesupport_c__get_service_typesupport_handle_function,
  &GetGroupUrdf_Request_message_type_support_handle,
  &GetGroupUrdf_Response_message_type_support_handle,
  &GetGroupUrdf_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    moveit_msgs,
    srv,
    GetGroupUrdf
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    moveit_msgs,
    srv,
    GetGroupUrdf
  ),
  &moveit_msgs__srv__GetGroupUrdf__get_type_hash,
  &moveit_msgs__srv__GetGroupUrdf__get_type_description,
  &moveit_msgs__srv__GetGroupUrdf__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace moveit_msgs

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, moveit_msgs, srv, GetGroupUrdf)() {
  return &::moveit_msgs::srv::rosidl_typesupport_c::GetGroupUrdf_service_type_support_handle;
}

#ifdef __cplusplus
}
#endif
