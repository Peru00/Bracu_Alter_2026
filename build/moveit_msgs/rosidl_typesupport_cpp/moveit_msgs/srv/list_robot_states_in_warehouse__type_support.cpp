// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from moveit_msgs:srv/ListRobotStatesInWarehouse.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "moveit_msgs/srv/detail/list_robot_states_in_warehouse__functions.h"
#include "moveit_msgs/srv/detail/list_robot_states_in_warehouse__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace moveit_msgs
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _ListRobotStatesInWarehouse_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ListRobotStatesInWarehouse_Request_type_support_ids_t;

static const _ListRobotStatesInWarehouse_Request_type_support_ids_t _ListRobotStatesInWarehouse_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _ListRobotStatesInWarehouse_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ListRobotStatesInWarehouse_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ListRobotStatesInWarehouse_Request_type_support_symbol_names_t _ListRobotStatesInWarehouse_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, moveit_msgs, srv, ListRobotStatesInWarehouse_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, moveit_msgs, srv, ListRobotStatesInWarehouse_Request)),
  }
};

typedef struct _ListRobotStatesInWarehouse_Request_type_support_data_t
{
  void * data[2];
} _ListRobotStatesInWarehouse_Request_type_support_data_t;

static _ListRobotStatesInWarehouse_Request_type_support_data_t _ListRobotStatesInWarehouse_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ListRobotStatesInWarehouse_Request_message_typesupport_map = {
  2,
  "moveit_msgs",
  &_ListRobotStatesInWarehouse_Request_message_typesupport_ids.typesupport_identifier[0],
  &_ListRobotStatesInWarehouse_Request_message_typesupport_symbol_names.symbol_name[0],
  &_ListRobotStatesInWarehouse_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t ListRobotStatesInWarehouse_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ListRobotStatesInWarehouse_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &moveit_msgs__srv__ListRobotStatesInWarehouse_Request__get_type_hash,
  &moveit_msgs__srv__ListRobotStatesInWarehouse_Request__get_type_description,
  &moveit_msgs__srv__ListRobotStatesInWarehouse_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace moveit_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<moveit_msgs::srv::ListRobotStatesInWarehouse_Request>()
{
  return &::moveit_msgs::srv::rosidl_typesupport_cpp::ListRobotStatesInWarehouse_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, moveit_msgs, srv, ListRobotStatesInWarehouse_Request)() {
  return get_message_type_support_handle<moveit_msgs::srv::ListRobotStatesInWarehouse_Request>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "moveit_msgs/srv/detail/list_robot_states_in_warehouse__functions.h"
// already included above
// #include "moveit_msgs/srv/detail/list_robot_states_in_warehouse__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace moveit_msgs
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _ListRobotStatesInWarehouse_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ListRobotStatesInWarehouse_Response_type_support_ids_t;

static const _ListRobotStatesInWarehouse_Response_type_support_ids_t _ListRobotStatesInWarehouse_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _ListRobotStatesInWarehouse_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ListRobotStatesInWarehouse_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ListRobotStatesInWarehouse_Response_type_support_symbol_names_t _ListRobotStatesInWarehouse_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, moveit_msgs, srv, ListRobotStatesInWarehouse_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, moveit_msgs, srv, ListRobotStatesInWarehouse_Response)),
  }
};

typedef struct _ListRobotStatesInWarehouse_Response_type_support_data_t
{
  void * data[2];
} _ListRobotStatesInWarehouse_Response_type_support_data_t;

static _ListRobotStatesInWarehouse_Response_type_support_data_t _ListRobotStatesInWarehouse_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ListRobotStatesInWarehouse_Response_message_typesupport_map = {
  2,
  "moveit_msgs",
  &_ListRobotStatesInWarehouse_Response_message_typesupport_ids.typesupport_identifier[0],
  &_ListRobotStatesInWarehouse_Response_message_typesupport_symbol_names.symbol_name[0],
  &_ListRobotStatesInWarehouse_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t ListRobotStatesInWarehouse_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ListRobotStatesInWarehouse_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &moveit_msgs__srv__ListRobotStatesInWarehouse_Response__get_type_hash,
  &moveit_msgs__srv__ListRobotStatesInWarehouse_Response__get_type_description,
  &moveit_msgs__srv__ListRobotStatesInWarehouse_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace moveit_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<moveit_msgs::srv::ListRobotStatesInWarehouse_Response>()
{
  return &::moveit_msgs::srv::rosidl_typesupport_cpp::ListRobotStatesInWarehouse_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, moveit_msgs, srv, ListRobotStatesInWarehouse_Response)() {
  return get_message_type_support_handle<moveit_msgs::srv::ListRobotStatesInWarehouse_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "moveit_msgs/srv/detail/list_robot_states_in_warehouse__functions.h"
// already included above
// #include "moveit_msgs/srv/detail/list_robot_states_in_warehouse__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace moveit_msgs
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _ListRobotStatesInWarehouse_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ListRobotStatesInWarehouse_Event_type_support_ids_t;

static const _ListRobotStatesInWarehouse_Event_type_support_ids_t _ListRobotStatesInWarehouse_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _ListRobotStatesInWarehouse_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ListRobotStatesInWarehouse_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ListRobotStatesInWarehouse_Event_type_support_symbol_names_t _ListRobotStatesInWarehouse_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, moveit_msgs, srv, ListRobotStatesInWarehouse_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, moveit_msgs, srv, ListRobotStatesInWarehouse_Event)),
  }
};

typedef struct _ListRobotStatesInWarehouse_Event_type_support_data_t
{
  void * data[2];
} _ListRobotStatesInWarehouse_Event_type_support_data_t;

static _ListRobotStatesInWarehouse_Event_type_support_data_t _ListRobotStatesInWarehouse_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ListRobotStatesInWarehouse_Event_message_typesupport_map = {
  2,
  "moveit_msgs",
  &_ListRobotStatesInWarehouse_Event_message_typesupport_ids.typesupport_identifier[0],
  &_ListRobotStatesInWarehouse_Event_message_typesupport_symbol_names.symbol_name[0],
  &_ListRobotStatesInWarehouse_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t ListRobotStatesInWarehouse_Event_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ListRobotStatesInWarehouse_Event_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &moveit_msgs__srv__ListRobotStatesInWarehouse_Event__get_type_hash,
  &moveit_msgs__srv__ListRobotStatesInWarehouse_Event__get_type_description,
  &moveit_msgs__srv__ListRobotStatesInWarehouse_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace moveit_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<moveit_msgs::srv::ListRobotStatesInWarehouse_Event>()
{
  return &::moveit_msgs::srv::rosidl_typesupport_cpp::ListRobotStatesInWarehouse_Event_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, moveit_msgs, srv, ListRobotStatesInWarehouse_Event)() {
  return get_message_type_support_handle<moveit_msgs::srv::ListRobotStatesInWarehouse_Event>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "moveit_msgs/srv/detail/list_robot_states_in_warehouse__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace moveit_msgs
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _ListRobotStatesInWarehouse_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ListRobotStatesInWarehouse_type_support_ids_t;

static const _ListRobotStatesInWarehouse_type_support_ids_t _ListRobotStatesInWarehouse_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _ListRobotStatesInWarehouse_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ListRobotStatesInWarehouse_type_support_symbol_names_t;
#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ListRobotStatesInWarehouse_type_support_symbol_names_t _ListRobotStatesInWarehouse_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, moveit_msgs, srv, ListRobotStatesInWarehouse)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, moveit_msgs, srv, ListRobotStatesInWarehouse)),
  }
};

typedef struct _ListRobotStatesInWarehouse_type_support_data_t
{
  void * data[2];
} _ListRobotStatesInWarehouse_type_support_data_t;

static _ListRobotStatesInWarehouse_type_support_data_t _ListRobotStatesInWarehouse_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ListRobotStatesInWarehouse_service_typesupport_map = {
  2,
  "moveit_msgs",
  &_ListRobotStatesInWarehouse_service_typesupport_ids.typesupport_identifier[0],
  &_ListRobotStatesInWarehouse_service_typesupport_symbol_names.symbol_name[0],
  &_ListRobotStatesInWarehouse_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t ListRobotStatesInWarehouse_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ListRobotStatesInWarehouse_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
  ::rosidl_typesupport_cpp::get_message_type_support_handle<moveit_msgs::srv::ListRobotStatesInWarehouse_Request>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<moveit_msgs::srv::ListRobotStatesInWarehouse_Response>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<moveit_msgs::srv::ListRobotStatesInWarehouse_Event>(),
  &::rosidl_typesupport_cpp::service_create_event_message<moveit_msgs::srv::ListRobotStatesInWarehouse>,
  &::rosidl_typesupport_cpp::service_destroy_event_message<moveit_msgs::srv::ListRobotStatesInWarehouse>,
  &moveit_msgs__srv__ListRobotStatesInWarehouse__get_type_hash,
  &moveit_msgs__srv__ListRobotStatesInWarehouse__get_type_description,
  &moveit_msgs__srv__ListRobotStatesInWarehouse__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace moveit_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<moveit_msgs::srv::ListRobotStatesInWarehouse>()
{
  return &::moveit_msgs::srv::rosidl_typesupport_cpp::ListRobotStatesInWarehouse_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, moveit_msgs, srv, ListRobotStatesInWarehouse)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<moveit_msgs::srv::ListRobotStatesInWarehouse>();
}

#ifdef __cplusplus
}
#endif
