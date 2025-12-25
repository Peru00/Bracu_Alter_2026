// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from moveit_msgs:srv/SaveGeometryToFile.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "moveit_msgs/srv/detail/save_geometry_to_file__functions.h"
#include "moveit_msgs/srv/detail/save_geometry_to_file__struct.hpp"
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

typedef struct _SaveGeometryToFile_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _SaveGeometryToFile_Request_type_support_ids_t;

static const _SaveGeometryToFile_Request_type_support_ids_t _SaveGeometryToFile_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _SaveGeometryToFile_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _SaveGeometryToFile_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _SaveGeometryToFile_Request_type_support_symbol_names_t _SaveGeometryToFile_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, moveit_msgs, srv, SaveGeometryToFile_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, moveit_msgs, srv, SaveGeometryToFile_Request)),
  }
};

typedef struct _SaveGeometryToFile_Request_type_support_data_t
{
  void * data[2];
} _SaveGeometryToFile_Request_type_support_data_t;

static _SaveGeometryToFile_Request_type_support_data_t _SaveGeometryToFile_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _SaveGeometryToFile_Request_message_typesupport_map = {
  2,
  "moveit_msgs",
  &_SaveGeometryToFile_Request_message_typesupport_ids.typesupport_identifier[0],
  &_SaveGeometryToFile_Request_message_typesupport_symbol_names.symbol_name[0],
  &_SaveGeometryToFile_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t SaveGeometryToFile_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_SaveGeometryToFile_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &moveit_msgs__srv__SaveGeometryToFile_Request__get_type_hash,
  &moveit_msgs__srv__SaveGeometryToFile_Request__get_type_description,
  &moveit_msgs__srv__SaveGeometryToFile_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace moveit_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<moveit_msgs::srv::SaveGeometryToFile_Request>()
{
  return &::moveit_msgs::srv::rosidl_typesupport_cpp::SaveGeometryToFile_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, moveit_msgs, srv, SaveGeometryToFile_Request)() {
  return get_message_type_support_handle<moveit_msgs::srv::SaveGeometryToFile_Request>();
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
// #include "moveit_msgs/srv/detail/save_geometry_to_file__functions.h"
// already included above
// #include "moveit_msgs/srv/detail/save_geometry_to_file__struct.hpp"
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

typedef struct _SaveGeometryToFile_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _SaveGeometryToFile_Response_type_support_ids_t;

static const _SaveGeometryToFile_Response_type_support_ids_t _SaveGeometryToFile_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _SaveGeometryToFile_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _SaveGeometryToFile_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _SaveGeometryToFile_Response_type_support_symbol_names_t _SaveGeometryToFile_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, moveit_msgs, srv, SaveGeometryToFile_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, moveit_msgs, srv, SaveGeometryToFile_Response)),
  }
};

typedef struct _SaveGeometryToFile_Response_type_support_data_t
{
  void * data[2];
} _SaveGeometryToFile_Response_type_support_data_t;

static _SaveGeometryToFile_Response_type_support_data_t _SaveGeometryToFile_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _SaveGeometryToFile_Response_message_typesupport_map = {
  2,
  "moveit_msgs",
  &_SaveGeometryToFile_Response_message_typesupport_ids.typesupport_identifier[0],
  &_SaveGeometryToFile_Response_message_typesupport_symbol_names.symbol_name[0],
  &_SaveGeometryToFile_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t SaveGeometryToFile_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_SaveGeometryToFile_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &moveit_msgs__srv__SaveGeometryToFile_Response__get_type_hash,
  &moveit_msgs__srv__SaveGeometryToFile_Response__get_type_description,
  &moveit_msgs__srv__SaveGeometryToFile_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace moveit_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<moveit_msgs::srv::SaveGeometryToFile_Response>()
{
  return &::moveit_msgs::srv::rosidl_typesupport_cpp::SaveGeometryToFile_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, moveit_msgs, srv, SaveGeometryToFile_Response)() {
  return get_message_type_support_handle<moveit_msgs::srv::SaveGeometryToFile_Response>();
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
// #include "moveit_msgs/srv/detail/save_geometry_to_file__functions.h"
// already included above
// #include "moveit_msgs/srv/detail/save_geometry_to_file__struct.hpp"
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

typedef struct _SaveGeometryToFile_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _SaveGeometryToFile_Event_type_support_ids_t;

static const _SaveGeometryToFile_Event_type_support_ids_t _SaveGeometryToFile_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _SaveGeometryToFile_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _SaveGeometryToFile_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _SaveGeometryToFile_Event_type_support_symbol_names_t _SaveGeometryToFile_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, moveit_msgs, srv, SaveGeometryToFile_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, moveit_msgs, srv, SaveGeometryToFile_Event)),
  }
};

typedef struct _SaveGeometryToFile_Event_type_support_data_t
{
  void * data[2];
} _SaveGeometryToFile_Event_type_support_data_t;

static _SaveGeometryToFile_Event_type_support_data_t _SaveGeometryToFile_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _SaveGeometryToFile_Event_message_typesupport_map = {
  2,
  "moveit_msgs",
  &_SaveGeometryToFile_Event_message_typesupport_ids.typesupport_identifier[0],
  &_SaveGeometryToFile_Event_message_typesupport_symbol_names.symbol_name[0],
  &_SaveGeometryToFile_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t SaveGeometryToFile_Event_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_SaveGeometryToFile_Event_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &moveit_msgs__srv__SaveGeometryToFile_Event__get_type_hash,
  &moveit_msgs__srv__SaveGeometryToFile_Event__get_type_description,
  &moveit_msgs__srv__SaveGeometryToFile_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace moveit_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<moveit_msgs::srv::SaveGeometryToFile_Event>()
{
  return &::moveit_msgs::srv::rosidl_typesupport_cpp::SaveGeometryToFile_Event_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, moveit_msgs, srv, SaveGeometryToFile_Event)() {
  return get_message_type_support_handle<moveit_msgs::srv::SaveGeometryToFile_Event>();
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
// #include "moveit_msgs/srv/detail/save_geometry_to_file__struct.hpp"
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

typedef struct _SaveGeometryToFile_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _SaveGeometryToFile_type_support_ids_t;

static const _SaveGeometryToFile_type_support_ids_t _SaveGeometryToFile_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _SaveGeometryToFile_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _SaveGeometryToFile_type_support_symbol_names_t;
#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _SaveGeometryToFile_type_support_symbol_names_t _SaveGeometryToFile_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, moveit_msgs, srv, SaveGeometryToFile)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, moveit_msgs, srv, SaveGeometryToFile)),
  }
};

typedef struct _SaveGeometryToFile_type_support_data_t
{
  void * data[2];
} _SaveGeometryToFile_type_support_data_t;

static _SaveGeometryToFile_type_support_data_t _SaveGeometryToFile_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _SaveGeometryToFile_service_typesupport_map = {
  2,
  "moveit_msgs",
  &_SaveGeometryToFile_service_typesupport_ids.typesupport_identifier[0],
  &_SaveGeometryToFile_service_typesupport_symbol_names.symbol_name[0],
  &_SaveGeometryToFile_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t SaveGeometryToFile_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_SaveGeometryToFile_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
  ::rosidl_typesupport_cpp::get_message_type_support_handle<moveit_msgs::srv::SaveGeometryToFile_Request>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<moveit_msgs::srv::SaveGeometryToFile_Response>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<moveit_msgs::srv::SaveGeometryToFile_Event>(),
  &::rosidl_typesupport_cpp::service_create_event_message<moveit_msgs::srv::SaveGeometryToFile>,
  &::rosidl_typesupport_cpp::service_destroy_event_message<moveit_msgs::srv::SaveGeometryToFile>,
  &moveit_msgs__srv__SaveGeometryToFile__get_type_hash,
  &moveit_msgs__srv__SaveGeometryToFile__get_type_description,
  &moveit_msgs__srv__SaveGeometryToFile__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace moveit_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<moveit_msgs::srv::SaveGeometryToFile>()
{
  return &::moveit_msgs::srv::rosidl_typesupport_cpp::SaveGeometryToFile_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, moveit_msgs, srv, SaveGeometryToFile)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<moveit_msgs::srv::SaveGeometryToFile>();
}

#ifdef __cplusplus
}
#endif
