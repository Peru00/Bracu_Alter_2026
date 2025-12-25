// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from moveit_msgs:msg/PipelineState.idl
// generated code does not contain a copyright notice
#include "moveit_msgs/msg/detail/pipeline_state__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "moveit_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "moveit_msgs/msg/detail/pipeline_state__struct.h"
#include "moveit_msgs/msg/detail/pipeline_state__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "moveit_msgs/msg/detail/motion_plan_request__functions.h"  // request
#include "moveit_msgs/msg/detail/motion_plan_response__functions.h"  // response
#include "rosidl_runtime_c/string.h"  // pipeline_stage
#include "rosidl_runtime_c/string_functions.h"  // pipeline_stage

// forward declare type support functions

bool cdr_serialize_moveit_msgs__msg__MotionPlanRequest(
  const moveit_msgs__msg__MotionPlanRequest * ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool cdr_deserialize_moveit_msgs__msg__MotionPlanRequest(
  eprosima::fastcdr::Cdr & cdr,
  moveit_msgs__msg__MotionPlanRequest * ros_message);

size_t get_serialized_size_moveit_msgs__msg__MotionPlanRequest(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_moveit_msgs__msg__MotionPlanRequest(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool cdr_serialize_key_moveit_msgs__msg__MotionPlanRequest(
  const moveit_msgs__msg__MotionPlanRequest * ros_message,
  eprosima::fastcdr::Cdr & cdr);

size_t get_serialized_size_key_moveit_msgs__msg__MotionPlanRequest(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_key_moveit_msgs__msg__MotionPlanRequest(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, moveit_msgs, msg, MotionPlanRequest)();

bool cdr_serialize_moveit_msgs__msg__MotionPlanResponse(
  const moveit_msgs__msg__MotionPlanResponse * ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool cdr_deserialize_moveit_msgs__msg__MotionPlanResponse(
  eprosima::fastcdr::Cdr & cdr,
  moveit_msgs__msg__MotionPlanResponse * ros_message);

size_t get_serialized_size_moveit_msgs__msg__MotionPlanResponse(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_moveit_msgs__msg__MotionPlanResponse(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool cdr_serialize_key_moveit_msgs__msg__MotionPlanResponse(
  const moveit_msgs__msg__MotionPlanResponse * ros_message,
  eprosima::fastcdr::Cdr & cdr);

size_t get_serialized_size_key_moveit_msgs__msg__MotionPlanResponse(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_key_moveit_msgs__msg__MotionPlanResponse(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, moveit_msgs, msg, MotionPlanResponse)();


using _PipelineState__ros_msg_type = moveit_msgs__msg__PipelineState;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_moveit_msgs
bool cdr_serialize_moveit_msgs__msg__PipelineState(
  const moveit_msgs__msg__PipelineState * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: request
  {
    cdr_serialize_moveit_msgs__msg__MotionPlanRequest(
      &ros_message->request, cdr);
  }

  // Field name: response
  {
    cdr_serialize_moveit_msgs__msg__MotionPlanResponse(
      &ros_message->response, cdr);
  }

  // Field name: pipeline_stage
  {
    const rosidl_runtime_c__String * str = &ros_message->pipeline_stage;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_moveit_msgs
bool cdr_deserialize_moveit_msgs__msg__PipelineState(
  eprosima::fastcdr::Cdr & cdr,
  moveit_msgs__msg__PipelineState * ros_message)
{
  // Field name: request
  {
    cdr_deserialize_moveit_msgs__msg__MotionPlanRequest(cdr, &ros_message->request);
  }

  // Field name: response
  {
    cdr_deserialize_moveit_msgs__msg__MotionPlanResponse(cdr, &ros_message->response);
  }

  // Field name: pipeline_stage
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->pipeline_stage.data) {
      rosidl_runtime_c__String__init(&ros_message->pipeline_stage);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->pipeline_stage,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'pipeline_stage'\n");
      return false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_moveit_msgs
size_t get_serialized_size_moveit_msgs__msg__PipelineState(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _PipelineState__ros_msg_type * ros_message = static_cast<const _PipelineState__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: request
  current_alignment += get_serialized_size_moveit_msgs__msg__MotionPlanRequest(
    &(ros_message->request), current_alignment);

  // Field name: response
  current_alignment += get_serialized_size_moveit_msgs__msg__MotionPlanResponse(
    &(ros_message->response), current_alignment);

  // Field name: pipeline_stage
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->pipeline_stage.size + 1);

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_moveit_msgs
size_t max_serialized_size_moveit_msgs__msg__PipelineState(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // Field name: request
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_moveit_msgs__msg__MotionPlanRequest(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: response
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_moveit_msgs__msg__MotionPlanResponse(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: pipeline_stage
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }


  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = moveit_msgs__msg__PipelineState;
    is_plain =
      (
      offsetof(DataType, pipeline_stage) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_moveit_msgs
bool cdr_serialize_key_moveit_msgs__msg__PipelineState(
  const moveit_msgs__msg__PipelineState * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: request
  {
    cdr_serialize_key_moveit_msgs__msg__MotionPlanRequest(
      &ros_message->request, cdr);
  }

  // Field name: response
  {
    cdr_serialize_key_moveit_msgs__msg__MotionPlanResponse(
      &ros_message->response, cdr);
  }

  // Field name: pipeline_stage
  {
    const rosidl_runtime_c__String * str = &ros_message->pipeline_stage;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_moveit_msgs
size_t get_serialized_size_key_moveit_msgs__msg__PipelineState(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _PipelineState__ros_msg_type * ros_message = static_cast<const _PipelineState__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: request
  current_alignment += get_serialized_size_key_moveit_msgs__msg__MotionPlanRequest(
    &(ros_message->request), current_alignment);

  // Field name: response
  current_alignment += get_serialized_size_key_moveit_msgs__msg__MotionPlanResponse(
    &(ros_message->response), current_alignment);

  // Field name: pipeline_stage
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->pipeline_stage.size + 1);

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_moveit_msgs
size_t max_serialized_size_key_moveit_msgs__msg__PipelineState(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;
  // Field name: request
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_moveit_msgs__msg__MotionPlanRequest(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: response
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_moveit_msgs__msg__MotionPlanResponse(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: pipeline_stage
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = moveit_msgs__msg__PipelineState;
    is_plain =
      (
      offsetof(DataType, pipeline_stage) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _PipelineState__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const moveit_msgs__msg__PipelineState * ros_message = static_cast<const moveit_msgs__msg__PipelineState *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_moveit_msgs__msg__PipelineState(ros_message, cdr);
}

static bool _PipelineState__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  moveit_msgs__msg__PipelineState * ros_message = static_cast<moveit_msgs__msg__PipelineState *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_moveit_msgs__msg__PipelineState(cdr, ros_message);
}

static uint32_t _PipelineState__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_moveit_msgs__msg__PipelineState(
      untyped_ros_message, 0));
}

static size_t _PipelineState__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_moveit_msgs__msg__PipelineState(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_PipelineState = {
  "moveit_msgs::msg",
  "PipelineState",
  _PipelineState__cdr_serialize,
  _PipelineState__cdr_deserialize,
  _PipelineState__get_serialized_size,
  _PipelineState__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _PipelineState__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_PipelineState,
  get_message_typesupport_handle_function,
  &moveit_msgs__msg__PipelineState__get_type_hash,
  &moveit_msgs__msg__PipelineState__get_type_description,
  &moveit_msgs__msg__PipelineState__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, moveit_msgs, msg, PipelineState)() {
  return &_PipelineState__type_support;
}

#if defined(__cplusplus)
}
#endif
