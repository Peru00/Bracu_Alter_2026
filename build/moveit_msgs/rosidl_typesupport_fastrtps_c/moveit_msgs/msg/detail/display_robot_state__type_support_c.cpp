// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from moveit_msgs:msg/DisplayRobotState.idl
// generated code does not contain a copyright notice
#include "moveit_msgs/msg/detail/display_robot_state__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "moveit_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "moveit_msgs/msg/detail/display_robot_state__struct.h"
#include "moveit_msgs/msg/detail/display_robot_state__functions.h"
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

#include "moveit_msgs/msg/detail/object_color__functions.h"  // highlight_links
#include "moveit_msgs/msg/detail/robot_state__functions.h"  // state

// forward declare type support functions

bool cdr_serialize_moveit_msgs__msg__ObjectColor(
  const moveit_msgs__msg__ObjectColor * ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool cdr_deserialize_moveit_msgs__msg__ObjectColor(
  eprosima::fastcdr::Cdr & cdr,
  moveit_msgs__msg__ObjectColor * ros_message);

size_t get_serialized_size_moveit_msgs__msg__ObjectColor(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_moveit_msgs__msg__ObjectColor(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool cdr_serialize_key_moveit_msgs__msg__ObjectColor(
  const moveit_msgs__msg__ObjectColor * ros_message,
  eprosima::fastcdr::Cdr & cdr);

size_t get_serialized_size_key_moveit_msgs__msg__ObjectColor(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_key_moveit_msgs__msg__ObjectColor(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, moveit_msgs, msg, ObjectColor)();

bool cdr_serialize_moveit_msgs__msg__RobotState(
  const moveit_msgs__msg__RobotState * ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool cdr_deserialize_moveit_msgs__msg__RobotState(
  eprosima::fastcdr::Cdr & cdr,
  moveit_msgs__msg__RobotState * ros_message);

size_t get_serialized_size_moveit_msgs__msg__RobotState(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_moveit_msgs__msg__RobotState(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool cdr_serialize_key_moveit_msgs__msg__RobotState(
  const moveit_msgs__msg__RobotState * ros_message,
  eprosima::fastcdr::Cdr & cdr);

size_t get_serialized_size_key_moveit_msgs__msg__RobotState(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_key_moveit_msgs__msg__RobotState(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, moveit_msgs, msg, RobotState)();


using _DisplayRobotState__ros_msg_type = moveit_msgs__msg__DisplayRobotState;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_moveit_msgs
bool cdr_serialize_moveit_msgs__msg__DisplayRobotState(
  const moveit_msgs__msg__DisplayRobotState * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: state
  {
    cdr_serialize_moveit_msgs__msg__RobotState(
      &ros_message->state, cdr);
  }

  // Field name: highlight_links
  {
    size_t size = ros_message->highlight_links.size;
    auto array_ptr = ros_message->highlight_links.data;
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      cdr_serialize_moveit_msgs__msg__ObjectColor(
        &array_ptr[i], cdr);
    }
  }

  // Field name: hide
  {
    cdr << (ros_message->hide ? true : false);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_moveit_msgs
bool cdr_deserialize_moveit_msgs__msg__DisplayRobotState(
  eprosima::fastcdr::Cdr & cdr,
  moveit_msgs__msg__DisplayRobotState * ros_message)
{
  // Field name: state
  {
    cdr_deserialize_moveit_msgs__msg__RobotState(cdr, &ros_message->state);
  }

  // Field name: highlight_links
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);

    // Check there are at least 'size' remaining bytes in the CDR stream before resizing
    auto old_state = cdr.get_state();
    bool correct_size = cdr.jump(size);
    cdr.set_state(old_state);
    if (!correct_size) {
      fprintf(stderr, "sequence size exceeds remaining buffer\n");
      return false;
    }

    if (ros_message->highlight_links.data) {
      moveit_msgs__msg__ObjectColor__Sequence__fini(&ros_message->highlight_links);
    }
    if (!moveit_msgs__msg__ObjectColor__Sequence__init(&ros_message->highlight_links, size)) {
      fprintf(stderr, "failed to create array for field 'highlight_links'");
      return false;
    }
    auto array_ptr = ros_message->highlight_links.data;
    for (size_t i = 0; i < size; ++i) {
      cdr_deserialize_moveit_msgs__msg__ObjectColor(cdr, &array_ptr[i]);
    }
  }

  // Field name: hide
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->hide = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_moveit_msgs
size_t get_serialized_size_moveit_msgs__msg__DisplayRobotState(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _DisplayRobotState__ros_msg_type * ros_message = static_cast<const _DisplayRobotState__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: state
  current_alignment += get_serialized_size_moveit_msgs__msg__RobotState(
    &(ros_message->state), current_alignment);

  // Field name: highlight_links
  {
    size_t array_size = ros_message->highlight_links.size;
    auto array_ptr = ros_message->highlight_links.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_moveit_msgs__msg__ObjectColor(
        &array_ptr[index], current_alignment);
    }
  }

  // Field name: hide
  {
    size_t item_size = sizeof(ros_message->hide);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_moveit_msgs
size_t max_serialized_size_moveit_msgs__msg__DisplayRobotState(
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

  // Field name: state
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_moveit_msgs__msg__RobotState(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: highlight_links
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_moveit_msgs__msg__ObjectColor(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: hide
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }


  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = moveit_msgs__msg__DisplayRobotState;
    is_plain =
      (
      offsetof(DataType, hide) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_moveit_msgs
bool cdr_serialize_key_moveit_msgs__msg__DisplayRobotState(
  const moveit_msgs__msg__DisplayRobotState * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: state
  {
    cdr_serialize_key_moveit_msgs__msg__RobotState(
      &ros_message->state, cdr);
  }

  // Field name: highlight_links
  {
    size_t size = ros_message->highlight_links.size;
    auto array_ptr = ros_message->highlight_links.data;
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      cdr_serialize_key_moveit_msgs__msg__ObjectColor(
        &array_ptr[i], cdr);
    }
  }

  // Field name: hide
  {
    cdr << (ros_message->hide ? true : false);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_moveit_msgs
size_t get_serialized_size_key_moveit_msgs__msg__DisplayRobotState(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _DisplayRobotState__ros_msg_type * ros_message = static_cast<const _DisplayRobotState__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: state
  current_alignment += get_serialized_size_key_moveit_msgs__msg__RobotState(
    &(ros_message->state), current_alignment);

  // Field name: highlight_links
  {
    size_t array_size = ros_message->highlight_links.size;
    auto array_ptr = ros_message->highlight_links.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_key_moveit_msgs__msg__ObjectColor(
        &array_ptr[index], current_alignment);
    }
  }

  // Field name: hide
  {
    size_t item_size = sizeof(ros_message->hide);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_moveit_msgs
size_t max_serialized_size_key_moveit_msgs__msg__DisplayRobotState(
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
  // Field name: state
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_moveit_msgs__msg__RobotState(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: highlight_links
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_moveit_msgs__msg__ObjectColor(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: hide
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = moveit_msgs__msg__DisplayRobotState;
    is_plain =
      (
      offsetof(DataType, hide) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _DisplayRobotState__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const moveit_msgs__msg__DisplayRobotState * ros_message = static_cast<const moveit_msgs__msg__DisplayRobotState *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_moveit_msgs__msg__DisplayRobotState(ros_message, cdr);
}

static bool _DisplayRobotState__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  moveit_msgs__msg__DisplayRobotState * ros_message = static_cast<moveit_msgs__msg__DisplayRobotState *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_moveit_msgs__msg__DisplayRobotState(cdr, ros_message);
}

static uint32_t _DisplayRobotState__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_moveit_msgs__msg__DisplayRobotState(
      untyped_ros_message, 0));
}

static size_t _DisplayRobotState__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_moveit_msgs__msg__DisplayRobotState(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_DisplayRobotState = {
  "moveit_msgs::msg",
  "DisplayRobotState",
  _DisplayRobotState__cdr_serialize,
  _DisplayRobotState__cdr_deserialize,
  _DisplayRobotState__get_serialized_size,
  _DisplayRobotState__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _DisplayRobotState__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_DisplayRobotState,
  get_message_typesupport_handle_function,
  &moveit_msgs__msg__DisplayRobotState__get_type_hash,
  &moveit_msgs__msg__DisplayRobotState__get_type_description,
  &moveit_msgs__msg__DisplayRobotState__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, moveit_msgs, msg, DisplayRobotState)() {
  return &_DisplayRobotState__type_support;
}

#if defined(__cplusplus)
}
#endif
