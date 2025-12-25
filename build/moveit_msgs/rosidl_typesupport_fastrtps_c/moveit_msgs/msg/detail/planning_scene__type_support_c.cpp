// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from moveit_msgs:msg/PlanningScene.idl
// generated code does not contain a copyright notice
#include "moveit_msgs/msg/detail/planning_scene__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "moveit_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "moveit_msgs/msg/detail/planning_scene__struct.h"
#include "moveit_msgs/msg/detail/planning_scene__functions.h"
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

#include "geometry_msgs/msg/detail/transform_stamped__functions.h"  // fixed_frame_transforms
#include "moveit_msgs/msg/detail/allowed_collision_matrix__functions.h"  // allowed_collision_matrix
#include "moveit_msgs/msg/detail/link_padding__functions.h"  // link_padding
#include "moveit_msgs/msg/detail/link_scale__functions.h"  // link_scale
#include "moveit_msgs/msg/detail/object_color__functions.h"  // object_colors
#include "moveit_msgs/msg/detail/planning_scene_world__functions.h"  // world
#include "moveit_msgs/msg/detail/robot_state__functions.h"  // robot_state
#include "rosidl_runtime_c/string.h"  // name, robot_model_name
#include "rosidl_runtime_c/string_functions.h"  // name, robot_model_name

// forward declare type support functions

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_moveit_msgs
bool cdr_serialize_geometry_msgs__msg__TransformStamped(
  const geometry_msgs__msg__TransformStamped * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_moveit_msgs
bool cdr_deserialize_geometry_msgs__msg__TransformStamped(
  eprosima::fastcdr::Cdr & cdr,
  geometry_msgs__msg__TransformStamped * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_moveit_msgs
size_t get_serialized_size_geometry_msgs__msg__TransformStamped(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_moveit_msgs
size_t max_serialized_size_geometry_msgs__msg__TransformStamped(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_moveit_msgs
bool cdr_serialize_key_geometry_msgs__msg__TransformStamped(
  const geometry_msgs__msg__TransformStamped * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_moveit_msgs
size_t get_serialized_size_key_geometry_msgs__msg__TransformStamped(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_moveit_msgs
size_t max_serialized_size_key_geometry_msgs__msg__TransformStamped(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_moveit_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, geometry_msgs, msg, TransformStamped)();

bool cdr_serialize_moveit_msgs__msg__AllowedCollisionMatrix(
  const moveit_msgs__msg__AllowedCollisionMatrix * ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool cdr_deserialize_moveit_msgs__msg__AllowedCollisionMatrix(
  eprosima::fastcdr::Cdr & cdr,
  moveit_msgs__msg__AllowedCollisionMatrix * ros_message);

size_t get_serialized_size_moveit_msgs__msg__AllowedCollisionMatrix(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_moveit_msgs__msg__AllowedCollisionMatrix(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool cdr_serialize_key_moveit_msgs__msg__AllowedCollisionMatrix(
  const moveit_msgs__msg__AllowedCollisionMatrix * ros_message,
  eprosima::fastcdr::Cdr & cdr);

size_t get_serialized_size_key_moveit_msgs__msg__AllowedCollisionMatrix(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_key_moveit_msgs__msg__AllowedCollisionMatrix(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, moveit_msgs, msg, AllowedCollisionMatrix)();

bool cdr_serialize_moveit_msgs__msg__LinkPadding(
  const moveit_msgs__msg__LinkPadding * ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool cdr_deserialize_moveit_msgs__msg__LinkPadding(
  eprosima::fastcdr::Cdr & cdr,
  moveit_msgs__msg__LinkPadding * ros_message);

size_t get_serialized_size_moveit_msgs__msg__LinkPadding(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_moveit_msgs__msg__LinkPadding(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool cdr_serialize_key_moveit_msgs__msg__LinkPadding(
  const moveit_msgs__msg__LinkPadding * ros_message,
  eprosima::fastcdr::Cdr & cdr);

size_t get_serialized_size_key_moveit_msgs__msg__LinkPadding(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_key_moveit_msgs__msg__LinkPadding(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, moveit_msgs, msg, LinkPadding)();

bool cdr_serialize_moveit_msgs__msg__LinkScale(
  const moveit_msgs__msg__LinkScale * ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool cdr_deserialize_moveit_msgs__msg__LinkScale(
  eprosima::fastcdr::Cdr & cdr,
  moveit_msgs__msg__LinkScale * ros_message);

size_t get_serialized_size_moveit_msgs__msg__LinkScale(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_moveit_msgs__msg__LinkScale(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool cdr_serialize_key_moveit_msgs__msg__LinkScale(
  const moveit_msgs__msg__LinkScale * ros_message,
  eprosima::fastcdr::Cdr & cdr);

size_t get_serialized_size_key_moveit_msgs__msg__LinkScale(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_key_moveit_msgs__msg__LinkScale(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, moveit_msgs, msg, LinkScale)();

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

bool cdr_serialize_moveit_msgs__msg__PlanningSceneWorld(
  const moveit_msgs__msg__PlanningSceneWorld * ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool cdr_deserialize_moveit_msgs__msg__PlanningSceneWorld(
  eprosima::fastcdr::Cdr & cdr,
  moveit_msgs__msg__PlanningSceneWorld * ros_message);

size_t get_serialized_size_moveit_msgs__msg__PlanningSceneWorld(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_moveit_msgs__msg__PlanningSceneWorld(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool cdr_serialize_key_moveit_msgs__msg__PlanningSceneWorld(
  const moveit_msgs__msg__PlanningSceneWorld * ros_message,
  eprosima::fastcdr::Cdr & cdr);

size_t get_serialized_size_key_moveit_msgs__msg__PlanningSceneWorld(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_key_moveit_msgs__msg__PlanningSceneWorld(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, moveit_msgs, msg, PlanningSceneWorld)();

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


using _PlanningScene__ros_msg_type = moveit_msgs__msg__PlanningScene;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_moveit_msgs
bool cdr_serialize_moveit_msgs__msg__PlanningScene(
  const moveit_msgs__msg__PlanningScene * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: name
  {
    const rosidl_runtime_c__String * str = &ros_message->name;
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

  // Field name: robot_state
  {
    cdr_serialize_moveit_msgs__msg__RobotState(
      &ros_message->robot_state, cdr);
  }

  // Field name: robot_model_name
  {
    const rosidl_runtime_c__String * str = &ros_message->robot_model_name;
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

  // Field name: fixed_frame_transforms
  {
    size_t size = ros_message->fixed_frame_transforms.size;
    auto array_ptr = ros_message->fixed_frame_transforms.data;
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      cdr_serialize_geometry_msgs__msg__TransformStamped(
        &array_ptr[i], cdr);
    }
  }

  // Field name: allowed_collision_matrix
  {
    cdr_serialize_moveit_msgs__msg__AllowedCollisionMatrix(
      &ros_message->allowed_collision_matrix, cdr);
  }

  // Field name: link_padding
  {
    size_t size = ros_message->link_padding.size;
    auto array_ptr = ros_message->link_padding.data;
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      cdr_serialize_moveit_msgs__msg__LinkPadding(
        &array_ptr[i], cdr);
    }
  }

  // Field name: link_scale
  {
    size_t size = ros_message->link_scale.size;
    auto array_ptr = ros_message->link_scale.data;
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      cdr_serialize_moveit_msgs__msg__LinkScale(
        &array_ptr[i], cdr);
    }
  }

  // Field name: object_colors
  {
    size_t size = ros_message->object_colors.size;
    auto array_ptr = ros_message->object_colors.data;
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      cdr_serialize_moveit_msgs__msg__ObjectColor(
        &array_ptr[i], cdr);
    }
  }

  // Field name: world
  {
    cdr_serialize_moveit_msgs__msg__PlanningSceneWorld(
      &ros_message->world, cdr);
  }

  // Field name: is_diff
  {
    cdr << (ros_message->is_diff ? true : false);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_moveit_msgs
bool cdr_deserialize_moveit_msgs__msg__PlanningScene(
  eprosima::fastcdr::Cdr & cdr,
  moveit_msgs__msg__PlanningScene * ros_message)
{
  // Field name: name
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->name.data) {
      rosidl_runtime_c__String__init(&ros_message->name);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->name,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'name'\n");
      return false;
    }
  }

  // Field name: robot_state
  {
    cdr_deserialize_moveit_msgs__msg__RobotState(cdr, &ros_message->robot_state);
  }

  // Field name: robot_model_name
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->robot_model_name.data) {
      rosidl_runtime_c__String__init(&ros_message->robot_model_name);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->robot_model_name,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'robot_model_name'\n");
      return false;
    }
  }

  // Field name: fixed_frame_transforms
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

    if (ros_message->fixed_frame_transforms.data) {
      geometry_msgs__msg__TransformStamped__Sequence__fini(&ros_message->fixed_frame_transforms);
    }
    if (!geometry_msgs__msg__TransformStamped__Sequence__init(&ros_message->fixed_frame_transforms, size)) {
      fprintf(stderr, "failed to create array for field 'fixed_frame_transforms'");
      return false;
    }
    auto array_ptr = ros_message->fixed_frame_transforms.data;
    for (size_t i = 0; i < size; ++i) {
      cdr_deserialize_geometry_msgs__msg__TransformStamped(cdr, &array_ptr[i]);
    }
  }

  // Field name: allowed_collision_matrix
  {
    cdr_deserialize_moveit_msgs__msg__AllowedCollisionMatrix(cdr, &ros_message->allowed_collision_matrix);
  }

  // Field name: link_padding
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

    if (ros_message->link_padding.data) {
      moveit_msgs__msg__LinkPadding__Sequence__fini(&ros_message->link_padding);
    }
    if (!moveit_msgs__msg__LinkPadding__Sequence__init(&ros_message->link_padding, size)) {
      fprintf(stderr, "failed to create array for field 'link_padding'");
      return false;
    }
    auto array_ptr = ros_message->link_padding.data;
    for (size_t i = 0; i < size; ++i) {
      cdr_deserialize_moveit_msgs__msg__LinkPadding(cdr, &array_ptr[i]);
    }
  }

  // Field name: link_scale
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

    if (ros_message->link_scale.data) {
      moveit_msgs__msg__LinkScale__Sequence__fini(&ros_message->link_scale);
    }
    if (!moveit_msgs__msg__LinkScale__Sequence__init(&ros_message->link_scale, size)) {
      fprintf(stderr, "failed to create array for field 'link_scale'");
      return false;
    }
    auto array_ptr = ros_message->link_scale.data;
    for (size_t i = 0; i < size; ++i) {
      cdr_deserialize_moveit_msgs__msg__LinkScale(cdr, &array_ptr[i]);
    }
  }

  // Field name: object_colors
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

    if (ros_message->object_colors.data) {
      moveit_msgs__msg__ObjectColor__Sequence__fini(&ros_message->object_colors);
    }
    if (!moveit_msgs__msg__ObjectColor__Sequence__init(&ros_message->object_colors, size)) {
      fprintf(stderr, "failed to create array for field 'object_colors'");
      return false;
    }
    auto array_ptr = ros_message->object_colors.data;
    for (size_t i = 0; i < size; ++i) {
      cdr_deserialize_moveit_msgs__msg__ObjectColor(cdr, &array_ptr[i]);
    }
  }

  // Field name: world
  {
    cdr_deserialize_moveit_msgs__msg__PlanningSceneWorld(cdr, &ros_message->world);
  }

  // Field name: is_diff
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->is_diff = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_moveit_msgs
size_t get_serialized_size_moveit_msgs__msg__PlanningScene(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _PlanningScene__ros_msg_type * ros_message = static_cast<const _PlanningScene__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: name
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->name.size + 1);

  // Field name: robot_state
  current_alignment += get_serialized_size_moveit_msgs__msg__RobotState(
    &(ros_message->robot_state), current_alignment);

  // Field name: robot_model_name
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->robot_model_name.size + 1);

  // Field name: fixed_frame_transforms
  {
    size_t array_size = ros_message->fixed_frame_transforms.size;
    auto array_ptr = ros_message->fixed_frame_transforms.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_geometry_msgs__msg__TransformStamped(
        &array_ptr[index], current_alignment);
    }
  }

  // Field name: allowed_collision_matrix
  current_alignment += get_serialized_size_moveit_msgs__msg__AllowedCollisionMatrix(
    &(ros_message->allowed_collision_matrix), current_alignment);

  // Field name: link_padding
  {
    size_t array_size = ros_message->link_padding.size;
    auto array_ptr = ros_message->link_padding.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_moveit_msgs__msg__LinkPadding(
        &array_ptr[index], current_alignment);
    }
  }

  // Field name: link_scale
  {
    size_t array_size = ros_message->link_scale.size;
    auto array_ptr = ros_message->link_scale.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_moveit_msgs__msg__LinkScale(
        &array_ptr[index], current_alignment);
    }
  }

  // Field name: object_colors
  {
    size_t array_size = ros_message->object_colors.size;
    auto array_ptr = ros_message->object_colors.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_moveit_msgs__msg__ObjectColor(
        &array_ptr[index], current_alignment);
    }
  }

  // Field name: world
  current_alignment += get_serialized_size_moveit_msgs__msg__PlanningSceneWorld(
    &(ros_message->world), current_alignment);

  // Field name: is_diff
  {
    size_t item_size = sizeof(ros_message->is_diff);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_moveit_msgs
size_t max_serialized_size_moveit_msgs__msg__PlanningScene(
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

  // Field name: name
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

  // Field name: robot_state
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

  // Field name: robot_model_name
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

  // Field name: fixed_frame_transforms
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
        max_serialized_size_geometry_msgs__msg__TransformStamped(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: allowed_collision_matrix
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_moveit_msgs__msg__AllowedCollisionMatrix(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: link_padding
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
        max_serialized_size_moveit_msgs__msg__LinkPadding(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: link_scale
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
        max_serialized_size_moveit_msgs__msg__LinkScale(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: object_colors
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

  // Field name: world
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_moveit_msgs__msg__PlanningSceneWorld(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: is_diff
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
    using DataType = moveit_msgs__msg__PlanningScene;
    is_plain =
      (
      offsetof(DataType, is_diff) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_moveit_msgs
bool cdr_serialize_key_moveit_msgs__msg__PlanningScene(
  const moveit_msgs__msg__PlanningScene * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: name
  {
    const rosidl_runtime_c__String * str = &ros_message->name;
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

  // Field name: robot_state
  {
    cdr_serialize_key_moveit_msgs__msg__RobotState(
      &ros_message->robot_state, cdr);
  }

  // Field name: robot_model_name
  {
    const rosidl_runtime_c__String * str = &ros_message->robot_model_name;
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

  // Field name: fixed_frame_transforms
  {
    size_t size = ros_message->fixed_frame_transforms.size;
    auto array_ptr = ros_message->fixed_frame_transforms.data;
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      cdr_serialize_key_geometry_msgs__msg__TransformStamped(
        &array_ptr[i], cdr);
    }
  }

  // Field name: allowed_collision_matrix
  {
    cdr_serialize_key_moveit_msgs__msg__AllowedCollisionMatrix(
      &ros_message->allowed_collision_matrix, cdr);
  }

  // Field name: link_padding
  {
    size_t size = ros_message->link_padding.size;
    auto array_ptr = ros_message->link_padding.data;
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      cdr_serialize_key_moveit_msgs__msg__LinkPadding(
        &array_ptr[i], cdr);
    }
  }

  // Field name: link_scale
  {
    size_t size = ros_message->link_scale.size;
    auto array_ptr = ros_message->link_scale.data;
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      cdr_serialize_key_moveit_msgs__msg__LinkScale(
        &array_ptr[i], cdr);
    }
  }

  // Field name: object_colors
  {
    size_t size = ros_message->object_colors.size;
    auto array_ptr = ros_message->object_colors.data;
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      cdr_serialize_key_moveit_msgs__msg__ObjectColor(
        &array_ptr[i], cdr);
    }
  }

  // Field name: world
  {
    cdr_serialize_key_moveit_msgs__msg__PlanningSceneWorld(
      &ros_message->world, cdr);
  }

  // Field name: is_diff
  {
    cdr << (ros_message->is_diff ? true : false);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_moveit_msgs
size_t get_serialized_size_key_moveit_msgs__msg__PlanningScene(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _PlanningScene__ros_msg_type * ros_message = static_cast<const _PlanningScene__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: name
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->name.size + 1);

  // Field name: robot_state
  current_alignment += get_serialized_size_key_moveit_msgs__msg__RobotState(
    &(ros_message->robot_state), current_alignment);

  // Field name: robot_model_name
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->robot_model_name.size + 1);

  // Field name: fixed_frame_transforms
  {
    size_t array_size = ros_message->fixed_frame_transforms.size;
    auto array_ptr = ros_message->fixed_frame_transforms.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_key_geometry_msgs__msg__TransformStamped(
        &array_ptr[index], current_alignment);
    }
  }

  // Field name: allowed_collision_matrix
  current_alignment += get_serialized_size_key_moveit_msgs__msg__AllowedCollisionMatrix(
    &(ros_message->allowed_collision_matrix), current_alignment);

  // Field name: link_padding
  {
    size_t array_size = ros_message->link_padding.size;
    auto array_ptr = ros_message->link_padding.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_key_moveit_msgs__msg__LinkPadding(
        &array_ptr[index], current_alignment);
    }
  }

  // Field name: link_scale
  {
    size_t array_size = ros_message->link_scale.size;
    auto array_ptr = ros_message->link_scale.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_key_moveit_msgs__msg__LinkScale(
        &array_ptr[index], current_alignment);
    }
  }

  // Field name: object_colors
  {
    size_t array_size = ros_message->object_colors.size;
    auto array_ptr = ros_message->object_colors.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_key_moveit_msgs__msg__ObjectColor(
        &array_ptr[index], current_alignment);
    }
  }

  // Field name: world
  current_alignment += get_serialized_size_key_moveit_msgs__msg__PlanningSceneWorld(
    &(ros_message->world), current_alignment);

  // Field name: is_diff
  {
    size_t item_size = sizeof(ros_message->is_diff);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_moveit_msgs
size_t max_serialized_size_key_moveit_msgs__msg__PlanningScene(
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
  // Field name: name
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

  // Field name: robot_state
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

  // Field name: robot_model_name
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

  // Field name: fixed_frame_transforms
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
        max_serialized_size_key_geometry_msgs__msg__TransformStamped(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: allowed_collision_matrix
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_moveit_msgs__msg__AllowedCollisionMatrix(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: link_padding
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
        max_serialized_size_key_moveit_msgs__msg__LinkPadding(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: link_scale
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
        max_serialized_size_key_moveit_msgs__msg__LinkScale(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: object_colors
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

  // Field name: world
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_moveit_msgs__msg__PlanningSceneWorld(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: is_diff
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
    using DataType = moveit_msgs__msg__PlanningScene;
    is_plain =
      (
      offsetof(DataType, is_diff) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _PlanningScene__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const moveit_msgs__msg__PlanningScene * ros_message = static_cast<const moveit_msgs__msg__PlanningScene *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_moveit_msgs__msg__PlanningScene(ros_message, cdr);
}

static bool _PlanningScene__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  moveit_msgs__msg__PlanningScene * ros_message = static_cast<moveit_msgs__msg__PlanningScene *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_moveit_msgs__msg__PlanningScene(cdr, ros_message);
}

static uint32_t _PlanningScene__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_moveit_msgs__msg__PlanningScene(
      untyped_ros_message, 0));
}

static size_t _PlanningScene__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_moveit_msgs__msg__PlanningScene(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_PlanningScene = {
  "moveit_msgs::msg",
  "PlanningScene",
  _PlanningScene__cdr_serialize,
  _PlanningScene__cdr_deserialize,
  _PlanningScene__get_serialized_size,
  _PlanningScene__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _PlanningScene__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_PlanningScene,
  get_message_typesupport_handle_function,
  &moveit_msgs__msg__PlanningScene__get_type_hash,
  &moveit_msgs__msg__PlanningScene__get_type_description,
  &moveit_msgs__msg__PlanningScene__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, moveit_msgs, msg, PlanningScene)() {
  return &_PlanningScene__type_support;
}

#if defined(__cplusplus)
}
#endif
