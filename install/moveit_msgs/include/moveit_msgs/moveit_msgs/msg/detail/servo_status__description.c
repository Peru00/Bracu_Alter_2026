// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from moveit_msgs:msg/ServoStatus.idl
// generated code does not contain a copyright notice

#include "moveit_msgs/msg/detail/servo_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_moveit_msgs
const rosidl_type_hash_t *
moveit_msgs__msg__ServoStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x88, 0x2a, 0x75, 0xf7, 0x64, 0x8d, 0x6f, 0x8a,
      0x9b, 0xf5, 0x28, 0xe4, 0x78, 0x7f, 0x51, 0xc5,
      0x8f, 0x2d, 0x5d, 0x88, 0xb7, 0x02, 0xe4, 0x50,
      0x44, 0x3d, 0x99, 0xf2, 0x70, 0xe6, 0x93, 0x6b,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char moveit_msgs__msg__ServoStatus__TYPE_NAME[] = "moveit_msgs/msg/ServoStatus";

// Define type names, field names, and default values
static char moveit_msgs__msg__ServoStatus__FIELD_NAME__code[] = "code";
static char moveit_msgs__msg__ServoStatus__FIELD_NAME__message[] = "message";

static rosidl_runtime_c__type_description__Field moveit_msgs__msg__ServoStatus__FIELDS[] = {
  {
    {moveit_msgs__msg__ServoStatus__FIELD_NAME__code, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__msg__ServoStatus__FIELD_NAME__message, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
moveit_msgs__msg__ServoStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {moveit_msgs__msg__ServoStatus__TYPE_NAME, 27, 27},
      {moveit_msgs__msg__ServoStatus__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Message\n"
  "int8 code # will contains integer code\n"
  "string message # will contain explanatory message\n"
  "\n"
  "# Status types (should reflect StatusCode from moveit_servo/utils/datatype.hpp)\n"
  "int8 INVALID = -1\n"
  "int8 NO_WARNING = 0\n"
  "int8 DECELERATE_FOR_APPROACHING_SINGULARITY = 1\n"
  "int8 HALT_FOR_SINGULARITY = 2\n"
  "int8 DECELERATE_FOR_LEAVING_SINGULARITY = 3\n"
  "int8 DECELERATE_FOR_COLLISION = 4\n"
  "int8 HALT_FOR_COLLISION = 5\n"
  "int8 JOINT_BOUND = 6";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
moveit_msgs__msg__ServoStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {moveit_msgs__msg__ServoStatus__TYPE_NAME, 27, 27},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 423, 423},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
moveit_msgs__msg__ServoStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *moveit_msgs__msg__ServoStatus__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
