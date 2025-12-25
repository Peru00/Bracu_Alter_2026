// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from moveit_msgs:msg/JointLimits.idl
// generated code does not contain a copyright notice

#include "moveit_msgs/msg/detail/joint_limits__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_moveit_msgs
const rosidl_type_hash_t *
moveit_msgs__msg__JointLimits__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xa0, 0x98, 0x26, 0xe8, 0xee, 0xd4, 0x81, 0x58,
      0x8b, 0x73, 0xc6, 0x51, 0xf3, 0x76, 0x8a, 0xf9,
      0xb5, 0x2d, 0x21, 0x3e, 0x02, 0xc4, 0x7e, 0x5b,
      0x14, 0x7f, 0x22, 0x31, 0x00, 0xa6, 0x5a, 0x6d,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char moveit_msgs__msg__JointLimits__TYPE_NAME[] = "moveit_msgs/msg/JointLimits";

// Define type names, field names, and default values
static char moveit_msgs__msg__JointLimits__FIELD_NAME__joint_name[] = "joint_name";
static char moveit_msgs__msg__JointLimits__FIELD_NAME__has_position_limits[] = "has_position_limits";
static char moveit_msgs__msg__JointLimits__FIELD_NAME__min_position[] = "min_position";
static char moveit_msgs__msg__JointLimits__FIELD_NAME__max_position[] = "max_position";
static char moveit_msgs__msg__JointLimits__FIELD_NAME__has_velocity_limits[] = "has_velocity_limits";
static char moveit_msgs__msg__JointLimits__FIELD_NAME__max_velocity[] = "max_velocity";
static char moveit_msgs__msg__JointLimits__FIELD_NAME__has_acceleration_limits[] = "has_acceleration_limits";
static char moveit_msgs__msg__JointLimits__FIELD_NAME__max_acceleration[] = "max_acceleration";
static char moveit_msgs__msg__JointLimits__FIELD_NAME__has_jerk_limits[] = "has_jerk_limits";
static char moveit_msgs__msg__JointLimits__FIELD_NAME__max_jerk[] = "max_jerk";

static rosidl_runtime_c__type_description__Field moveit_msgs__msg__JointLimits__FIELDS[] = {
  {
    {moveit_msgs__msg__JointLimits__FIELD_NAME__joint_name, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__msg__JointLimits__FIELD_NAME__has_position_limits, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__msg__JointLimits__FIELD_NAME__min_position, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__msg__JointLimits__FIELD_NAME__max_position, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__msg__JointLimits__FIELD_NAME__has_velocity_limits, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__msg__JointLimits__FIELD_NAME__max_velocity, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__msg__JointLimits__FIELD_NAME__has_acceleration_limits, 23, 23},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__msg__JointLimits__FIELD_NAME__max_acceleration, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__msg__JointLimits__FIELD_NAME__has_jerk_limits, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__msg__JointLimits__FIELD_NAME__max_jerk, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
moveit_msgs__msg__JointLimits__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {moveit_msgs__msg__JointLimits__TYPE_NAME, 27, 27},
      {moveit_msgs__msg__JointLimits__FIELDS, 10, 10},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# This message contains information about limits of a particular joint (or control dimension)\n"
  "string joint_name\n"
  "\n"
  "# true if the joint has position limits\n"
  "bool has_position_limits\n"
  "\n"
  "# min and max position limits\n"
  "float64 min_position\n"
  "float64 max_position\n"
  "\n"
  "# true if joint has velocity limits\n"
  "bool has_velocity_limits\n"
  "\n"
  "# max velocity limit\n"
  "float64 max_velocity\n"
  "# min_velocity is assumed to be -max_velocity\n"
  "\n"
  "# true if joint has acceleration limits\n"
  "bool has_acceleration_limits\n"
  "# max acceleration limit\n"
  "float64 max_acceleration\n"
  "# min_acceleration is assumed to be -max_acceleration\n"
  "\n"
  "# true if joint has jerk limits\n"
  "bool has_jerk_limits\n"
  "# max jerk limit\n"
  "float64 max_jerk\n"
  "# min_jerk is assumed to be -max_jerk";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
moveit_msgs__msg__JointLimits__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {moveit_msgs__msg__JointLimits__TYPE_NAME, 27, 27},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 702, 702},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
moveit_msgs__msg__JointLimits__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *moveit_msgs__msg__JointLimits__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
