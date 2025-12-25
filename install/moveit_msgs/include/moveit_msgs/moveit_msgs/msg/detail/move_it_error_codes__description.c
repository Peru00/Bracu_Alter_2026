// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from moveit_msgs:msg/MoveItErrorCodes.idl
// generated code does not contain a copyright notice

#include "moveit_msgs/msg/detail/move_it_error_codes__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_moveit_msgs
const rosidl_type_hash_t *
moveit_msgs__msg__MoveItErrorCodes__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xdc, 0xda, 0x8a, 0x99, 0x50, 0xcf, 0x6d, 0xd6,
      0xac, 0x22, 0x1c, 0x2c, 0x0d, 0x25, 0x09, 0xf5,
      0x2f, 0x13, 0x6a, 0xfa, 0x04, 0x03, 0xdd, 0x19,
      0xbe, 0xac, 0xdf, 0x07, 0xe8, 0x3e, 0x4a, 0x07,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char moveit_msgs__msg__MoveItErrorCodes__TYPE_NAME[] = "moveit_msgs/msg/MoveItErrorCodes";

// Define type names, field names, and default values
static char moveit_msgs__msg__MoveItErrorCodes__FIELD_NAME__val[] = "val";
static char moveit_msgs__msg__MoveItErrorCodes__FIELD_NAME__message[] = "message";
static char moveit_msgs__msg__MoveItErrorCodes__FIELD_NAME__source[] = "source";

static rosidl_runtime_c__type_description__Field moveit_msgs__msg__MoveItErrorCodes__FIELDS[] = {
  {
    {moveit_msgs__msg__MoveItErrorCodes__FIELD_NAME__val, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__msg__MoveItErrorCodes__FIELD_NAME__message, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__msg__MoveItErrorCodes__FIELD_NAME__source, 6, 6},
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
moveit_msgs__msg__MoveItErrorCodes__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {moveit_msgs__msg__MoveItErrorCodes__TYPE_NAME, 32, 32},
      {moveit_msgs__msg__MoveItErrorCodes__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "int32 val\n"
  "\n"
  "# overall behavior\n"
  "int32 SUCCESS=1\n"
  "int32 UNDEFINED=0\n"
  "int32 FAILURE=99999\n"
  "\n"
  "int32 PLANNING_FAILED=-1\n"
  "int32 INVALID_MOTION_PLAN=-2\n"
  "int32 MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE=-3\n"
  "int32 CONTROL_FAILED=-4\n"
  "int32 UNABLE_TO_AQUIRE_SENSOR_DATA=-5\n"
  "int32 TIMED_OUT=-6\n"
  "int32 PREEMPTED=-7\n"
  "\n"
  "# planning & kinematics request errors\n"
  "int32 START_STATE_IN_COLLISION=-10\n"
  "int32 START_STATE_VIOLATES_PATH_CONSTRAINTS=-11\n"
  "int32 START_STATE_INVALID=-26\n"
  "\n"
  "int32 GOAL_IN_COLLISION=-12\n"
  "int32 GOAL_VIOLATES_PATH_CONSTRAINTS=-13\n"
  "int32 GOAL_CONSTRAINTS_VIOLATED=-14\n"
  "int32 GOAL_STATE_INVALID=-27\n"
  "int32 UNRECOGNIZED_GOAL_TYPE=-28\n"
  "\n"
  "int32 INVALID_GROUP_NAME=-15\n"
  "int32 INVALID_GOAL_CONSTRAINTS=-16\n"
  "int32 INVALID_ROBOT_STATE=-17\n"
  "int32 INVALID_LINK_NAME=-18\n"
  "int32 INVALID_OBJECT_NAME=-19\n"
  "\n"
  "# system errors\n"
  "int32 FRAME_TRANSFORM_FAILURE=-21\n"
  "int32 COLLISION_CHECKING_UNAVAILABLE=-22\n"
  "int32 ROBOT_STATE_STALE=-23\n"
  "int32 SENSOR_INFO_STALE=-24\n"
  "int32 COMMUNICATION_FAILURE=-25\n"
  "int32 CRASH=-29\n"
  "int32 ABORT=-30\n"
  "\n"
  "# kinematics errors\n"
  "int32 NO_IK_SOLUTION=-31\n"
  "\n"
  "# A message to provide additional information.\n"
  "string message\n"
  "\n"
  "# Name of the component that created the status.\n"
  "# This is helpful to locate error source.\n"
  "string source";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
moveit_msgs__msg__MoveItErrorCodes__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {moveit_msgs__msg__MoveItErrorCodes__TYPE_NAME, 32, 32},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 1195, 1195},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
moveit_msgs__msg__MoveItErrorCodes__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *moveit_msgs__msg__MoveItErrorCodes__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
