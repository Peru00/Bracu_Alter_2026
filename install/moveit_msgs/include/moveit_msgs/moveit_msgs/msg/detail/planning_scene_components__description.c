// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from moveit_msgs:msg/PlanningSceneComponents.idl
// generated code does not contain a copyright notice

#include "moveit_msgs/msg/detail/planning_scene_components__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_moveit_msgs
const rosidl_type_hash_t *
moveit_msgs__msg__PlanningSceneComponents__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x76, 0xb7, 0x7c, 0xb5, 0x3f, 0x7a, 0x0b, 0x68,
      0x0d, 0x46, 0xfa, 0xd6, 0xc0, 0x05, 0x6d, 0x37,
      0x10, 0xa7, 0x4e, 0xc3, 0x35, 0xdd, 0x5f, 0x36,
      0x1f, 0x3d, 0xf6, 0x23, 0x83, 0x31, 0x3d, 0x69,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char moveit_msgs__msg__PlanningSceneComponents__TYPE_NAME[] = "moveit_msgs/msg/PlanningSceneComponents";

// Define type names, field names, and default values
static char moveit_msgs__msg__PlanningSceneComponents__FIELD_NAME__components[] = "components";

static rosidl_runtime_c__type_description__Field moveit_msgs__msg__PlanningSceneComponents__FIELDS[] = {
  {
    {moveit_msgs__msg__PlanningSceneComponents__FIELD_NAME__components, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
moveit_msgs__msg__PlanningSceneComponents__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {moveit_msgs__msg__PlanningSceneComponents__TYPE_NAME, 39, 39},
      {moveit_msgs__msg__PlanningSceneComponents__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# This message defines the components that make up the PlanningScene message.\n"
  "# The values can be used as a bitfield to specify which parts of the PlanningScene message\n"
  "# are of interest\n"
  "\n"
  "# Scene name, model name, model root\n"
  "uint32 SCENE_SETTINGS=1\n"
  "\n"
  "# Joint values of the robot state\n"
  "uint32 ROBOT_STATE=2\n"
  "\n"
  "# Attached objects (including geometry) for the robot state\n"
  "uint32 ROBOT_STATE_ATTACHED_OBJECTS=4\n"
  "\n"
  "# The names of the world objects\n"
  "uint32 WORLD_OBJECT_NAMES=8\n"
  "\n"
  "# The geometry of the world objects\n"
  "uint32 WORLD_OBJECT_GEOMETRY=16\n"
  "\n"
  "# The maintained octomap\n"
  "uint32 OCTOMAP=32\n"
  "\n"
  "# The maintained list of transforms\n"
  "uint32 TRANSFORMS=64\n"
  "\n"
  "# The allowed collision matrix\n"
  "uint32 ALLOWED_COLLISION_MATRIX=128\n"
  "\n"
  "# The default link padding and link scaling\n"
  "uint32 LINK_PADDING_AND_SCALING=256\n"
  "\n"
  "# The stored object colors\n"
  "uint32 OBJECT_COLORS=512\n"
  "\n"
  "# Bitfield combining options indicated above\n"
  "uint32 components";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
moveit_msgs__msg__PlanningSceneComponents__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {moveit_msgs__msg__PlanningSceneComponents__TYPE_NAME, 39, 39},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 903, 903},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
moveit_msgs__msg__PlanningSceneComponents__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *moveit_msgs__msg__PlanningSceneComponents__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
