// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from moveit_msgs:msg/JointConstraint.idl
// generated code does not contain a copyright notice

#include "moveit_msgs/msg/detail/joint_constraint__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_moveit_msgs
const rosidl_type_hash_t *
moveit_msgs__msg__JointConstraint__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x49, 0x96, 0x64, 0x1a, 0x8e, 0x12, 0xf5, 0xb7,
      0xfd, 0xe0, 0x7f, 0x1c, 0xec, 0x14, 0x90, 0x32,
      0x4e, 0x1c, 0x40, 0x35, 0xac, 0x5f, 0xc9, 0x89,
      0xce, 0x99, 0xc9, 0x60, 0x0c, 0x31, 0xe3, 0x0d,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char moveit_msgs__msg__JointConstraint__TYPE_NAME[] = "moveit_msgs/msg/JointConstraint";

// Define type names, field names, and default values
static char moveit_msgs__msg__JointConstraint__FIELD_NAME__joint_name[] = "joint_name";
static char moveit_msgs__msg__JointConstraint__FIELD_NAME__position[] = "position";
static char moveit_msgs__msg__JointConstraint__FIELD_NAME__tolerance_above[] = "tolerance_above";
static char moveit_msgs__msg__JointConstraint__FIELD_NAME__tolerance_below[] = "tolerance_below";
static char moveit_msgs__msg__JointConstraint__FIELD_NAME__weight[] = "weight";

static rosidl_runtime_c__type_description__Field moveit_msgs__msg__JointConstraint__FIELDS[] = {
  {
    {moveit_msgs__msg__JointConstraint__FIELD_NAME__joint_name, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__msg__JointConstraint__FIELD_NAME__position, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__msg__JointConstraint__FIELD_NAME__tolerance_above, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__msg__JointConstraint__FIELD_NAME__tolerance_below, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__msg__JointConstraint__FIELD_NAME__weight, 6, 6},
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
moveit_msgs__msg__JointConstraint__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {moveit_msgs__msg__JointConstraint__TYPE_NAME, 31, 31},
      {moveit_msgs__msg__JointConstraint__FIELDS, 5, 5},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Constrain the position of a joint to be within a certain bound\n"
  "string joint_name\n"
  "\n"
  "# the bound to be achieved is [position - tolerance_below, position + tolerance_above]\n"
  "float64 position\n"
  "float64 tolerance_above\n"
  "float64 tolerance_below\n"
  "\n"
  "# A weighting factor for this constraint (denotes relative importance to other constraints. Closer to zero means less important)\n"
  "float64 weight";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
moveit_msgs__msg__JointConstraint__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {moveit_msgs__msg__JointConstraint__TYPE_NAME, 31, 31},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 381, 381},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
moveit_msgs__msg__JointConstraint__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *moveit_msgs__msg__JointConstraint__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
