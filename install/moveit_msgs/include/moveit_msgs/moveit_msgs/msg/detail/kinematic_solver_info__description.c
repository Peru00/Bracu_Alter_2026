// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from moveit_msgs:msg/KinematicSolverInfo.idl
// generated code does not contain a copyright notice

#include "moveit_msgs/msg/detail/kinematic_solver_info__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_moveit_msgs
const rosidl_type_hash_t *
moveit_msgs__msg__KinematicSolverInfo__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xaf, 0xf8, 0x81, 0xf3, 0xa4, 0x71, 0xd4, 0x1c,
      0xfa, 0x24, 0x3c, 0xb8, 0x90, 0x5a, 0xea, 0xe0,
      0x3e, 0xce, 0xd1, 0x0a, 0xe1, 0xf2, 0x46, 0x61,
      0x38, 0x4f, 0x72, 0xd5, 0xc1, 0x7f, 0xba, 0xf2,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "moveit_msgs/msg/detail/joint_limits__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t moveit_msgs__msg__JointLimits__EXPECTED_HASH = {1, {
    0xa0, 0x98, 0x26, 0xe8, 0xee, 0xd4, 0x81, 0x58,
    0x8b, 0x73, 0xc6, 0x51, 0xf3, 0x76, 0x8a, 0xf9,
    0xb5, 0x2d, 0x21, 0x3e, 0x02, 0xc4, 0x7e, 0x5b,
    0x14, 0x7f, 0x22, 0x31, 0x00, 0xa6, 0x5a, 0x6d,
  }};
#endif

static char moveit_msgs__msg__KinematicSolverInfo__TYPE_NAME[] = "moveit_msgs/msg/KinematicSolverInfo";
static char moveit_msgs__msg__JointLimits__TYPE_NAME[] = "moveit_msgs/msg/JointLimits";

// Define type names, field names, and default values
static char moveit_msgs__msg__KinematicSolverInfo__FIELD_NAME__joint_names[] = "joint_names";
static char moveit_msgs__msg__KinematicSolverInfo__FIELD_NAME__limits[] = "limits";
static char moveit_msgs__msg__KinematicSolverInfo__FIELD_NAME__link_names[] = "link_names";

static rosidl_runtime_c__type_description__Field moveit_msgs__msg__KinematicSolverInfo__FIELDS[] = {
  {
    {moveit_msgs__msg__KinematicSolverInfo__FIELD_NAME__joint_names, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__msg__KinematicSolverInfo__FIELD_NAME__limits, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {moveit_msgs__msg__JointLimits__TYPE_NAME, 27, 27},
    },
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__msg__KinematicSolverInfo__FIELD_NAME__link_names, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription moveit_msgs__msg__KinematicSolverInfo__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {moveit_msgs__msg__JointLimits__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
moveit_msgs__msg__KinematicSolverInfo__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {moveit_msgs__msg__KinematicSolverInfo__TYPE_NAME, 35, 35},
      {moveit_msgs__msg__KinematicSolverInfo__FIELDS, 3, 3},
    },
    {moveit_msgs__msg__KinematicSolverInfo__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&moveit_msgs__msg__JointLimits__EXPECTED_HASH, moveit_msgs__msg__JointLimits__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = moveit_msgs__msg__JointLimits__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# A list of joints in the kinematic tree\n"
  "string[] joint_names\n"
  "# A list of joint limits corresponding to the joint names\n"
  "moveit_msgs/JointLimits[] limits\n"
  "# A list of links that the kinematics node provides solutions for\n"
  "string[] link_names";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
moveit_msgs__msg__KinematicSolverInfo__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {moveit_msgs__msg__KinematicSolverInfo__TYPE_NAME, 35, 35},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 239, 239},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
moveit_msgs__msg__KinematicSolverInfo__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *moveit_msgs__msg__KinematicSolverInfo__get_individual_type_description_source(NULL),
    sources[1] = *moveit_msgs__msg__JointLimits__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
