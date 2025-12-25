// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from moveit_msgs:msg/AllowedCollisionEntry.idl
// generated code does not contain a copyright notice

#include "moveit_msgs/msg/detail/allowed_collision_entry__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_moveit_msgs
const rosidl_type_hash_t *
moveit_msgs__msg__AllowedCollisionEntry__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xee, 0xb7, 0xcc, 0x3f, 0x1b, 0xfd, 0xb8, 0xc7,
      0x78, 0xaa, 0x74, 0x54, 0xae, 0x90, 0x03, 0x7c,
      0x27, 0x49, 0xda, 0xb3, 0xca, 0x0a, 0xbd, 0xfe,
      0x3d, 0xad, 0x2a, 0xe8, 0x72, 0x39, 0x77, 0xdb,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char moveit_msgs__msg__AllowedCollisionEntry__TYPE_NAME[] = "moveit_msgs/msg/AllowedCollisionEntry";

// Define type names, field names, and default values
static char moveit_msgs__msg__AllowedCollisionEntry__FIELD_NAME__enabled[] = "enabled";

static rosidl_runtime_c__type_description__Field moveit_msgs__msg__AllowedCollisionEntry__FIELDS[] = {
  {
    {moveit_msgs__msg__AllowedCollisionEntry__FIELD_NAME__enabled, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
moveit_msgs__msg__AllowedCollisionEntry__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {moveit_msgs__msg__AllowedCollisionEntry__TYPE_NAME, 37, 37},
      {moveit_msgs__msg__AllowedCollisionEntry__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# whether or not collision checking is enabled\n"
  "bool[] enabled";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
moveit_msgs__msg__AllowedCollisionEntry__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {moveit_msgs__msg__AllowedCollisionEntry__TYPE_NAME, 37, 37},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 62, 62},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
moveit_msgs__msg__AllowedCollisionEntry__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *moveit_msgs__msg__AllowedCollisionEntry__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
