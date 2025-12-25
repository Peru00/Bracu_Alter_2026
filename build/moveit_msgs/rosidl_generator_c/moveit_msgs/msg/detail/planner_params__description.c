// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from moveit_msgs:msg/PlannerParams.idl
// generated code does not contain a copyright notice

#include "moveit_msgs/msg/detail/planner_params__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_moveit_msgs
const rosidl_type_hash_t *
moveit_msgs__msg__PlannerParams__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x91, 0xfe, 0x6e, 0xbf, 0x17, 0x15, 0x8d, 0x27,
      0x24, 0x01, 0x18, 0xbc, 0x8c, 0xce, 0xde, 0x6c,
      0xca, 0x55, 0xd6, 0x31, 0xc9, 0xa1, 0xcb, 0xce,
      0x84, 0x7b, 0x34, 0x27, 0x61, 0x26, 0x9d, 0x49,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char moveit_msgs__msg__PlannerParams__TYPE_NAME[] = "moveit_msgs/msg/PlannerParams";

// Define type names, field names, and default values
static char moveit_msgs__msg__PlannerParams__FIELD_NAME__keys[] = "keys";
static char moveit_msgs__msg__PlannerParams__FIELD_NAME__values[] = "values";
static char moveit_msgs__msg__PlannerParams__FIELD_NAME__descriptions[] = "descriptions";

static rosidl_runtime_c__type_description__Field moveit_msgs__msg__PlannerParams__FIELDS[] = {
  {
    {moveit_msgs__msg__PlannerParams__FIELD_NAME__keys, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__msg__PlannerParams__FIELD_NAME__values, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__msg__PlannerParams__FIELD_NAME__descriptions, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
moveit_msgs__msg__PlannerParams__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {moveit_msgs__msg__PlannerParams__TYPE_NAME, 29, 29},
      {moveit_msgs__msg__PlannerParams__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# parameter names (same size as values)\n"
  "string[] keys\n"
  "\n"
  "# parameter values (same size as keys)\n"
  "string[] values\n"
  "\n"
  "# parameter description (can be empty)\n"
  "string[] descriptions";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
moveit_msgs__msg__PlannerParams__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {moveit_msgs__msg__PlannerParams__TYPE_NAME, 29, 29},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 172, 172},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
moveit_msgs__msg__PlannerParams__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *moveit_msgs__msg__PlannerParams__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
