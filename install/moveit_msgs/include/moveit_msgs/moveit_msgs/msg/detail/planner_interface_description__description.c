// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from moveit_msgs:msg/PlannerInterfaceDescription.idl
// generated code does not contain a copyright notice

#include "moveit_msgs/msg/detail/planner_interface_description__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_moveit_msgs
const rosidl_type_hash_t *
moveit_msgs__msg__PlannerInterfaceDescription__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xbe, 0xfe, 0x55, 0xb3, 0x5f, 0x33, 0x7f, 0x0f,
      0xd4, 0xf8, 0x4d, 0xdd, 0xa3, 0xfe, 0x08, 0x69,
      0x60, 0xc4, 0x7e, 0xf4, 0x95, 0xa6, 0xb2, 0xa3,
      0x69, 0x99, 0x1b, 0x37, 0x38, 0xb8, 0xf2, 0x20,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char moveit_msgs__msg__PlannerInterfaceDescription__TYPE_NAME[] = "moveit_msgs/msg/PlannerInterfaceDescription";

// Define type names, field names, and default values
static char moveit_msgs__msg__PlannerInterfaceDescription__FIELD_NAME__name[] = "name";
static char moveit_msgs__msg__PlannerInterfaceDescription__FIELD_NAME__pipeline_id[] = "pipeline_id";
static char moveit_msgs__msg__PlannerInterfaceDescription__FIELD_NAME__planner_ids[] = "planner_ids";

static rosidl_runtime_c__type_description__Field moveit_msgs__msg__PlannerInterfaceDescription__FIELDS[] = {
  {
    {moveit_msgs__msg__PlannerInterfaceDescription__FIELD_NAME__name, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__msg__PlannerInterfaceDescription__FIELD_NAME__pipeline_id, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__msg__PlannerInterfaceDescription__FIELD_NAME__planner_ids, 11, 11},
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
moveit_msgs__msg__PlannerInterfaceDescription__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {moveit_msgs__msg__PlannerInterfaceDescription__TYPE_NAME, 43, 43},
      {moveit_msgs__msg__PlannerInterfaceDescription__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# The name of the planner interface\n"
  "string name\n"
  "\n"
  "# The name of the planning pipeline\n"
  "string pipeline_id\n"
  "\n"
  "# The names of the planner ids within the interface\n"
  "string[] planner_ids";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
moveit_msgs__msg__PlannerInterfaceDescription__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {moveit_msgs__msg__PlannerInterfaceDescription__TYPE_NAME, 43, 43},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 178, 178},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
moveit_msgs__msg__PlannerInterfaceDescription__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *moveit_msgs__msg__PlannerInterfaceDescription__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
