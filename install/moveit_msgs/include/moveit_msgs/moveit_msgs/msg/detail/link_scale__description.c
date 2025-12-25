// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from moveit_msgs:msg/LinkScale.idl
// generated code does not contain a copyright notice

#include "moveit_msgs/msg/detail/link_scale__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_moveit_msgs
const rosidl_type_hash_t *
moveit_msgs__msg__LinkScale__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x46, 0x66, 0xa0, 0x68, 0x5f, 0x26, 0x36, 0x5c,
      0xcf, 0x4c, 0x9b, 0xfe, 0xd0, 0xb1, 0x4c, 0x65,
      0x46, 0x04, 0xfa, 0x05, 0x64, 0xbe, 0x65, 0xc2,
      0x21, 0xd8, 0xd5, 0x4f, 0xdc, 0x7a, 0x0f, 0x83,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char moveit_msgs__msg__LinkScale__TYPE_NAME[] = "moveit_msgs/msg/LinkScale";

// Define type names, field names, and default values
static char moveit_msgs__msg__LinkScale__FIELD_NAME__link_name[] = "link_name";
static char moveit_msgs__msg__LinkScale__FIELD_NAME__scale[] = "scale";

static rosidl_runtime_c__type_description__Field moveit_msgs__msg__LinkScale__FIELDS[] = {
  {
    {moveit_msgs__msg__LinkScale__FIELD_NAME__link_name, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__msg__LinkScale__FIELD_NAME__scale, 5, 5},
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
moveit_msgs__msg__LinkScale__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {moveit_msgs__msg__LinkScale__TYPE_NAME, 25, 25},
      {moveit_msgs__msg__LinkScale__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "#name for the link\n"
  "string link_name\n"
  "\n"
  "# scaling to apply to the link\n"
  "float64 scale";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
moveit_msgs__msg__LinkScale__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {moveit_msgs__msg__LinkScale__TYPE_NAME, 25, 25},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 82, 82},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
moveit_msgs__msg__LinkScale__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *moveit_msgs__msg__LinkScale__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
