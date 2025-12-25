// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from moveit_msgs:msg/LinkPadding.idl
// generated code does not contain a copyright notice

#include "moveit_msgs/msg/detail/link_padding__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_moveit_msgs
const rosidl_type_hash_t *
moveit_msgs__msg__LinkPadding__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x86, 0x47, 0x62, 0xd9, 0xe1, 0x9f, 0x75, 0x8b,
      0x03, 0x85, 0x42, 0x7f, 0x75, 0x69, 0x13, 0x74,
      0xca, 0xe8, 0x30, 0x49, 0x57, 0x27, 0xc9, 0x64,
      0x05, 0xf2, 0xba, 0xd0, 0xeb, 0x20, 0xc7, 0x66,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char moveit_msgs__msg__LinkPadding__TYPE_NAME[] = "moveit_msgs/msg/LinkPadding";

// Define type names, field names, and default values
static char moveit_msgs__msg__LinkPadding__FIELD_NAME__link_name[] = "link_name";
static char moveit_msgs__msg__LinkPadding__FIELD_NAME__padding[] = "padding";

static rosidl_runtime_c__type_description__Field moveit_msgs__msg__LinkPadding__FIELDS[] = {
  {
    {moveit_msgs__msg__LinkPadding__FIELD_NAME__link_name, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__msg__LinkPadding__FIELD_NAME__padding, 7, 7},
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
moveit_msgs__msg__LinkPadding__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {moveit_msgs__msg__LinkPadding__TYPE_NAME, 27, 27},
      {moveit_msgs__msg__LinkPadding__FIELDS, 2, 2},
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
  "# padding to apply to the link\n"
  "float64 padding";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
moveit_msgs__msg__LinkPadding__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {moveit_msgs__msg__LinkPadding__TYPE_NAME, 27, 27},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 84, 84},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
moveit_msgs__msg__LinkPadding__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *moveit_msgs__msg__LinkPadding__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
