// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from moveit_msgs:msg/AllowedCollisionMatrix.idl
// generated code does not contain a copyright notice

#include "moveit_msgs/msg/detail/allowed_collision_matrix__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_moveit_msgs
const rosidl_type_hash_t *
moveit_msgs__msg__AllowedCollisionMatrix__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xac, 0x01, 0x59, 0xee, 0xc0, 0xd9, 0x9f, 0x73,
      0x15, 0x8c, 0x73, 0x49, 0xc9, 0xd1, 0xc0, 0x1b,
      0x3d, 0xc7, 0xf7, 0x49, 0xd7, 0x5a, 0x2e, 0xa2,
      0x2e, 0x74, 0xa2, 0xa1, 0x09, 0x1a, 0x9b, 0x05,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "moveit_msgs/msg/detail/allowed_collision_entry__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t moveit_msgs__msg__AllowedCollisionEntry__EXPECTED_HASH = {1, {
    0xee, 0xb7, 0xcc, 0x3f, 0x1b, 0xfd, 0xb8, 0xc7,
    0x78, 0xaa, 0x74, 0x54, 0xae, 0x90, 0x03, 0x7c,
    0x27, 0x49, 0xda, 0xb3, 0xca, 0x0a, 0xbd, 0xfe,
    0x3d, 0xad, 0x2a, 0xe8, 0x72, 0x39, 0x77, 0xdb,
  }};
#endif

static char moveit_msgs__msg__AllowedCollisionMatrix__TYPE_NAME[] = "moveit_msgs/msg/AllowedCollisionMatrix";
static char moveit_msgs__msg__AllowedCollisionEntry__TYPE_NAME[] = "moveit_msgs/msg/AllowedCollisionEntry";

// Define type names, field names, and default values
static char moveit_msgs__msg__AllowedCollisionMatrix__FIELD_NAME__entry_names[] = "entry_names";
static char moveit_msgs__msg__AllowedCollisionMatrix__FIELD_NAME__entry_values[] = "entry_values";
static char moveit_msgs__msg__AllowedCollisionMatrix__FIELD_NAME__default_entry_names[] = "default_entry_names";
static char moveit_msgs__msg__AllowedCollisionMatrix__FIELD_NAME__default_entry_values[] = "default_entry_values";

static rosidl_runtime_c__type_description__Field moveit_msgs__msg__AllowedCollisionMatrix__FIELDS[] = {
  {
    {moveit_msgs__msg__AllowedCollisionMatrix__FIELD_NAME__entry_names, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__msg__AllowedCollisionMatrix__FIELD_NAME__entry_values, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {moveit_msgs__msg__AllowedCollisionEntry__TYPE_NAME, 37, 37},
    },
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__msg__AllowedCollisionMatrix__FIELD_NAME__default_entry_names, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__msg__AllowedCollisionMatrix__FIELD_NAME__default_entry_values, 20, 20},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription moveit_msgs__msg__AllowedCollisionMatrix__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {moveit_msgs__msg__AllowedCollisionEntry__TYPE_NAME, 37, 37},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
moveit_msgs__msg__AllowedCollisionMatrix__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {moveit_msgs__msg__AllowedCollisionMatrix__TYPE_NAME, 38, 38},
      {moveit_msgs__msg__AllowedCollisionMatrix__FIELDS, 4, 4},
    },
    {moveit_msgs__msg__AllowedCollisionMatrix__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&moveit_msgs__msg__AllowedCollisionEntry__EXPECTED_HASH, moveit_msgs__msg__AllowedCollisionEntry__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = moveit_msgs__msg__AllowedCollisionEntry__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# The list of entry names in the matrix\n"
  "string[] entry_names\n"
  "\n"
  "# The individual entries in the allowed collision matrix\n"
  "# square, symmetric, with same order as entry_names\n"
  "AllowedCollisionEntry[] entry_values\n"
  "\n"
  "# In addition to the collision matrix itself, we also have\n"
  "# the default entry value for each entry name.\n"
  "\n"
  "# If the allowed collision flag is queried for a pair of names (n1, n2)\n"
  "# that is not found in the collision matrix itself, the value of\n"
  "# the collision flag is considered to be that of the entry (n1 or n2)\n"
  "# specified in the list below. If both n1 and n2 are found in the list\n"
  "# of defaults, the result is computed with an AND operation\n"
  "\n"
  "string[] default_entry_names\n"
  "bool[] default_entry_values";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
moveit_msgs__msg__AllowedCollisionMatrix__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {moveit_msgs__msg__AllowedCollisionMatrix__TYPE_NAME, 38, 38},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 712, 712},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
moveit_msgs__msg__AllowedCollisionMatrix__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *moveit_msgs__msg__AllowedCollisionMatrix__get_individual_type_description_source(NULL),
    sources[1] = *moveit_msgs__msg__AllowedCollisionEntry__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
