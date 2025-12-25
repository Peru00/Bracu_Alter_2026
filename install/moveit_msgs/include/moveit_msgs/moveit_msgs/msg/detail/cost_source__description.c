// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from moveit_msgs:msg/CostSource.idl
// generated code does not contain a copyright notice

#include "moveit_msgs/msg/detail/cost_source__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_moveit_msgs
const rosidl_type_hash_t *
moveit_msgs__msg__CostSource__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xdd, 0x6b, 0xd4, 0xe3, 0x83, 0xe1, 0x02, 0xb4,
      0xb7, 0xef, 0x54, 0xe4, 0x6e, 0x2f, 0x8c, 0xd4,
      0xb4, 0x32, 0x3a, 0x2c, 0x56, 0x37, 0x0c, 0x42,
      0x86, 0x2b, 0xd2, 0xd3, 0x35, 0x8a, 0x59, 0xf6,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "geometry_msgs/msg/detail/vector3__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t geometry_msgs__msg__Vector3__EXPECTED_HASH = {1, {
    0xcc, 0x12, 0xfe, 0x83, 0xe4, 0xc0, 0x27, 0x19,
    0xf1, 0xce, 0x80, 0x70, 0xbf, 0xd1, 0x4a, 0xec,
    0xd4, 0x0f, 0x75, 0xa9, 0x66, 0x96, 0xa6, 0x7a,
    0x2a, 0x1f, 0x37, 0xf7, 0xdb, 0xb0, 0x76, 0x5d,
  }};
#endif

static char moveit_msgs__msg__CostSource__TYPE_NAME[] = "moveit_msgs/msg/CostSource";
static char geometry_msgs__msg__Vector3__TYPE_NAME[] = "geometry_msgs/msg/Vector3";

// Define type names, field names, and default values
static char moveit_msgs__msg__CostSource__FIELD_NAME__cost_density[] = "cost_density";
static char moveit_msgs__msg__CostSource__FIELD_NAME__aabb_min[] = "aabb_min";
static char moveit_msgs__msg__CostSource__FIELD_NAME__aabb_max[] = "aabb_max";

static rosidl_runtime_c__type_description__Field moveit_msgs__msg__CostSource__FIELDS[] = {
  {
    {moveit_msgs__msg__CostSource__FIELD_NAME__cost_density, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__msg__CostSource__FIELD_NAME__aabb_min, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {geometry_msgs__msg__Vector3__TYPE_NAME, 25, 25},
    },
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__msg__CostSource__FIELD_NAME__aabb_max, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {geometry_msgs__msg__Vector3__TYPE_NAME, 25, 25},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription moveit_msgs__msg__CostSource__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {geometry_msgs__msg__Vector3__TYPE_NAME, 25, 25},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
moveit_msgs__msg__CostSource__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {moveit_msgs__msg__CostSource__TYPE_NAME, 26, 26},
      {moveit_msgs__msg__CostSource__FIELDS, 3, 3},
    },
    {moveit_msgs__msg__CostSource__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&geometry_msgs__msg__Vector3__EXPECTED_HASH, geometry_msgs__msg__Vector3__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = geometry_msgs__msg__Vector3__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# The density of the cost source\n"
  "float64 cost_density\n"
  "\n"
  "# The volume of the cost source is represented as an\n"
  "# axis-aligned bounding box (AABB)\n"
  "# The AABB is specified by two of its opposite corners\n"
  "\n"
  "geometry_msgs/Vector3 aabb_min\n"
  "\n"
  "geometry_msgs/Vector3 aabb_max";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
moveit_msgs__msg__CostSource__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {moveit_msgs__msg__CostSource__TYPE_NAME, 26, 26},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 262, 262},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
moveit_msgs__msg__CostSource__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *moveit_msgs__msg__CostSource__get_individual_type_description_source(NULL),
    sources[1] = *geometry_msgs__msg__Vector3__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
