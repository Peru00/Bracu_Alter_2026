// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from moveit_msgs:msg/ConstraintEvalResult.idl
// generated code does not contain a copyright notice

#include "moveit_msgs/msg/detail/constraint_eval_result__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_moveit_msgs
const rosidl_type_hash_t *
moveit_msgs__msg__ConstraintEvalResult__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xb6, 0x32, 0xab, 0x96, 0xd6, 0x8f, 0xbf, 0xd5,
      0x87, 0x54, 0x6b, 0x1c, 0x58, 0x5a, 0x3b, 0x8b,
      0xe0, 0x60, 0x6b, 0x7d, 0xe7, 0x75, 0x19, 0x29,
      0x65, 0x33, 0x92, 0xb3, 0x43, 0x03, 0x99, 0x97,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char moveit_msgs__msg__ConstraintEvalResult__TYPE_NAME[] = "moveit_msgs/msg/ConstraintEvalResult";

// Define type names, field names, and default values
static char moveit_msgs__msg__ConstraintEvalResult__FIELD_NAME__result[] = "result";
static char moveit_msgs__msg__ConstraintEvalResult__FIELD_NAME__distance[] = "distance";

static rosidl_runtime_c__type_description__Field moveit_msgs__msg__ConstraintEvalResult__FIELDS[] = {
  {
    {moveit_msgs__msg__ConstraintEvalResult__FIELD_NAME__result, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__msg__ConstraintEvalResult__FIELD_NAME__distance, 8, 8},
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
moveit_msgs__msg__ConstraintEvalResult__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {moveit_msgs__msg__ConstraintEvalResult__TYPE_NAME, 36, 36},
      {moveit_msgs__msg__ConstraintEvalResult__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# This message contains result from constraint evaluation\n"
  "# result specifies the result of constraint evaluation\n"
  "# (true indicates state satisfies constraint, false indicates state violates constraint)\n"
  "# if false, distance specifies a measure of the distance of the state from the constraint\n"
  "# if true, distance is set to zero\n"
  "bool result\n"
  "float64 distance";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
moveit_msgs__msg__ConstraintEvalResult__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {moveit_msgs__msg__ConstraintEvalResult__TYPE_NAME, 36, 36},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 356, 356},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
moveit_msgs__msg__ConstraintEvalResult__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *moveit_msgs__msg__ConstraintEvalResult__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
