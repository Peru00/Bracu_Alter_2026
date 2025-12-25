// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from moveit_msgs:msg/OrientationConstraint.idl
// generated code does not contain a copyright notice

#include "moveit_msgs/msg/detail/orientation_constraint__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_moveit_msgs
const rosidl_type_hash_t *
moveit_msgs__msg__OrientationConstraint__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x7a, 0x19, 0x17, 0xac, 0x47, 0x88, 0x6e, 0xb1,
      0xfc, 0x17, 0x03, 0x17, 0x95, 0x21, 0x98, 0xb2,
      0x03, 0xbb, 0x79, 0x7f, 0x07, 0x27, 0x04, 0x75,
      0xa7, 0xbb, 0xaf, 0x69, 0x2c, 0xad, 0x28, 0x71,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "geometry_msgs/msg/detail/quaternion__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "std_msgs/msg/detail/header__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t geometry_msgs__msg__Quaternion__EXPECTED_HASH = {1, {
    0x8a, 0x76, 0x5f, 0x66, 0x77, 0x8c, 0x8f, 0xf7,
    0xc8, 0xab, 0x94, 0xaf, 0xcc, 0x59, 0x0a, 0x2e,
    0xd5, 0x32, 0x5a, 0x1d, 0x9a, 0x07, 0x6f, 0xff,
    0xf3, 0x8f, 0xbc, 0xe3, 0x6f, 0x45, 0x86, 0x84,
  }};
static const rosidl_type_hash_t std_msgs__msg__Header__EXPECTED_HASH = {1, {
    0xf4, 0x9f, 0xb3, 0xae, 0x2c, 0xf0, 0x70, 0xf7,
    0x93, 0x64, 0x5f, 0xf7, 0x49, 0x68, 0x3a, 0xc6,
    0xb0, 0x62, 0x03, 0xe4, 0x1c, 0x89, 0x1e, 0x17,
    0x70, 0x1b, 0x1c, 0xb5, 0x97, 0xce, 0x6a, 0x01,
  }};
#endif

static char moveit_msgs__msg__OrientationConstraint__TYPE_NAME[] = "moveit_msgs/msg/OrientationConstraint";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char geometry_msgs__msg__Quaternion__TYPE_NAME[] = "geometry_msgs/msg/Quaternion";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";

// Define type names, field names, and default values
static char moveit_msgs__msg__OrientationConstraint__FIELD_NAME__header[] = "header";
static char moveit_msgs__msg__OrientationConstraint__FIELD_NAME__orientation[] = "orientation";
static char moveit_msgs__msg__OrientationConstraint__FIELD_NAME__link_name[] = "link_name";
static char moveit_msgs__msg__OrientationConstraint__FIELD_NAME__absolute_x_axis_tolerance[] = "absolute_x_axis_tolerance";
static char moveit_msgs__msg__OrientationConstraint__FIELD_NAME__absolute_y_axis_tolerance[] = "absolute_y_axis_tolerance";
static char moveit_msgs__msg__OrientationConstraint__FIELD_NAME__absolute_z_axis_tolerance[] = "absolute_z_axis_tolerance";
static char moveit_msgs__msg__OrientationConstraint__FIELD_NAME__parameterization[] = "parameterization";
static char moveit_msgs__msg__OrientationConstraint__FIELD_NAME__weight[] = "weight";

static rosidl_runtime_c__type_description__Field moveit_msgs__msg__OrientationConstraint__FIELDS[] = {
  {
    {moveit_msgs__msg__OrientationConstraint__FIELD_NAME__header, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    },
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__msg__OrientationConstraint__FIELD_NAME__orientation, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {geometry_msgs__msg__Quaternion__TYPE_NAME, 28, 28},
    },
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__msg__OrientationConstraint__FIELD_NAME__link_name, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__msg__OrientationConstraint__FIELD_NAME__absolute_x_axis_tolerance, 25, 25},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__msg__OrientationConstraint__FIELD_NAME__absolute_y_axis_tolerance, 25, 25},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__msg__OrientationConstraint__FIELD_NAME__absolute_z_axis_tolerance, 25, 25},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__msg__OrientationConstraint__FIELD_NAME__parameterization, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__msg__OrientationConstraint__FIELD_NAME__weight, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription moveit_msgs__msg__OrientationConstraint__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Quaternion__TYPE_NAME, 28, 28},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
moveit_msgs__msg__OrientationConstraint__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {moveit_msgs__msg__OrientationConstraint__TYPE_NAME, 37, 37},
      {moveit_msgs__msg__OrientationConstraint__FIELDS, 8, 8},
    },
    {moveit_msgs__msg__OrientationConstraint__REFERENCED_TYPE_DESCRIPTIONS, 3, 3},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Quaternion__EXPECTED_HASH, geometry_msgs__msg__Quaternion__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = geometry_msgs__msg__Quaternion__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# This message contains the definition of an orientation constraint.\n"
  "\n"
  "std_msgs/Header header\n"
  "\n"
  "# The desired orientation of the robot link specified as a quaternion\n"
  "geometry_msgs/Quaternion orientation\n"
  "\n"
  "# The robot link this constraint refers to\n"
  "string link_name\n"
  "\n"
  "# Tolerance on the three vector components of the orientation error (optional)\n"
  "# This is a +/- tolerance i.e. (+ x_axis_tol, - x_axis_tol)\n"
  "float64 absolute_x_axis_tolerance\n"
  "float64 absolute_y_axis_tolerance\n"
  "float64 absolute_z_axis_tolerance\n"
  "\n"
  "# Defines how the orientation error is calculated\n"
  "# The error is compared to the tolerance defined above\n"
  "uint8 parameterization\n"
  "\n"
  "# The different options for the orientation error parameterization\n"
  "# - Intrinsic xyz Euler angles (default value)\n"
  "uint8 XYZ_EULER_ANGLES=0\n"
  "# - A rotation vector. This is similar to the angle-axis representation,\n"
  "# but the magnitude of the vector represents the rotation angle.\n"
  "uint8 ROTATION_VECTOR=1\n"
  "\n"
  "# A weighting factor for this constraint (denotes relative importance to other constraints. Closer to zero means less important)\n"
  "float64 weight";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
moveit_msgs__msg__OrientationConstraint__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {moveit_msgs__msg__OrientationConstraint__TYPE_NAME, 37, 37},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 1080, 1080},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
moveit_msgs__msg__OrientationConstraint__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[4];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 4, 4};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *moveit_msgs__msg__OrientationConstraint__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *geometry_msgs__msg__Quaternion__get_individual_type_description_source(NULL);
    sources[3] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
