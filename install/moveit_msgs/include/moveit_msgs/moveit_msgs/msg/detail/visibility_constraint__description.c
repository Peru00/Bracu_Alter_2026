// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from moveit_msgs:msg/VisibilityConstraint.idl
// generated code does not contain a copyright notice

#include "moveit_msgs/msg/detail/visibility_constraint__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_moveit_msgs
const rosidl_type_hash_t *
moveit_msgs__msg__VisibilityConstraint__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x2b, 0x47, 0x99, 0x35, 0x30, 0x7d, 0xe4, 0x21,
      0x62, 0x32, 0x21, 0x52, 0x0e, 0xe5, 0xd9, 0xff,
      0xab, 0x49, 0x4a, 0xf7, 0xf4, 0xb1, 0x94, 0x51,
      0x14, 0xfc, 0x7e, 0x48, 0x31, 0x85, 0x6c, 0x75,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "std_msgs/msg/detail/header__functions.h"
#include "geometry_msgs/msg/detail/pose__functions.h"
#include "geometry_msgs/msg/detail/quaternion__functions.h"
#include "geometry_msgs/msg/detail/pose_stamped__functions.h"
#include "geometry_msgs/msg/detail/point__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t geometry_msgs__msg__Point__EXPECTED_HASH = {1, {
    0x69, 0x63, 0x08, 0x48, 0x42, 0xa9, 0xb0, 0x44,
    0x94, 0xd6, 0xb2, 0x94, 0x1d, 0x11, 0x44, 0x47,
    0x08, 0xd8, 0x92, 0xda, 0x2f, 0x4b, 0x09, 0x84,
    0x3b, 0x9c, 0x43, 0xf4, 0x2a, 0x7f, 0x68, 0x81,
  }};
static const rosidl_type_hash_t geometry_msgs__msg__Pose__EXPECTED_HASH = {1, {
    0xd5, 0x01, 0x95, 0x4e, 0x94, 0x76, 0xce, 0xa2,
    0x99, 0x69, 0x84, 0xe8, 0x12, 0x05, 0x4b, 0x68,
    0x02, 0x6a, 0xe0, 0xbf, 0xae, 0x78, 0x9d, 0x9a,
    0x10, 0xb2, 0x3d, 0xaf, 0x35, 0xcc, 0x90, 0xfa,
  }};
static const rosidl_type_hash_t geometry_msgs__msg__PoseStamped__EXPECTED_HASH = {1, {
    0x10, 0xf3, 0x78, 0x6d, 0x7d, 0x40, 0xfd, 0x2b,
    0x54, 0x36, 0x78, 0x35, 0x61, 0x4b, 0xff, 0x85,
    0xd4, 0xad, 0x3b, 0x5d, 0xab, 0x62, 0xbf, 0x8b,
    0xca, 0x0c, 0xc2, 0x32, 0xd7, 0x3b, 0x4c, 0xd8,
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

static char moveit_msgs__msg__VisibilityConstraint__TYPE_NAME[] = "moveit_msgs/msg/VisibilityConstraint";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char geometry_msgs__msg__Point__TYPE_NAME[] = "geometry_msgs/msg/Point";
static char geometry_msgs__msg__Pose__TYPE_NAME[] = "geometry_msgs/msg/Pose";
static char geometry_msgs__msg__PoseStamped__TYPE_NAME[] = "geometry_msgs/msg/PoseStamped";
static char geometry_msgs__msg__Quaternion__TYPE_NAME[] = "geometry_msgs/msg/Quaternion";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";

// Define type names, field names, and default values
static char moveit_msgs__msg__VisibilityConstraint__FIELD_NAME__target_radius[] = "target_radius";
static char moveit_msgs__msg__VisibilityConstraint__FIELD_NAME__target_pose[] = "target_pose";
static char moveit_msgs__msg__VisibilityConstraint__FIELD_NAME__cone_sides[] = "cone_sides";
static char moveit_msgs__msg__VisibilityConstraint__FIELD_NAME__sensor_pose[] = "sensor_pose";
static char moveit_msgs__msg__VisibilityConstraint__FIELD_NAME__max_view_angle[] = "max_view_angle";
static char moveit_msgs__msg__VisibilityConstraint__FIELD_NAME__max_range_angle[] = "max_range_angle";
static char moveit_msgs__msg__VisibilityConstraint__FIELD_NAME__sensor_view_direction[] = "sensor_view_direction";
static char moveit_msgs__msg__VisibilityConstraint__FIELD_NAME__weight[] = "weight";

static rosidl_runtime_c__type_description__Field moveit_msgs__msg__VisibilityConstraint__FIELDS[] = {
  {
    {moveit_msgs__msg__VisibilityConstraint__FIELD_NAME__target_radius, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__msg__VisibilityConstraint__FIELD_NAME__target_pose, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {geometry_msgs__msg__PoseStamped__TYPE_NAME, 29, 29},
    },
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__msg__VisibilityConstraint__FIELD_NAME__cone_sides, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__msg__VisibilityConstraint__FIELD_NAME__sensor_pose, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {geometry_msgs__msg__PoseStamped__TYPE_NAME, 29, 29},
    },
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__msg__VisibilityConstraint__FIELD_NAME__max_view_angle, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__msg__VisibilityConstraint__FIELD_NAME__max_range_angle, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__msg__VisibilityConstraint__FIELD_NAME__sensor_view_direction, 21, 21},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__msg__VisibilityConstraint__FIELD_NAME__weight, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription moveit_msgs__msg__VisibilityConstraint__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Point__TYPE_NAME, 23, 23},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Pose__TYPE_NAME, 22, 22},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__PoseStamped__TYPE_NAME, 29, 29},
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
moveit_msgs__msg__VisibilityConstraint__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {moveit_msgs__msg__VisibilityConstraint__TYPE_NAME, 36, 36},
      {moveit_msgs__msg__VisibilityConstraint__FIELDS, 8, 8},
    },
    {moveit_msgs__msg__VisibilityConstraint__REFERENCED_TYPE_DESCRIPTIONS, 6, 6},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Point__EXPECTED_HASH, geometry_msgs__msg__Point__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = geometry_msgs__msg__Point__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Pose__EXPECTED_HASH, geometry_msgs__msg__Pose__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = geometry_msgs__msg__Pose__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__PoseStamped__EXPECTED_HASH, geometry_msgs__msg__PoseStamped__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = geometry_msgs__msg__PoseStamped__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Quaternion__EXPECTED_HASH, geometry_msgs__msg__Quaternion__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = geometry_msgs__msg__Quaternion__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[5].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# The constraint is useful to maintain visibility to a disc (the target) in a particular frame.\n"
  "# This disc forms the base of a visibiliy cone whose tip is at the origin of the sensor.\n"
  "# Maintaining visibility is done by ensuring the robot does not obstruct the visibility cone.\n"
  "# Note:\n"
  "# This constraint does NOT enforce minimum or maximum distances between the sensor\n"
  "# and the target, nor does it enforce the target to be in the field of view of\n"
  "# the sensor. A PositionConstraint can (and probably should) be used for such purposes.\n"
  "\n"
  "# The radius of the disc that should be maintained visible\n"
  "float64 target_radius\n"
  "\n"
  "# The pose of the disc; as the robot moves, the pose of the disc may change as well\n"
  "# This can be in the frame of a particular robot link, for example\n"
  "geometry_msgs/PoseStamped target_pose\n"
  "\n"
  "# From the sensor origin towards the target, the disc forms a visibility cone\n"
  "# This cone is approximated using many sides. For example, when using 4 sides,\n"
  "# that in fact makes the visibility region be a pyramid.\n"
  "# This value should always be 3 or more.\n"
  "int32 cone_sides\n"
  "\n"
  "# The pose in which visibility is to be maintained.\n"
  "# The frame id should represent the robot link to which the sensor is attached.\n"
  "# It is assumed the sensor can look directly at the target, in any direction.\n"
  "# This assumption is usually not true, but additional PositionConstraints\n"
  "# can resolve this issue.\n"
  "geometry_msgs/PoseStamped sensor_pose\n"
  "\n"
  "# Even though the disc is maintained visible, the visibility cone can be very small\n"
  "# because of the orientation of the disc with respect to the sensor. It is possible\n"
  "# for example to view the disk almost from a side, in which case the visibility cone\n"
  "# can end up having close to 0 volume. The view angle is defined to be the angle between\n"
  "# the normal to the visibility disc and the direction vector from the sensor origin.\n"
  "# The value below represents the minimum desired view angle. For a perfect view,\n"
  "# this value will be 0 (the two vectors are exact opposites). For a completely obstructed view\n"
  "# this value will be Pi/2 (the vectors are perpendicular). This value defined below\n"
  "# is the maximum view angle to be maintained. This should be a value in the open interval\n"
  "# (0, Pi/2). If 0 is set, the view angle is NOT enforced.\n"
  "float64 max_view_angle\n"
  "\n"
  "# This angle is used similarly to max_view_angle but limits the maximum angle\n"
  "# between the sensor origin direction vector and the axis that connects the\n"
  "# sensor origin to the target frame origin. The value is again in the range (0, Pi/2)\n"
  "# and is NOT enforced if set to 0.\n"
  "float64 max_range_angle\n"
  "\n"
  "# The axis that is assumed to indicate the direction of view for the sensor\n"
  "# X = 2, Y = 1, Z = 0\n"
  "uint8 SENSOR_Z=0\n"
  "uint8 SENSOR_Y=1\n"
  "uint8 SENSOR_X=2\n"
  "uint8 sensor_view_direction\n"
  "\n"
  "# A weighting factor for this constraint (denotes relative importance to other constraints. Closer to zero means less important)\n"
  "float64 weight";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
moveit_msgs__msg__VisibilityConstraint__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {moveit_msgs__msg__VisibilityConstraint__TYPE_NAME, 36, 36},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 2913, 2913},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
moveit_msgs__msg__VisibilityConstraint__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[7];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 7, 7};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *moveit_msgs__msg__VisibilityConstraint__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *geometry_msgs__msg__Point__get_individual_type_description_source(NULL);
    sources[3] = *geometry_msgs__msg__Pose__get_individual_type_description_source(NULL);
    sources[4] = *geometry_msgs__msg__PoseStamped__get_individual_type_description_source(NULL);
    sources[5] = *geometry_msgs__msg__Quaternion__get_individual_type_description_source(NULL);
    sources[6] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
