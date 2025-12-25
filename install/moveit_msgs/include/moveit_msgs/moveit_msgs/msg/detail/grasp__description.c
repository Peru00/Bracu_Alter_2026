// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from moveit_msgs:msg/Grasp.idl
// generated code does not contain a copyright notice

#include "moveit_msgs/msg/detail/grasp__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_moveit_msgs
const rosidl_type_hash_t *
moveit_msgs__msg__Grasp__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x53, 0x69, 0x66, 0x9d, 0xe8, 0x8f, 0xe8, 0x5f,
      0x7b, 0xca, 0x5d, 0xc9, 0x48, 0xd7, 0x0f, 0xe6,
      0xe8, 0x6f, 0xe9, 0x3c, 0x53, 0x1d, 0x59, 0x0e,
      0xcb, 0xcd, 0x9c, 0x21, 0x38, 0x5a, 0xa5, 0x93,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "builtin_interfaces/msg/detail/duration__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "std_msgs/msg/detail/header__functions.h"
#include "geometry_msgs/msg/detail/vector3_stamped__functions.h"
#include "moveit_msgs/msg/detail/gripper_translation__functions.h"
#include "trajectory_msgs/msg/detail/joint_trajectory_point__functions.h"
#include "geometry_msgs/msg/detail/pose__functions.h"
#include "trajectory_msgs/msg/detail/joint_trajectory__functions.h"
#include "geometry_msgs/msg/detail/quaternion__functions.h"
#include "geometry_msgs/msg/detail/pose_stamped__functions.h"
#include "geometry_msgs/msg/detail/point__functions.h"
#include "geometry_msgs/msg/detail/vector3__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Duration__EXPECTED_HASH = {1, {
    0xe8, 0xd0, 0x09, 0xf6, 0x59, 0x81, 0x6f, 0x75,
    0x8b, 0x75, 0x33, 0x4e, 0xe1, 0xa9, 0xca, 0x5b,
    0x5c, 0x0b, 0x85, 0x98, 0x43, 0x26, 0x1f, 0x14,
    0xc7, 0xf9, 0x37, 0x34, 0x95, 0x99, 0xd9, 0x3b,
  }};
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
static const rosidl_type_hash_t geometry_msgs__msg__Vector3__EXPECTED_HASH = {1, {
    0xcc, 0x12, 0xfe, 0x83, 0xe4, 0xc0, 0x27, 0x19,
    0xf1, 0xce, 0x80, 0x70, 0xbf, 0xd1, 0x4a, 0xec,
    0xd4, 0x0f, 0x75, 0xa9, 0x66, 0x96, 0xa6, 0x7a,
    0x2a, 0x1f, 0x37, 0xf7, 0xdb, 0xb0, 0x76, 0x5d,
  }};
static const rosidl_type_hash_t geometry_msgs__msg__Vector3Stamped__EXPECTED_HASH = {1, {
    0xd4, 0x82, 0x96, 0x22, 0x28, 0x8c, 0xbb, 0x44,
    0x38, 0x86, 0xe7, 0xea, 0x94, 0xea, 0x56, 0x71,
    0xa3, 0xb1, 0xbe, 0x6b, 0xab, 0x4a, 0xd0, 0x42,
    0x24, 0x43, 0x2a, 0x65, 0xf7, 0xd7, 0x88, 0x7a,
  }};
static const rosidl_type_hash_t moveit_msgs__msg__GripperTranslation__EXPECTED_HASH = {1, {
    0x54, 0xe1, 0x47, 0x21, 0x38, 0x6d, 0xe9, 0xac,
    0x9e, 0x09, 0xdc, 0x6a, 0x56, 0xdc, 0x9e, 0x8a,
    0xe9, 0xd5, 0x32, 0x7e, 0x99, 0xce, 0xba, 0xeb,
    0xa4, 0x43, 0xe7, 0xf8, 0x80, 0x6d, 0x17, 0x90,
  }};
static const rosidl_type_hash_t std_msgs__msg__Header__EXPECTED_HASH = {1, {
    0xf4, 0x9f, 0xb3, 0xae, 0x2c, 0xf0, 0x70, 0xf7,
    0x93, 0x64, 0x5f, 0xf7, 0x49, 0x68, 0x3a, 0xc6,
    0xb0, 0x62, 0x03, 0xe4, 0x1c, 0x89, 0x1e, 0x17,
    0x70, 0x1b, 0x1c, 0xb5, 0x97, 0xce, 0x6a, 0x01,
  }};
static const rosidl_type_hash_t trajectory_msgs__msg__JointTrajectory__EXPECTED_HASH = {1, {
    0x17, 0x9b, 0x33, 0xeb, 0xa5, 0x9d, 0x67, 0x6f,
    0x6d, 0x96, 0x7a, 0xc7, 0x1f, 0xe3, 0x5e, 0x7c,
    0xa2, 0xf6, 0x4b, 0x2f, 0x39, 0x28, 0xf4, 0xa0,
    0x18, 0xce, 0xc1, 0x15, 0xe2, 0x13, 0x79, 0x6e,
  }};
static const rosidl_type_hash_t trajectory_msgs__msg__JointTrajectoryPoint__EXPECTED_HASH = {1, {
    0xde, 0x89, 0x07, 0x03, 0x6d, 0x8b, 0xd4, 0x5a,
    0xac, 0x6f, 0x30, 0xcc, 0x90, 0x44, 0xa3, 0xd4,
    0xa3, 0x29, 0xc4, 0x2c, 0xbf, 0x71, 0x9a, 0xff,
    0x7d, 0x95, 0xa5, 0x84, 0xcf, 0xa5, 0x32, 0xd7,
  }};
#endif

static char moveit_msgs__msg__Grasp__TYPE_NAME[] = "moveit_msgs/msg/Grasp";
static char builtin_interfaces__msg__Duration__TYPE_NAME[] = "builtin_interfaces/msg/Duration";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char geometry_msgs__msg__Point__TYPE_NAME[] = "geometry_msgs/msg/Point";
static char geometry_msgs__msg__Pose__TYPE_NAME[] = "geometry_msgs/msg/Pose";
static char geometry_msgs__msg__PoseStamped__TYPE_NAME[] = "geometry_msgs/msg/PoseStamped";
static char geometry_msgs__msg__Quaternion__TYPE_NAME[] = "geometry_msgs/msg/Quaternion";
static char geometry_msgs__msg__Vector3__TYPE_NAME[] = "geometry_msgs/msg/Vector3";
static char geometry_msgs__msg__Vector3Stamped__TYPE_NAME[] = "geometry_msgs/msg/Vector3Stamped";
static char moveit_msgs__msg__GripperTranslation__TYPE_NAME[] = "moveit_msgs/msg/GripperTranslation";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";
static char trajectory_msgs__msg__JointTrajectory__TYPE_NAME[] = "trajectory_msgs/msg/JointTrajectory";
static char trajectory_msgs__msg__JointTrajectoryPoint__TYPE_NAME[] = "trajectory_msgs/msg/JointTrajectoryPoint";

// Define type names, field names, and default values
static char moveit_msgs__msg__Grasp__FIELD_NAME__id[] = "id";
static char moveit_msgs__msg__Grasp__FIELD_NAME__pre_grasp_posture[] = "pre_grasp_posture";
static char moveit_msgs__msg__Grasp__FIELD_NAME__grasp_posture[] = "grasp_posture";
static char moveit_msgs__msg__Grasp__FIELD_NAME__grasp_pose[] = "grasp_pose";
static char moveit_msgs__msg__Grasp__FIELD_NAME__grasp_quality[] = "grasp_quality";
static char moveit_msgs__msg__Grasp__FIELD_NAME__pre_grasp_approach[] = "pre_grasp_approach";
static char moveit_msgs__msg__Grasp__FIELD_NAME__post_grasp_retreat[] = "post_grasp_retreat";
static char moveit_msgs__msg__Grasp__FIELD_NAME__post_place_retreat[] = "post_place_retreat";
static char moveit_msgs__msg__Grasp__FIELD_NAME__max_contact_force[] = "max_contact_force";
static char moveit_msgs__msg__Grasp__FIELD_NAME__allowed_touch_objects[] = "allowed_touch_objects";

static rosidl_runtime_c__type_description__Field moveit_msgs__msg__Grasp__FIELDS[] = {
  {
    {moveit_msgs__msg__Grasp__FIELD_NAME__id, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__msg__Grasp__FIELD_NAME__pre_grasp_posture, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {trajectory_msgs__msg__JointTrajectory__TYPE_NAME, 35, 35},
    },
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__msg__Grasp__FIELD_NAME__grasp_posture, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {trajectory_msgs__msg__JointTrajectory__TYPE_NAME, 35, 35},
    },
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__msg__Grasp__FIELD_NAME__grasp_pose, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {geometry_msgs__msg__PoseStamped__TYPE_NAME, 29, 29},
    },
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__msg__Grasp__FIELD_NAME__grasp_quality, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__msg__Grasp__FIELD_NAME__pre_grasp_approach, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {moveit_msgs__msg__GripperTranslation__TYPE_NAME, 34, 34},
    },
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__msg__Grasp__FIELD_NAME__post_grasp_retreat, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {moveit_msgs__msg__GripperTranslation__TYPE_NAME, 34, 34},
    },
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__msg__Grasp__FIELD_NAME__post_place_retreat, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {moveit_msgs__msg__GripperTranslation__TYPE_NAME, 34, 34},
    },
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__msg__Grasp__FIELD_NAME__max_contact_force, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__msg__Grasp__FIELD_NAME__allowed_touch_objects, 21, 21},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription moveit_msgs__msg__Grasp__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Duration__TYPE_NAME, 31, 31},
    {NULL, 0, 0},
  },
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
    {geometry_msgs__msg__Vector3__TYPE_NAME, 25, 25},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Vector3Stamped__TYPE_NAME, 32, 32},
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__msg__GripperTranslation__TYPE_NAME, 34, 34},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
  {
    {trajectory_msgs__msg__JointTrajectory__TYPE_NAME, 35, 35},
    {NULL, 0, 0},
  },
  {
    {trajectory_msgs__msg__JointTrajectoryPoint__TYPE_NAME, 40, 40},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
moveit_msgs__msg__Grasp__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {moveit_msgs__msg__Grasp__TYPE_NAME, 21, 21},
      {moveit_msgs__msg__Grasp__FIELDS, 10, 10},
    },
    {moveit_msgs__msg__Grasp__REFERENCED_TYPE_DESCRIPTIONS, 12, 12},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Duration__EXPECTED_HASH, builtin_interfaces__msg__Duration__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Duration__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Point__EXPECTED_HASH, geometry_msgs__msg__Point__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = geometry_msgs__msg__Point__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Pose__EXPECTED_HASH, geometry_msgs__msg__Pose__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = geometry_msgs__msg__Pose__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__PoseStamped__EXPECTED_HASH, geometry_msgs__msg__PoseStamped__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = geometry_msgs__msg__PoseStamped__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Quaternion__EXPECTED_HASH, geometry_msgs__msg__Quaternion__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[5].fields = geometry_msgs__msg__Quaternion__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Vector3__EXPECTED_HASH, geometry_msgs__msg__Vector3__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[6].fields = geometry_msgs__msg__Vector3__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Vector3Stamped__EXPECTED_HASH, geometry_msgs__msg__Vector3Stamped__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[7].fields = geometry_msgs__msg__Vector3Stamped__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&moveit_msgs__msg__GripperTranslation__EXPECTED_HASH, moveit_msgs__msg__GripperTranslation__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[8].fields = moveit_msgs__msg__GripperTranslation__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[9].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&trajectory_msgs__msg__JointTrajectory__EXPECTED_HASH, trajectory_msgs__msg__JointTrajectory__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[10].fields = trajectory_msgs__msg__JointTrajectory__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&trajectory_msgs__msg__JointTrajectoryPoint__EXPECTED_HASH, trajectory_msgs__msg__JointTrajectoryPoint__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[11].fields = trajectory_msgs__msg__JointTrajectoryPoint__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# This message contains a description of a grasp that would be used\n"
  "# with a particular end-effector to grasp an object, including how to\n"
  "# approach it, grip it, etc.  This message does not contain any\n"
  "# information about a \"grasp point\" (a position ON the object).\n"
  "# Whatever generates this message should have already combined\n"
  "# information about grasp points with information about the geometry\n"
  "# of the end-effector to compute the grasp_pose in this message.\n"
  "\n"
  "# A name for this grasp\n"
  "string id\n"
  "\n"
  "# The internal posture of the hand for the pre-grasp\n"
  "# only positions are used\n"
  "trajectory_msgs/JointTrajectory pre_grasp_posture\n"
  "\n"
  "# The internal posture of the hand for the grasp\n"
  "# positions and efforts are used\n"
  "trajectory_msgs/JointTrajectory grasp_posture\n"
  "\n"
  "# The position of the end-effector for the grasp.  This is the pose of\n"
  "# the \"parent_link\" of the end-effector, not actually the pose of any\n"
  "# link *in* the end-effector.  Typically this would be the pose of the\n"
  "# most distal wrist link before the hand (end-effector) links began.\n"
  "geometry_msgs/PoseStamped grasp_pose\n"
  "\n"
  "# The estimated probability of success for this grasp, or some other\n"
  "# measure of how \"good\" it is.\n"
  "float64 grasp_quality\n"
  "\n"
  "# The approach direction to take before picking an object\n"
  "GripperTranslation pre_grasp_approach\n"
  "\n"
  "# The retreat direction to take after a grasp has been completed (object is attached)\n"
  "GripperTranslation post_grasp_retreat\n"
  "\n"
  "# The retreat motion to perform when releasing the object; this information\n"
  "# is not necessary for the grasp itself, but when releasing the object,\n"
  "# the information will be necessary. The grasp used to perform a pickup\n"
  "# is returned as part of the result, so this information is available for\n"
  "# later use.\n"
  "GripperTranslation post_place_retreat\n"
  "\n"
  "# the maximum contact force to use while grasping (<=0 to disable)\n"
  "float32 max_contact_force\n"
  "\n"
  "# an optional list of obstacles that we have semantic information about\n"
  "# and that can be touched/pushed/moved in the course of grasping\n"
  "string[] allowed_touch_objects";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
moveit_msgs__msg__Grasp__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {moveit_msgs__msg__Grasp__TYPE_NAME, 21, 21},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 2030, 2030},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
moveit_msgs__msg__Grasp__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[13];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 13, 13};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *moveit_msgs__msg__Grasp__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Duration__get_individual_type_description_source(NULL);
    sources[2] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[3] = *geometry_msgs__msg__Point__get_individual_type_description_source(NULL);
    sources[4] = *geometry_msgs__msg__Pose__get_individual_type_description_source(NULL);
    sources[5] = *geometry_msgs__msg__PoseStamped__get_individual_type_description_source(NULL);
    sources[6] = *geometry_msgs__msg__Quaternion__get_individual_type_description_source(NULL);
    sources[7] = *geometry_msgs__msg__Vector3__get_individual_type_description_source(NULL);
    sources[8] = *geometry_msgs__msg__Vector3Stamped__get_individual_type_description_source(NULL);
    sources[9] = *moveit_msgs__msg__GripperTranslation__get_individual_type_description_source(NULL);
    sources[10] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    sources[11] = *trajectory_msgs__msg__JointTrajectory__get_individual_type_description_source(NULL);
    sources[12] = *trajectory_msgs__msg__JointTrajectoryPoint__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
