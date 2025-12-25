// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from moveit_msgs:msg/PlanningSceneWorld.idl
// generated code does not contain a copyright notice

#include "moveit_msgs/msg/detail/planning_scene_world__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_moveit_msgs
const rosidl_type_hash_t *
moveit_msgs__msg__PlanningSceneWorld__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xe0, 0x8e, 0x6a, 0x4a, 0x81, 0xca, 0x6b, 0x6a,
      0x69, 0x53, 0xac, 0x75, 0x8d, 0xad, 0xc2, 0xc9,
      0x49, 0x55, 0x50, 0x93, 0xef, 0xea, 0x6a, 0x9a,
      0x22, 0xd9, 0xac, 0xcf, 0x62, 0xc2, 0xc0, 0x99,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "std_msgs/msg/detail/header__functions.h"
#include "shape_msgs/msg/detail/plane__functions.h"
#include "object_recognition_msgs/msg/detail/object_type__functions.h"
#include "shape_msgs/msg/detail/mesh__functions.h"
#include "geometry_msgs/msg/detail/point32__functions.h"
#include "geometry_msgs/msg/detail/pose__functions.h"
#include "shape_msgs/msg/detail/solid_primitive__functions.h"
#include "moveit_msgs/msg/detail/collision_object__functions.h"
#include "geometry_msgs/msg/detail/quaternion__functions.h"
#include "octomap_msgs/msg/detail/octomap_with_pose__functions.h"
#include "octomap_msgs/msg/detail/octomap__functions.h"
#include "shape_msgs/msg/detail/mesh_triangle__functions.h"
#include "geometry_msgs/msg/detail/polygon__functions.h"
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
static const rosidl_type_hash_t geometry_msgs__msg__Point32__EXPECTED_HASH = {1, {
    0x2f, 0xc4, 0xdb, 0x7c, 0xae, 0x16, 0xa4, 0x58,
    0x2c, 0x79, 0xa5, 0x6b, 0x66, 0x17, 0x3a, 0x8d,
    0x48, 0xd5, 0x2c, 0x7d, 0xc5, 0x20, 0xdd, 0xc5,
    0x5a, 0x0d, 0x4b, 0xcf, 0x2a, 0x4b, 0xfd, 0xbc,
  }};
static const rosidl_type_hash_t geometry_msgs__msg__Polygon__EXPECTED_HASH = {1, {
    0x37, 0x82, 0xf9, 0xf0, 0xbf, 0x04, 0x49, 0x64,
    0xd6, 0x92, 0xd6, 0xc0, 0x17, 0xd7, 0x05, 0xe3,
    0x76, 0x11, 0xaf, 0xb1, 0xf0, 0xbf, 0x6a, 0x9d,
    0xee, 0x24, 0x8a, 0x7d, 0xda, 0x0f, 0x78, 0x4a,
  }};
static const rosidl_type_hash_t geometry_msgs__msg__Pose__EXPECTED_HASH = {1, {
    0xd5, 0x01, 0x95, 0x4e, 0x94, 0x76, 0xce, 0xa2,
    0x99, 0x69, 0x84, 0xe8, 0x12, 0x05, 0x4b, 0x68,
    0x02, 0x6a, 0xe0, 0xbf, 0xae, 0x78, 0x9d, 0x9a,
    0x10, 0xb2, 0x3d, 0xaf, 0x35, 0xcc, 0x90, 0xfa,
  }};
static const rosidl_type_hash_t geometry_msgs__msg__Quaternion__EXPECTED_HASH = {1, {
    0x8a, 0x76, 0x5f, 0x66, 0x77, 0x8c, 0x8f, 0xf7,
    0xc8, 0xab, 0x94, 0xaf, 0xcc, 0x59, 0x0a, 0x2e,
    0xd5, 0x32, 0x5a, 0x1d, 0x9a, 0x07, 0x6f, 0xff,
    0xf3, 0x8f, 0xbc, 0xe3, 0x6f, 0x45, 0x86, 0x84,
  }};
static const rosidl_type_hash_t moveit_msgs__msg__CollisionObject__EXPECTED_HASH = {1, {
    0x74, 0xb0, 0xac, 0xd6, 0xd5, 0x6b, 0xe0, 0x32,
    0xbf, 0x4f, 0x76, 0x4b, 0x2c, 0x26, 0x33, 0xf5,
    0x05, 0x88, 0xb5, 0xfb, 0x2b, 0xd7, 0x1e, 0x97,
    0x20, 0x6c, 0x59, 0x05, 0x27, 0x47, 0xcf, 0x7e,
  }};
static const rosidl_type_hash_t object_recognition_msgs__msg__ObjectType__EXPECTED_HASH = {1, {
    0x5e, 0xb3, 0xee, 0x39, 0x76, 0x40, 0x33, 0x66,
    0xd7, 0xc5, 0x0d, 0x7d, 0xbe, 0xe8, 0xbe, 0xc9,
    0x6b, 0x3b, 0xce, 0xd1, 0xe7, 0x52, 0x67, 0x4a,
    0x71, 0x94, 0xa3, 0x58, 0x41, 0x63, 0x21, 0x34,
  }};
static const rosidl_type_hash_t octomap_msgs__msg__Octomap__EXPECTED_HASH = {1, {
    0x98, 0x91, 0xa1, 0x9a, 0x05, 0xb7, 0x0e, 0xc2,
    0x0a, 0xa8, 0xf4, 0x5d, 0xb3, 0xf8, 0x30, 0x0b,
    0x9a, 0xb8, 0x0a, 0xff, 0xf3, 0x2c, 0x61, 0xc5,
    0x49, 0xd2, 0x50, 0x87, 0xda, 0x96, 0x25, 0xb0,
  }};
static const rosidl_type_hash_t octomap_msgs__msg__OctomapWithPose__EXPECTED_HASH = {1, {
    0xe6, 0x5f, 0x87, 0x60, 0xaa, 0x96, 0xa2, 0xac,
    0xef, 0x86, 0xb6, 0xd2, 0x06, 0x17, 0x5c, 0xfc,
    0x78, 0x42, 0x00, 0x82, 0xb6, 0x15, 0x9d, 0xd3,
    0xfa, 0x6e, 0xbf, 0x93, 0xf5, 0xf8, 0x19, 0x79,
  }};
static const rosidl_type_hash_t shape_msgs__msg__Mesh__EXPECTED_HASH = {1, {
    0xf2, 0x15, 0x0b, 0x82, 0xd8, 0xee, 0x7e, 0x8b,
    0xc3, 0xf3, 0x96, 0xa2, 0xb1, 0x58, 0xae, 0xfb,
    0x4b, 0x9a, 0x55, 0x10, 0xa4, 0x74, 0xbe, 0x27,
    0x1b, 0xa1, 0x26, 0x8a, 0xeb, 0xb5, 0x52, 0x89,
  }};
static const rosidl_type_hash_t shape_msgs__msg__MeshTriangle__EXPECTED_HASH = {1, {
    0x61, 0x8e, 0x5c, 0x07, 0x3e, 0xeb, 0x72, 0x9e,
    0x43, 0x3e, 0xf6, 0x22, 0x6e, 0x72, 0xc0, 0x1d,
    0x99, 0x5c, 0x45, 0x9f, 0xb7, 0xd7, 0x63, 0x48,
    0xc9, 0x70, 0x04, 0x09, 0xa5, 0x02, 0x0b, 0xd0,
  }};
static const rosidl_type_hash_t shape_msgs__msg__Plane__EXPECTED_HASH = {1, {
    0xdf, 0xbf, 0xe8, 0x31, 0x46, 0x89, 0xc8, 0x50,
    0x61, 0x5d, 0x4a, 0x72, 0x7a, 0xf0, 0x17, 0xe9,
    0xaa, 0x86, 0xc1, 0x0e, 0x36, 0x9a, 0x60, 0x6c,
    0x8c, 0x85, 0x1e, 0xf8, 0xf1, 0x6c, 0x58, 0xc8,
  }};
static const rosidl_type_hash_t shape_msgs__msg__SolidPrimitive__EXPECTED_HASH = {1, {
    0x28, 0x02, 0xa1, 0x51, 0x90, 0xaa, 0xdc, 0x3f,
    0x49, 0x65, 0x84, 0xdf, 0x4b, 0x0b, 0x4c, 0x58,
    0x24, 0xd8, 0xa0, 0xb3, 0x1a, 0xae, 0xf8, 0x39,
    0xfa, 0xa7, 0x5b, 0xc3, 0x4d, 0xda, 0x38, 0xac,
  }};
static const rosidl_type_hash_t std_msgs__msg__Header__EXPECTED_HASH = {1, {
    0xf4, 0x9f, 0xb3, 0xae, 0x2c, 0xf0, 0x70, 0xf7,
    0x93, 0x64, 0x5f, 0xf7, 0x49, 0x68, 0x3a, 0xc6,
    0xb0, 0x62, 0x03, 0xe4, 0x1c, 0x89, 0x1e, 0x17,
    0x70, 0x1b, 0x1c, 0xb5, 0x97, 0xce, 0x6a, 0x01,
  }};
#endif

static char moveit_msgs__msg__PlanningSceneWorld__TYPE_NAME[] = "moveit_msgs/msg/PlanningSceneWorld";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char geometry_msgs__msg__Point__TYPE_NAME[] = "geometry_msgs/msg/Point";
static char geometry_msgs__msg__Point32__TYPE_NAME[] = "geometry_msgs/msg/Point32";
static char geometry_msgs__msg__Polygon__TYPE_NAME[] = "geometry_msgs/msg/Polygon";
static char geometry_msgs__msg__Pose__TYPE_NAME[] = "geometry_msgs/msg/Pose";
static char geometry_msgs__msg__Quaternion__TYPE_NAME[] = "geometry_msgs/msg/Quaternion";
static char moveit_msgs__msg__CollisionObject__TYPE_NAME[] = "moveit_msgs/msg/CollisionObject";
static char object_recognition_msgs__msg__ObjectType__TYPE_NAME[] = "object_recognition_msgs/msg/ObjectType";
static char octomap_msgs__msg__Octomap__TYPE_NAME[] = "octomap_msgs/msg/Octomap";
static char octomap_msgs__msg__OctomapWithPose__TYPE_NAME[] = "octomap_msgs/msg/OctomapWithPose";
static char shape_msgs__msg__Mesh__TYPE_NAME[] = "shape_msgs/msg/Mesh";
static char shape_msgs__msg__MeshTriangle__TYPE_NAME[] = "shape_msgs/msg/MeshTriangle";
static char shape_msgs__msg__Plane__TYPE_NAME[] = "shape_msgs/msg/Plane";
static char shape_msgs__msg__SolidPrimitive__TYPE_NAME[] = "shape_msgs/msg/SolidPrimitive";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";

// Define type names, field names, and default values
static char moveit_msgs__msg__PlanningSceneWorld__FIELD_NAME__collision_objects[] = "collision_objects";
static char moveit_msgs__msg__PlanningSceneWorld__FIELD_NAME__octomap[] = "octomap";

static rosidl_runtime_c__type_description__Field moveit_msgs__msg__PlanningSceneWorld__FIELDS[] = {
  {
    {moveit_msgs__msg__PlanningSceneWorld__FIELD_NAME__collision_objects, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {moveit_msgs__msg__CollisionObject__TYPE_NAME, 31, 31},
    },
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__msg__PlanningSceneWorld__FIELD_NAME__octomap, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {octomap_msgs__msg__OctomapWithPose__TYPE_NAME, 32, 32},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription moveit_msgs__msg__PlanningSceneWorld__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Point__TYPE_NAME, 23, 23},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Point32__TYPE_NAME, 25, 25},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Polygon__TYPE_NAME, 25, 25},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Pose__TYPE_NAME, 22, 22},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Quaternion__TYPE_NAME, 28, 28},
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__msg__CollisionObject__TYPE_NAME, 31, 31},
    {NULL, 0, 0},
  },
  {
    {object_recognition_msgs__msg__ObjectType__TYPE_NAME, 38, 38},
    {NULL, 0, 0},
  },
  {
    {octomap_msgs__msg__Octomap__TYPE_NAME, 24, 24},
    {NULL, 0, 0},
  },
  {
    {octomap_msgs__msg__OctomapWithPose__TYPE_NAME, 32, 32},
    {NULL, 0, 0},
  },
  {
    {shape_msgs__msg__Mesh__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
  {
    {shape_msgs__msg__MeshTriangle__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {shape_msgs__msg__Plane__TYPE_NAME, 20, 20},
    {NULL, 0, 0},
  },
  {
    {shape_msgs__msg__SolidPrimitive__TYPE_NAME, 29, 29},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
moveit_msgs__msg__PlanningSceneWorld__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {moveit_msgs__msg__PlanningSceneWorld__TYPE_NAME, 34, 34},
      {moveit_msgs__msg__PlanningSceneWorld__FIELDS, 2, 2},
    },
    {moveit_msgs__msg__PlanningSceneWorld__REFERENCED_TYPE_DESCRIPTIONS, 15, 15},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Point__EXPECTED_HASH, geometry_msgs__msg__Point__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = geometry_msgs__msg__Point__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Point32__EXPECTED_HASH, geometry_msgs__msg__Point32__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = geometry_msgs__msg__Point32__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Polygon__EXPECTED_HASH, geometry_msgs__msg__Polygon__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = geometry_msgs__msg__Polygon__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Pose__EXPECTED_HASH, geometry_msgs__msg__Pose__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = geometry_msgs__msg__Pose__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Quaternion__EXPECTED_HASH, geometry_msgs__msg__Quaternion__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[5].fields = geometry_msgs__msg__Quaternion__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&moveit_msgs__msg__CollisionObject__EXPECTED_HASH, moveit_msgs__msg__CollisionObject__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[6].fields = moveit_msgs__msg__CollisionObject__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&object_recognition_msgs__msg__ObjectType__EXPECTED_HASH, object_recognition_msgs__msg__ObjectType__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[7].fields = object_recognition_msgs__msg__ObjectType__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&octomap_msgs__msg__Octomap__EXPECTED_HASH, octomap_msgs__msg__Octomap__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[8].fields = octomap_msgs__msg__Octomap__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&octomap_msgs__msg__OctomapWithPose__EXPECTED_HASH, octomap_msgs__msg__OctomapWithPose__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[9].fields = octomap_msgs__msg__OctomapWithPose__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&shape_msgs__msg__Mesh__EXPECTED_HASH, shape_msgs__msg__Mesh__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[10].fields = shape_msgs__msg__Mesh__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&shape_msgs__msg__MeshTriangle__EXPECTED_HASH, shape_msgs__msg__MeshTriangle__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[11].fields = shape_msgs__msg__MeshTriangle__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&shape_msgs__msg__Plane__EXPECTED_HASH, shape_msgs__msg__Plane__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[12].fields = shape_msgs__msg__Plane__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&shape_msgs__msg__SolidPrimitive__EXPECTED_HASH, shape_msgs__msg__SolidPrimitive__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[13].fields = shape_msgs__msg__SolidPrimitive__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[14].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# collision objects\n"
  "CollisionObject[] collision_objects\n"
  "\n"
  "# The octomap that represents additional collision data\n"
  "octomap_msgs/OctomapWithPose octomap";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
moveit_msgs__msg__PlanningSceneWorld__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {moveit_msgs__msg__PlanningSceneWorld__TYPE_NAME, 34, 34},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 150, 150},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
moveit_msgs__msg__PlanningSceneWorld__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[16];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 16, 16};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *moveit_msgs__msg__PlanningSceneWorld__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *geometry_msgs__msg__Point__get_individual_type_description_source(NULL);
    sources[3] = *geometry_msgs__msg__Point32__get_individual_type_description_source(NULL);
    sources[4] = *geometry_msgs__msg__Polygon__get_individual_type_description_source(NULL);
    sources[5] = *geometry_msgs__msg__Pose__get_individual_type_description_source(NULL);
    sources[6] = *geometry_msgs__msg__Quaternion__get_individual_type_description_source(NULL);
    sources[7] = *moveit_msgs__msg__CollisionObject__get_individual_type_description_source(NULL);
    sources[8] = *object_recognition_msgs__msg__ObjectType__get_individual_type_description_source(NULL);
    sources[9] = *octomap_msgs__msg__Octomap__get_individual_type_description_source(NULL);
    sources[10] = *octomap_msgs__msg__OctomapWithPose__get_individual_type_description_source(NULL);
    sources[11] = *shape_msgs__msg__Mesh__get_individual_type_description_source(NULL);
    sources[12] = *shape_msgs__msg__MeshTriangle__get_individual_type_description_source(NULL);
    sources[13] = *shape_msgs__msg__Plane__get_individual_type_description_source(NULL);
    sources[14] = *shape_msgs__msg__SolidPrimitive__get_individual_type_description_source(NULL);
    sources[15] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
