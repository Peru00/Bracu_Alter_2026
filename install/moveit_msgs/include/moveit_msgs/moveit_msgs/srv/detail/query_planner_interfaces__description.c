// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from moveit_msgs:srv/QueryPlannerInterfaces.idl
// generated code does not contain a copyright notice

#include "moveit_msgs/srv/detail/query_planner_interfaces__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_moveit_msgs
const rosidl_type_hash_t *
moveit_msgs__srv__QueryPlannerInterfaces__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xc7, 0x55, 0x0f, 0xca, 0x36, 0xd2, 0xac, 0x8b,
      0x34, 0x63, 0xb2, 0x5c, 0x72, 0x3b, 0xaa, 0xf7,
      0xec, 0xd3, 0x13, 0xd6, 0x59, 0xd2, 0x3d, 0x51,
      0x9c, 0x02, 0x1b, 0x51, 0x9e, 0x07, 0xb4, 0x31,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_moveit_msgs
const rosidl_type_hash_t *
moveit_msgs__srv__QueryPlannerInterfaces_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x71, 0x6b, 0x54, 0x9d, 0x77, 0x43, 0x09, 0x7c,
      0x7f, 0x87, 0xe1, 0xb6, 0x07, 0x17, 0x06, 0xcd,
      0x3a, 0xef, 0xc0, 0xa2, 0xfc, 0xa5, 0x04, 0x96,
      0x12, 0x6b, 0x2c, 0xe2, 0x2a, 0xa7, 0xab, 0xe7,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_moveit_msgs
const rosidl_type_hash_t *
moveit_msgs__srv__QueryPlannerInterfaces_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x72, 0xa6, 0x0b, 0x95, 0xe3, 0xc8, 0xb5, 0xe0,
      0xcf, 0xad, 0x47, 0x2a, 0xe8, 0x2a, 0xa9, 0x73,
      0x02, 0x0d, 0x32, 0x62, 0xfb, 0x3e, 0x8d, 0x2d,
      0xbc, 0xba, 0x6d, 0x34, 0x35, 0x64, 0x3f, 0x3f,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_moveit_msgs
const rosidl_type_hash_t *
moveit_msgs__srv__QueryPlannerInterfaces_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x56, 0x28, 0x8e, 0x13, 0xcf, 0x68, 0x8b, 0x3c,
      0x11, 0xf9, 0xd2, 0xe0, 0x56, 0xd4, 0x0f, 0x11,
      0xd0, 0x18, 0xd2, 0x67, 0xa6, 0x6c, 0x23, 0xeb,
      0xe6, 0xda, 0x0c, 0xf9, 0x69, 0x32, 0x0b, 0x1e,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "service_msgs/msg/detail/service_event_info__functions.h"
#include "moveit_msgs/msg/detail/planner_interface_description__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t moveit_msgs__msg__PlannerInterfaceDescription__EXPECTED_HASH = {1, {
    0xbe, 0xfe, 0x55, 0xb3, 0x5f, 0x33, 0x7f, 0x0f,
    0xd4, 0xf8, 0x4d, 0xdd, 0xa3, 0xfe, 0x08, 0x69,
    0x60, 0xc4, 0x7e, 0xf4, 0x95, 0xa6, 0xb2, 0xa3,
    0x69, 0x99, 0x1b, 0x37, 0x38, 0xb8, 0xf2, 0x20,
  }};
static const rosidl_type_hash_t service_msgs__msg__ServiceEventInfo__EXPECTED_HASH = {1, {
    0x41, 0xbc, 0xbb, 0xe0, 0x7a, 0x75, 0xc9, 0xb5,
    0x2b, 0xc9, 0x6b, 0xfd, 0x5c, 0x24, 0xd7, 0xf0,
    0xfc, 0x0a, 0x08, 0xc0, 0xcb, 0x79, 0x21, 0xb3,
    0x37, 0x3c, 0x57, 0x32, 0x34, 0x5a, 0x6f, 0x45,
  }};
#endif

static char moveit_msgs__srv__QueryPlannerInterfaces__TYPE_NAME[] = "moveit_msgs/srv/QueryPlannerInterfaces";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char moveit_msgs__msg__PlannerInterfaceDescription__TYPE_NAME[] = "moveit_msgs/msg/PlannerInterfaceDescription";
static char moveit_msgs__srv__QueryPlannerInterfaces_Event__TYPE_NAME[] = "moveit_msgs/srv/QueryPlannerInterfaces_Event";
static char moveit_msgs__srv__QueryPlannerInterfaces_Request__TYPE_NAME[] = "moveit_msgs/srv/QueryPlannerInterfaces_Request";
static char moveit_msgs__srv__QueryPlannerInterfaces_Response__TYPE_NAME[] = "moveit_msgs/srv/QueryPlannerInterfaces_Response";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char moveit_msgs__srv__QueryPlannerInterfaces__FIELD_NAME__request_message[] = "request_message";
static char moveit_msgs__srv__QueryPlannerInterfaces__FIELD_NAME__response_message[] = "response_message";
static char moveit_msgs__srv__QueryPlannerInterfaces__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field moveit_msgs__srv__QueryPlannerInterfaces__FIELDS[] = {
  {
    {moveit_msgs__srv__QueryPlannerInterfaces__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {moveit_msgs__srv__QueryPlannerInterfaces_Request__TYPE_NAME, 46, 46},
    },
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__srv__QueryPlannerInterfaces__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {moveit_msgs__srv__QueryPlannerInterfaces_Response__TYPE_NAME, 47, 47},
    },
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__srv__QueryPlannerInterfaces__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {moveit_msgs__srv__QueryPlannerInterfaces_Event__TYPE_NAME, 44, 44},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription moveit_msgs__srv__QueryPlannerInterfaces__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__msg__PlannerInterfaceDescription__TYPE_NAME, 43, 43},
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__srv__QueryPlannerInterfaces_Event__TYPE_NAME, 44, 44},
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__srv__QueryPlannerInterfaces_Request__TYPE_NAME, 46, 46},
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__srv__QueryPlannerInterfaces_Response__TYPE_NAME, 47, 47},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
moveit_msgs__srv__QueryPlannerInterfaces__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {moveit_msgs__srv__QueryPlannerInterfaces__TYPE_NAME, 38, 38},
      {moveit_msgs__srv__QueryPlannerInterfaces__FIELDS, 3, 3},
    },
    {moveit_msgs__srv__QueryPlannerInterfaces__REFERENCED_TYPE_DESCRIPTIONS, 6, 6},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&moveit_msgs__msg__PlannerInterfaceDescription__EXPECTED_HASH, moveit_msgs__msg__PlannerInterfaceDescription__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = moveit_msgs__msg__PlannerInterfaceDescription__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = moveit_msgs__srv__QueryPlannerInterfaces_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = moveit_msgs__srv__QueryPlannerInterfaces_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[4].fields = moveit_msgs__srv__QueryPlannerInterfaces_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[5].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char moveit_msgs__srv__QueryPlannerInterfaces_Request__FIELD_NAME__structure_needs_at_least_one_member[] = "structure_needs_at_least_one_member";

static rosidl_runtime_c__type_description__Field moveit_msgs__srv__QueryPlannerInterfaces_Request__FIELDS[] = {
  {
    {moveit_msgs__srv__QueryPlannerInterfaces_Request__FIELD_NAME__structure_needs_at_least_one_member, 35, 35},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
moveit_msgs__srv__QueryPlannerInterfaces_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {moveit_msgs__srv__QueryPlannerInterfaces_Request__TYPE_NAME, 46, 46},
      {moveit_msgs__srv__QueryPlannerInterfaces_Request__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char moveit_msgs__srv__QueryPlannerInterfaces_Response__FIELD_NAME__planner_interfaces[] = "planner_interfaces";

static rosidl_runtime_c__type_description__Field moveit_msgs__srv__QueryPlannerInterfaces_Response__FIELDS[] = {
  {
    {moveit_msgs__srv__QueryPlannerInterfaces_Response__FIELD_NAME__planner_interfaces, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {moveit_msgs__msg__PlannerInterfaceDescription__TYPE_NAME, 43, 43},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription moveit_msgs__srv__QueryPlannerInterfaces_Response__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {moveit_msgs__msg__PlannerInterfaceDescription__TYPE_NAME, 43, 43},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
moveit_msgs__srv__QueryPlannerInterfaces_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {moveit_msgs__srv__QueryPlannerInterfaces_Response__TYPE_NAME, 47, 47},
      {moveit_msgs__srv__QueryPlannerInterfaces_Response__FIELDS, 1, 1},
    },
    {moveit_msgs__srv__QueryPlannerInterfaces_Response__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&moveit_msgs__msg__PlannerInterfaceDescription__EXPECTED_HASH, moveit_msgs__msg__PlannerInterfaceDescription__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = moveit_msgs__msg__PlannerInterfaceDescription__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char moveit_msgs__srv__QueryPlannerInterfaces_Event__FIELD_NAME__info[] = "info";
static char moveit_msgs__srv__QueryPlannerInterfaces_Event__FIELD_NAME__request[] = "request";
static char moveit_msgs__srv__QueryPlannerInterfaces_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field moveit_msgs__srv__QueryPlannerInterfaces_Event__FIELDS[] = {
  {
    {moveit_msgs__srv__QueryPlannerInterfaces_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__srv__QueryPlannerInterfaces_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {moveit_msgs__srv__QueryPlannerInterfaces_Request__TYPE_NAME, 46, 46},
    },
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__srv__QueryPlannerInterfaces_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {moveit_msgs__srv__QueryPlannerInterfaces_Response__TYPE_NAME, 47, 47},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription moveit_msgs__srv__QueryPlannerInterfaces_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__msg__PlannerInterfaceDescription__TYPE_NAME, 43, 43},
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__srv__QueryPlannerInterfaces_Request__TYPE_NAME, 46, 46},
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__srv__QueryPlannerInterfaces_Response__TYPE_NAME, 47, 47},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
moveit_msgs__srv__QueryPlannerInterfaces_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {moveit_msgs__srv__QueryPlannerInterfaces_Event__TYPE_NAME, 44, 44},
      {moveit_msgs__srv__QueryPlannerInterfaces_Event__FIELDS, 3, 3},
    },
    {moveit_msgs__srv__QueryPlannerInterfaces_Event__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&moveit_msgs__msg__PlannerInterfaceDescription__EXPECTED_HASH, moveit_msgs__msg__PlannerInterfaceDescription__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = moveit_msgs__msg__PlannerInterfaceDescription__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = moveit_msgs__srv__QueryPlannerInterfaces_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = moveit_msgs__srv__QueryPlannerInterfaces_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "\n"
  "---\n"
  "\n"
  "# The planning instances that could be used in the benchmark\n"
  "PlannerInterfaceDescription[] planner_interfaces";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
moveit_msgs__srv__QueryPlannerInterfaces__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {moveit_msgs__srv__QueryPlannerInterfaces__TYPE_NAME, 38, 38},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 116, 116},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
moveit_msgs__srv__QueryPlannerInterfaces_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {moveit_msgs__srv__QueryPlannerInterfaces_Request__TYPE_NAME, 46, 46},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
moveit_msgs__srv__QueryPlannerInterfaces_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {moveit_msgs__srv__QueryPlannerInterfaces_Response__TYPE_NAME, 47, 47},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
moveit_msgs__srv__QueryPlannerInterfaces_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {moveit_msgs__srv__QueryPlannerInterfaces_Event__TYPE_NAME, 44, 44},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
moveit_msgs__srv__QueryPlannerInterfaces__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[7];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 7, 7};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *moveit_msgs__srv__QueryPlannerInterfaces__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *moveit_msgs__msg__PlannerInterfaceDescription__get_individual_type_description_source(NULL);
    sources[3] = *moveit_msgs__srv__QueryPlannerInterfaces_Event__get_individual_type_description_source(NULL);
    sources[4] = *moveit_msgs__srv__QueryPlannerInterfaces_Request__get_individual_type_description_source(NULL);
    sources[5] = *moveit_msgs__srv__QueryPlannerInterfaces_Response__get_individual_type_description_source(NULL);
    sources[6] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
moveit_msgs__srv__QueryPlannerInterfaces_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *moveit_msgs__srv__QueryPlannerInterfaces_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
moveit_msgs__srv__QueryPlannerInterfaces_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *moveit_msgs__srv__QueryPlannerInterfaces_Response__get_individual_type_description_source(NULL),
    sources[1] = *moveit_msgs__msg__PlannerInterfaceDescription__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
moveit_msgs__srv__QueryPlannerInterfaces_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *moveit_msgs__srv__QueryPlannerInterfaces_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *moveit_msgs__msg__PlannerInterfaceDescription__get_individual_type_description_source(NULL);
    sources[3] = *moveit_msgs__srv__QueryPlannerInterfaces_Request__get_individual_type_description_source(NULL);
    sources[4] = *moveit_msgs__srv__QueryPlannerInterfaces_Response__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
