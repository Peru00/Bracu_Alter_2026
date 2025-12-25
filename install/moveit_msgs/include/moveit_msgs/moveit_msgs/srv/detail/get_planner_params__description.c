// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from moveit_msgs:srv/GetPlannerParams.idl
// generated code does not contain a copyright notice

#include "moveit_msgs/srv/detail/get_planner_params__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_moveit_msgs
const rosidl_type_hash_t *
moveit_msgs__srv__GetPlannerParams__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x77, 0x02, 0x99, 0x09, 0x1d, 0x17, 0xec, 0x6f,
      0x49, 0x84, 0x0b, 0x0c, 0x5f, 0x9f, 0xee, 0x51,
      0xa5, 0x15, 0x08, 0xc0, 0xd5, 0xc4, 0xb5, 0xb5,
      0x61, 0xec, 0x3b, 0x87, 0x8f, 0xbb, 0xfa, 0x42,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_moveit_msgs
const rosidl_type_hash_t *
moveit_msgs__srv__GetPlannerParams_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xd5, 0xc5, 0xaa, 0x54, 0x79, 0x67, 0xf1, 0x4f,
      0x3f, 0xf0, 0xdd, 0x58, 0xa0, 0xd6, 0x8b, 0xab,
      0x7f, 0xd2, 0xcf, 0x20, 0xa1, 0xc2, 0xe7, 0xa8,
      0x3c, 0x45, 0x0d, 0x0c, 0x8b, 0x10, 0xc9, 0x95,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_moveit_msgs
const rosidl_type_hash_t *
moveit_msgs__srv__GetPlannerParams_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x92, 0x1f, 0xb4, 0x30, 0x66, 0x0c, 0xe8, 0x92,
      0x94, 0x87, 0x37, 0xa0, 0x28, 0xe2, 0x39, 0x27,
      0x39, 0xf9, 0xdb, 0x36, 0x5e, 0xd8, 0x8e, 0xaa,
      0xda, 0x0c, 0x93, 0x12, 0x4e, 0x16, 0xf3, 0xb1,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_moveit_msgs
const rosidl_type_hash_t *
moveit_msgs__srv__GetPlannerParams_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x9e, 0x3c, 0x51, 0xc4, 0x3e, 0x5f, 0xeb, 0x84,
      0x1d, 0x22, 0x52, 0x57, 0xb1, 0x89, 0x45, 0xad,
      0x91, 0xd4, 0xef, 0xe6, 0x8b, 0x30, 0xb7, 0xc3,
      0x2f, 0xb6, 0x9e, 0x99, 0x74, 0x3f, 0x99, 0x53,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "moveit_msgs/msg/detail/planner_params__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "service_msgs/msg/detail/service_event_info__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t moveit_msgs__msg__PlannerParams__EXPECTED_HASH = {1, {
    0x91, 0xfe, 0x6e, 0xbf, 0x17, 0x15, 0x8d, 0x27,
    0x24, 0x01, 0x18, 0xbc, 0x8c, 0xce, 0xde, 0x6c,
    0xca, 0x55, 0xd6, 0x31, 0xc9, 0xa1, 0xcb, 0xce,
    0x84, 0x7b, 0x34, 0x27, 0x61, 0x26, 0x9d, 0x49,
  }};
static const rosidl_type_hash_t service_msgs__msg__ServiceEventInfo__EXPECTED_HASH = {1, {
    0x41, 0xbc, 0xbb, 0xe0, 0x7a, 0x75, 0xc9, 0xb5,
    0x2b, 0xc9, 0x6b, 0xfd, 0x5c, 0x24, 0xd7, 0xf0,
    0xfc, 0x0a, 0x08, 0xc0, 0xcb, 0x79, 0x21, 0xb3,
    0x37, 0x3c, 0x57, 0x32, 0x34, 0x5a, 0x6f, 0x45,
  }};
#endif

static char moveit_msgs__srv__GetPlannerParams__TYPE_NAME[] = "moveit_msgs/srv/GetPlannerParams";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char moveit_msgs__msg__PlannerParams__TYPE_NAME[] = "moveit_msgs/msg/PlannerParams";
static char moveit_msgs__srv__GetPlannerParams_Event__TYPE_NAME[] = "moveit_msgs/srv/GetPlannerParams_Event";
static char moveit_msgs__srv__GetPlannerParams_Request__TYPE_NAME[] = "moveit_msgs/srv/GetPlannerParams_Request";
static char moveit_msgs__srv__GetPlannerParams_Response__TYPE_NAME[] = "moveit_msgs/srv/GetPlannerParams_Response";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char moveit_msgs__srv__GetPlannerParams__FIELD_NAME__request_message[] = "request_message";
static char moveit_msgs__srv__GetPlannerParams__FIELD_NAME__response_message[] = "response_message";
static char moveit_msgs__srv__GetPlannerParams__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field moveit_msgs__srv__GetPlannerParams__FIELDS[] = {
  {
    {moveit_msgs__srv__GetPlannerParams__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {moveit_msgs__srv__GetPlannerParams_Request__TYPE_NAME, 40, 40},
    },
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__srv__GetPlannerParams__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {moveit_msgs__srv__GetPlannerParams_Response__TYPE_NAME, 41, 41},
    },
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__srv__GetPlannerParams__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {moveit_msgs__srv__GetPlannerParams_Event__TYPE_NAME, 38, 38},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription moveit_msgs__srv__GetPlannerParams__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__msg__PlannerParams__TYPE_NAME, 29, 29},
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__srv__GetPlannerParams_Event__TYPE_NAME, 38, 38},
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__srv__GetPlannerParams_Request__TYPE_NAME, 40, 40},
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__srv__GetPlannerParams_Response__TYPE_NAME, 41, 41},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
moveit_msgs__srv__GetPlannerParams__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {moveit_msgs__srv__GetPlannerParams__TYPE_NAME, 32, 32},
      {moveit_msgs__srv__GetPlannerParams__FIELDS, 3, 3},
    },
    {moveit_msgs__srv__GetPlannerParams__REFERENCED_TYPE_DESCRIPTIONS, 6, 6},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&moveit_msgs__msg__PlannerParams__EXPECTED_HASH, moveit_msgs__msg__PlannerParams__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = moveit_msgs__msg__PlannerParams__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = moveit_msgs__srv__GetPlannerParams_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = moveit_msgs__srv__GetPlannerParams_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[4].fields = moveit_msgs__srv__GetPlannerParams_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[5].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char moveit_msgs__srv__GetPlannerParams_Request__FIELD_NAME__pipeline_id[] = "pipeline_id";
static char moveit_msgs__srv__GetPlannerParams_Request__FIELD_NAME__planner_config[] = "planner_config";
static char moveit_msgs__srv__GetPlannerParams_Request__FIELD_NAME__group[] = "group";

static rosidl_runtime_c__type_description__Field moveit_msgs__srv__GetPlannerParams_Request__FIELDS[] = {
  {
    {moveit_msgs__srv__GetPlannerParams_Request__FIELD_NAME__pipeline_id, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__srv__GetPlannerParams_Request__FIELD_NAME__planner_config, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__srv__GetPlannerParams_Request__FIELD_NAME__group, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
moveit_msgs__srv__GetPlannerParams_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {moveit_msgs__srv__GetPlannerParams_Request__TYPE_NAME, 40, 40},
      {moveit_msgs__srv__GetPlannerParams_Request__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char moveit_msgs__srv__GetPlannerParams_Response__FIELD_NAME__params[] = "params";

static rosidl_runtime_c__type_description__Field moveit_msgs__srv__GetPlannerParams_Response__FIELDS[] = {
  {
    {moveit_msgs__srv__GetPlannerParams_Response__FIELD_NAME__params, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {moveit_msgs__msg__PlannerParams__TYPE_NAME, 29, 29},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription moveit_msgs__srv__GetPlannerParams_Response__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {moveit_msgs__msg__PlannerParams__TYPE_NAME, 29, 29},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
moveit_msgs__srv__GetPlannerParams_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {moveit_msgs__srv__GetPlannerParams_Response__TYPE_NAME, 41, 41},
      {moveit_msgs__srv__GetPlannerParams_Response__FIELDS, 1, 1},
    },
    {moveit_msgs__srv__GetPlannerParams_Response__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&moveit_msgs__msg__PlannerParams__EXPECTED_HASH, moveit_msgs__msg__PlannerParams__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = moveit_msgs__msg__PlannerParams__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char moveit_msgs__srv__GetPlannerParams_Event__FIELD_NAME__info[] = "info";
static char moveit_msgs__srv__GetPlannerParams_Event__FIELD_NAME__request[] = "request";
static char moveit_msgs__srv__GetPlannerParams_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field moveit_msgs__srv__GetPlannerParams_Event__FIELDS[] = {
  {
    {moveit_msgs__srv__GetPlannerParams_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__srv__GetPlannerParams_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {moveit_msgs__srv__GetPlannerParams_Request__TYPE_NAME, 40, 40},
    },
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__srv__GetPlannerParams_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {moveit_msgs__srv__GetPlannerParams_Response__TYPE_NAME, 41, 41},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription moveit_msgs__srv__GetPlannerParams_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__msg__PlannerParams__TYPE_NAME, 29, 29},
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__srv__GetPlannerParams_Request__TYPE_NAME, 40, 40},
    {NULL, 0, 0},
  },
  {
    {moveit_msgs__srv__GetPlannerParams_Response__TYPE_NAME, 41, 41},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
moveit_msgs__srv__GetPlannerParams_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {moveit_msgs__srv__GetPlannerParams_Event__TYPE_NAME, 38, 38},
      {moveit_msgs__srv__GetPlannerParams_Event__FIELDS, 3, 3},
    },
    {moveit_msgs__srv__GetPlannerParams_Event__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&moveit_msgs__msg__PlannerParams__EXPECTED_HASH, moveit_msgs__msg__PlannerParams__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = moveit_msgs__msg__PlannerParams__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = moveit_msgs__srv__GetPlannerParams_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = moveit_msgs__srv__GetPlannerParams_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Name of the planning pipeline, uses default if empty\n"
  "string pipeline_id\n"
  "\n"
  "# Name of planning config\n"
  "string planner_config\n"
  "\n"
  "# Optional name of planning group (return global defaults if empty)\n"
  "string group\n"
  "\n"
  "---\n"
  "\n"
  "# parameters as key-value pairs\n"
  "PlannerParams params";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
moveit_msgs__srv__GetPlannerParams__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {moveit_msgs__srv__GetPlannerParams__TYPE_NAME, 32, 32},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 264, 264},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
moveit_msgs__srv__GetPlannerParams_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {moveit_msgs__srv__GetPlannerParams_Request__TYPE_NAME, 40, 40},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
moveit_msgs__srv__GetPlannerParams_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {moveit_msgs__srv__GetPlannerParams_Response__TYPE_NAME, 41, 41},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
moveit_msgs__srv__GetPlannerParams_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {moveit_msgs__srv__GetPlannerParams_Event__TYPE_NAME, 38, 38},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
moveit_msgs__srv__GetPlannerParams__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[7];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 7, 7};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *moveit_msgs__srv__GetPlannerParams__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *moveit_msgs__msg__PlannerParams__get_individual_type_description_source(NULL);
    sources[3] = *moveit_msgs__srv__GetPlannerParams_Event__get_individual_type_description_source(NULL);
    sources[4] = *moveit_msgs__srv__GetPlannerParams_Request__get_individual_type_description_source(NULL);
    sources[5] = *moveit_msgs__srv__GetPlannerParams_Response__get_individual_type_description_source(NULL);
    sources[6] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
moveit_msgs__srv__GetPlannerParams_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *moveit_msgs__srv__GetPlannerParams_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
moveit_msgs__srv__GetPlannerParams_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *moveit_msgs__srv__GetPlannerParams_Response__get_individual_type_description_source(NULL),
    sources[1] = *moveit_msgs__msg__PlannerParams__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
moveit_msgs__srv__GetPlannerParams_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *moveit_msgs__srv__GetPlannerParams_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *moveit_msgs__msg__PlannerParams__get_individual_type_description_source(NULL);
    sources[3] = *moveit_msgs__srv__GetPlannerParams_Request__get_individual_type_description_source(NULL);
    sources[4] = *moveit_msgs__srv__GetPlannerParams_Response__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
