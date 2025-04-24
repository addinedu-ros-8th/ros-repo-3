// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from shared_interfaces:msg/TaskResult.idl
// generated code does not contain a copyright notice

#include "shared_interfaces/msg/detail/task_result__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_shared_interfaces
const rosidl_type_hash_t *
shared_interfaces__msg__TaskResult__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x91, 0xbd, 0x41, 0x0a, 0x2f, 0xd3, 0xaf, 0x12,
      0x53, 0x41, 0x85, 0x65, 0x39, 0xf9, 0x74, 0x40,
      0xec, 0x68, 0x0c, 0x27, 0xa5, 0xa1, 0xbd, 0x68,
      0x33, 0xf4, 0x61, 0x37, 0xd2, 0x09, 0x18, 0xd8,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char shared_interfaces__msg__TaskResult__TYPE_NAME[] = "shared_interfaces/msg/TaskResult";

// Define type names, field names, and default values
static char shared_interfaces__msg__TaskResult__FIELD_NAME__task_id[] = "task_id";
static char shared_interfaces__msg__TaskResult__FIELD_NAME__result[] = "result";

static rosidl_runtime_c__type_description__Field shared_interfaces__msg__TaskResult__FIELDS[] = {
  {
    {shared_interfaces__msg__TaskResult__FIELD_NAME__task_id, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {shared_interfaces__msg__TaskResult__FIELD_NAME__result, 6, 6},
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
shared_interfaces__msg__TaskResult__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {shared_interfaces__msg__TaskResult__TYPE_NAME, 32, 32},
      {shared_interfaces__msg__TaskResult__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "string task_id\n"
  "string result";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
shared_interfaces__msg__TaskResult__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {shared_interfaces__msg__TaskResult__TYPE_NAME, 32, 32},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 28, 28},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
shared_interfaces__msg__TaskResult__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *shared_interfaces__msg__TaskResult__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
