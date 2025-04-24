// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from shared_interfaces:msg/Emergency.idl
// generated code does not contain a copyright notice

#include "shared_interfaces/msg/detail/emergency__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_shared_interfaces
const rosidl_type_hash_t *
shared_interfaces__msg__Emergency__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xab, 0x7c, 0xcf, 0x9c, 0x1c, 0xb0, 0x60, 0xeb,
      0x3f, 0x9a, 0xf4, 0xaa, 0x10, 0x1c, 0x3e, 0xe0,
      0xc8, 0x91, 0x65, 0x7b, 0x95, 0x4a, 0x53, 0xc1,
      0xcd, 0x10, 0x92, 0x5f, 0xe8, 0xa1, 0xd8, 0x3e,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char shared_interfaces__msg__Emergency__TYPE_NAME[] = "shared_interfaces/msg/Emergency";

// Define type names, field names, and default values
static char shared_interfaces__msg__Emergency__FIELD_NAME__emergency[] = "emergency";
static char shared_interfaces__msg__Emergency__FIELD_NAME__description[] = "description";

static rosidl_runtime_c__type_description__Field shared_interfaces__msg__Emergency__FIELDS[] = {
  {
    {shared_interfaces__msg__Emergency__FIELD_NAME__emergency, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {shared_interfaces__msg__Emergency__FIELD_NAME__description, 11, 11},
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
shared_interfaces__msg__Emergency__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {shared_interfaces__msg__Emergency__TYPE_NAME, 31, 31},
      {shared_interfaces__msg__Emergency__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "bool emergency\n"
  "string description";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
shared_interfaces__msg__Emergency__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {shared_interfaces__msg__Emergency__TYPE_NAME, 31, 31},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 33, 33},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
shared_interfaces__msg__Emergency__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *shared_interfaces__msg__Emergency__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
