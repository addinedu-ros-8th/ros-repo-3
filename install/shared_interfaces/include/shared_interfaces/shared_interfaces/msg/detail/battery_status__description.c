// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from shared_interfaces:msg/BatteryStatus.idl
// generated code does not contain a copyright notice

#include "shared_interfaces/msg/detail/battery_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_shared_interfaces
const rosidl_type_hash_t *
shared_interfaces__msg__BatteryStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x7c, 0x30, 0x97, 0x9c, 0x00, 0xe1, 0x8f, 0x21,
      0xf8, 0xa1, 0xb9, 0x68, 0x78, 0x8a, 0x26, 0x3b,
      0xfc, 0x0a, 0x9f, 0xa4, 0x27, 0x33, 0xf9, 0x64,
      0x83, 0x79, 0x7d, 0x31, 0xaf, 0x92, 0x2d, 0x3c,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char shared_interfaces__msg__BatteryStatus__TYPE_NAME[] = "shared_interfaces/msg/BatteryStatus";

// Define type names, field names, and default values
static char shared_interfaces__msg__BatteryStatus__FIELD_NAME__voltage[] = "voltage";
static char shared_interfaces__msg__BatteryStatus__FIELD_NAME__current[] = "current";
static char shared_interfaces__msg__BatteryStatus__FIELD_NAME__percentage[] = "percentage";

static rosidl_runtime_c__type_description__Field shared_interfaces__msg__BatteryStatus__FIELDS[] = {
  {
    {shared_interfaces__msg__BatteryStatus__FIELD_NAME__voltage, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {shared_interfaces__msg__BatteryStatus__FIELD_NAME__current, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {shared_interfaces__msg__BatteryStatus__FIELD_NAME__percentage, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
shared_interfaces__msg__BatteryStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {shared_interfaces__msg__BatteryStatus__TYPE_NAME, 35, 35},
      {shared_interfaces__msg__BatteryStatus__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "float32 voltage\n"
  "float32 current\n"
  "float32 percentage";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
shared_interfaces__msg__BatteryStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {shared_interfaces__msg__BatteryStatus__TYPE_NAME, 35, 35},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 50, 50},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
shared_interfaces__msg__BatteryStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *shared_interfaces__msg__BatteryStatus__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
