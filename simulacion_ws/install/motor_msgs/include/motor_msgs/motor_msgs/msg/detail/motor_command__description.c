// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from motor_msgs:msg/MotorCommand.idl
// generated code does not contain a copyright notice

#include "motor_msgs/msg/detail/motor_command__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_motor_msgs
const rosidl_type_hash_t *
motor_msgs__msg__MotorCommand__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xe0, 0x44, 0xbe, 0xc4, 0xe6, 0x80, 0xe4, 0xd7,
      0x5d, 0x8e, 0x36, 0x51, 0x3e, 0x38, 0x03, 0xbd,
      0x6d, 0x55, 0xe8, 0xc8, 0xd7, 0xe8, 0xbb, 0x0d,
      0x3f, 0xbf, 0x99, 0xaf, 0xba, 0x3c, 0x42, 0x3d,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char motor_msgs__msg__MotorCommand__TYPE_NAME[] = "motor_msgs/msg/MotorCommand";

// Define type names, field names, and default values
static char motor_msgs__msg__MotorCommand__FIELD_NAME__dir_dc[] = "dir_dc";
static char motor_msgs__msg__MotorCommand__FIELD_NAME__speed_dc[] = "speed_dc";
static char motor_msgs__msg__MotorCommand__FIELD_NAME__dir_servo[] = "dir_servo";

static rosidl_runtime_c__type_description__Field motor_msgs__msg__MotorCommand__FIELDS[] = {
  {
    {motor_msgs__msg__MotorCommand__FIELD_NAME__dir_dc, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {motor_msgs__msg__MotorCommand__FIELD_NAME__speed_dc, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {motor_msgs__msg__MotorCommand__FIELD_NAME__dir_servo, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
motor_msgs__msg__MotorCommand__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {motor_msgs__msg__MotorCommand__TYPE_NAME, 27, 27},
      {motor_msgs__msg__MotorCommand__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Comando para motores DC y servo\n"
  "# dir_dc: 0=Stop, 1=Adelante, 2=Atr\\xc3\\xa1s\n"
  "# speed_dc: 0-100\n"
  "# dir_servo: 1110-1740 (1500=Centro)\n"
  "\n"
  "int8 dir_dc\n"
  "int8 speed_dc\n"
  "int16 dir_servo";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
motor_msgs__msg__MotorCommand__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {motor_msgs__msg__MotorCommand__TYPE_NAME, 27, 27},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 169, 169},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
motor_msgs__msg__MotorCommand__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *motor_msgs__msg__MotorCommand__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
