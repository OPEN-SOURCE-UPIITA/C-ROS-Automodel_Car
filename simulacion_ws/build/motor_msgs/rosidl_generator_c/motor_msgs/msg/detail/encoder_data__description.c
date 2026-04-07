// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from motor_msgs:msg/EncoderData.idl
// generated code does not contain a copyright notice

#include "motor_msgs/msg/detail/encoder_data__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_motor_msgs
const rosidl_type_hash_t *
motor_msgs__msg__EncoderData__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x48, 0x59, 0x8c, 0xa8, 0x16, 0xe7, 0xe3, 0xf5,
      0xac, 0x10, 0xe1, 0x30, 0x45, 0xbe, 0x1a, 0x78,
      0xdc, 0xe8, 0x97, 0xb5, 0xcb, 0xdf, 0x09, 0xe3,
      0x18, 0x05, 0xf0, 0x82, 0x0e, 0x67, 0x5e, 0x36,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char motor_msgs__msg__EncoderData__TYPE_NAME[] = "motor_msgs/msg/EncoderData";

// Define type names, field names, and default values
static char motor_msgs__msg__EncoderData__FIELD_NAME__vel_m1[] = "vel_m1";
static char motor_msgs__msg__EncoderData__FIELD_NAME__dir_m1[] = "dir_m1";
static char motor_msgs__msg__EncoderData__FIELD_NAME__vel_m2[] = "vel_m2";
static char motor_msgs__msg__EncoderData__FIELD_NAME__dir_m2[] = "dir_m2";

static rosidl_runtime_c__type_description__Field motor_msgs__msg__EncoderData__FIELDS[] = {
  {
    {motor_msgs__msg__EncoderData__FIELD_NAME__vel_m1, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {motor_msgs__msg__EncoderData__FIELD_NAME__dir_m1, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {motor_msgs__msg__EncoderData__FIELD_NAME__vel_m2, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {motor_msgs__msg__EncoderData__FIELD_NAME__dir_m2, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
motor_msgs__msg__EncoderData__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {motor_msgs__msg__EncoderData__TYPE_NAME, 26, 26},
      {motor_msgs__msg__EncoderData__FIELDS, 4, 4},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Datos de Odometr\\xc3\\xada de los Encoders\n"
  "# vel: Velocidad en \"ticks por ciclo\" (Valor absoluto)\n"
  "# dir: 0=Stop, 1=Adelante, 2=Atr\\xc3\\xa1s (Misma convenci\\xc3\\xb3n que los motores DC)\n"
  "\n"
  "int32 vel_m1\n"
  "int8 dir_m1\n"
  "int32 vel_m2\n"
  "int8 dir_m2";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
motor_msgs__msg__EncoderData__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {motor_msgs__msg__EncoderData__TYPE_NAME, 26, 26},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 215, 215},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
motor_msgs__msg__EncoderData__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *motor_msgs__msg__EncoderData__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
