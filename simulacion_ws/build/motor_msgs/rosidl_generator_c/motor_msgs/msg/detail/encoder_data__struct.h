// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from motor_msgs:msg/EncoderData.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "motor_msgs/msg/encoder_data.h"


#ifndef MOTOR_MSGS__MSG__DETAIL__ENCODER_DATA__STRUCT_H_
#define MOTOR_MSGS__MSG__DETAIL__ENCODER_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/EncoderData in the package motor_msgs.
/**
  * Datos para el control de los Encoders
  * vel: Velocidad en "ticks por ciclo" (Valor absoluto)
  * dir: 0=Stop, 1=Adelante, 2=Atrás (Misma convención que los motores DC)
 */
typedef struct motor_msgs__msg__EncoderData
{
  int32_t vel_m1;
  int8_t dir_m1;
  int32_t vel_m2;
  int8_t dir_m2;
} motor_msgs__msg__EncoderData;

// Struct for a sequence of motor_msgs__msg__EncoderData.
typedef struct motor_msgs__msg__EncoderData__Sequence
{
  motor_msgs__msg__EncoderData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} motor_msgs__msg__EncoderData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MOTOR_MSGS__MSG__DETAIL__ENCODER_DATA__STRUCT_H_
