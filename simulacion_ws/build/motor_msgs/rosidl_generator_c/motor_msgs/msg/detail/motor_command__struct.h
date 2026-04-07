// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from motor_msgs:msg/MotorCommand.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "motor_msgs/msg/motor_command.h"


#ifndef MOTOR_MSGS__MSG__DETAIL__MOTOR_COMMAND__STRUCT_H_
#define MOTOR_MSGS__MSG__DETAIL__MOTOR_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/MotorCommand in the package motor_msgs.
/**
  * Comando para motores DC y servo
  * dir_dc: 0=Stop, 1=Adelante, 2=Atrás
  * speed_dc: 0-100
  * dir_servo: 1110-1740 (1500=Centro)
 */
typedef struct motor_msgs__msg__MotorCommand
{
  int8_t dir_dc;
  int8_t speed_dc;
  int16_t dir_servo;
} motor_msgs__msg__MotorCommand;

// Struct for a sequence of motor_msgs__msg__MotorCommand.
typedef struct motor_msgs__msg__MotorCommand__Sequence
{
  motor_msgs__msg__MotorCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} motor_msgs__msg__MotorCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MOTOR_MSGS__MSG__DETAIL__MOTOR_COMMAND__STRUCT_H_
