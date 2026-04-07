// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from motor_msgs:msg/EncoderData.idl
// generated code does not contain a copyright notice
#ifndef MOTOR_MSGS__MSG__DETAIL__ENCODER_DATA__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define MOTOR_MSGS__MSG__DETAIL__ENCODER_DATA__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "motor_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "motor_msgs/msg/detail/encoder_data__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_motor_msgs
bool cdr_serialize_motor_msgs__msg__EncoderData(
  const motor_msgs__msg__EncoderData * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_motor_msgs
bool cdr_deserialize_motor_msgs__msg__EncoderData(
  eprosima::fastcdr::Cdr &,
  motor_msgs__msg__EncoderData * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_motor_msgs
size_t get_serialized_size_motor_msgs__msg__EncoderData(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_motor_msgs
size_t max_serialized_size_motor_msgs__msg__EncoderData(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_motor_msgs
bool cdr_serialize_key_motor_msgs__msg__EncoderData(
  const motor_msgs__msg__EncoderData * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_motor_msgs
size_t get_serialized_size_key_motor_msgs__msg__EncoderData(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_motor_msgs
size_t max_serialized_size_key_motor_msgs__msg__EncoderData(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_motor_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, motor_msgs, msg, EncoderData)();

#ifdef __cplusplus
}
#endif

#endif  // MOTOR_MSGS__MSG__DETAIL__ENCODER_DATA__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
