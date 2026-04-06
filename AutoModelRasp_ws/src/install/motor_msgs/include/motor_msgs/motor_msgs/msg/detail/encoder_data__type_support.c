// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from motor_msgs:msg/EncoderData.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "motor_msgs/msg/detail/encoder_data__rosidl_typesupport_introspection_c.h"
#include "motor_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "motor_msgs/msg/detail/encoder_data__functions.h"
#include "motor_msgs/msg/detail/encoder_data__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void motor_msgs__msg__EncoderData__rosidl_typesupport_introspection_c__EncoderData_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  motor_msgs__msg__EncoderData__init(message_memory);
}

void motor_msgs__msg__EncoderData__rosidl_typesupport_introspection_c__EncoderData_fini_function(void * message_memory)
{
  motor_msgs__msg__EncoderData__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember motor_msgs__msg__EncoderData__rosidl_typesupport_introspection_c__EncoderData_message_member_array[4] = {
  {
    "vel_m1",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(motor_msgs__msg__EncoderData, vel_m1),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "dir_m1",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(motor_msgs__msg__EncoderData, dir_m1),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "vel_m2",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(motor_msgs__msg__EncoderData, vel_m2),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "dir_m2",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(motor_msgs__msg__EncoderData, dir_m2),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers motor_msgs__msg__EncoderData__rosidl_typesupport_introspection_c__EncoderData_message_members = {
  "motor_msgs__msg",  // message namespace
  "EncoderData",  // message name
  4,  // number of fields
  sizeof(motor_msgs__msg__EncoderData),
  false,  // has_any_key_member_
  motor_msgs__msg__EncoderData__rosidl_typesupport_introspection_c__EncoderData_message_member_array,  // message members
  motor_msgs__msg__EncoderData__rosidl_typesupport_introspection_c__EncoderData_init_function,  // function to initialize message memory (memory has to be allocated)
  motor_msgs__msg__EncoderData__rosidl_typesupport_introspection_c__EncoderData_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t motor_msgs__msg__EncoderData__rosidl_typesupport_introspection_c__EncoderData_message_type_support_handle = {
  0,
  &motor_msgs__msg__EncoderData__rosidl_typesupport_introspection_c__EncoderData_message_members,
  get_message_typesupport_handle_function,
  &motor_msgs__msg__EncoderData__get_type_hash,
  &motor_msgs__msg__EncoderData__get_type_description,
  &motor_msgs__msg__EncoderData__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_motor_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, motor_msgs, msg, EncoderData)() {
  if (!motor_msgs__msg__EncoderData__rosidl_typesupport_introspection_c__EncoderData_message_type_support_handle.typesupport_identifier) {
    motor_msgs__msg__EncoderData__rosidl_typesupport_introspection_c__EncoderData_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &motor_msgs__msg__EncoderData__rosidl_typesupport_introspection_c__EncoderData_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
