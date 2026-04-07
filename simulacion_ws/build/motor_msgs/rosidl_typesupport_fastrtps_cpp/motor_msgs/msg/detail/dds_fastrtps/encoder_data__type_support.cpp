// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from motor_msgs:msg/EncoderData.idl
// generated code does not contain a copyright notice
#include "motor_msgs/msg/detail/encoder_data__rosidl_typesupport_fastrtps_cpp.hpp"
#include "motor_msgs/msg/detail/encoder_data__functions.h"
#include "motor_msgs/msg/detail/encoder_data__struct.hpp"

#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace motor_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{


bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_motor_msgs
cdr_serialize(
  const motor_msgs::msg::EncoderData & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: vel_m1
  cdr << ros_message.vel_m1;

  // Member: dir_m1
  cdr << ros_message.dir_m1;

  // Member: vel_m2
  cdr << ros_message.vel_m2;

  // Member: dir_m2
  cdr << ros_message.dir_m2;

  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_motor_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  motor_msgs::msg::EncoderData & ros_message)
{
  // Member: vel_m1
  cdr >> ros_message.vel_m1;

  // Member: dir_m1
  cdr >> ros_message.dir_m1;

  // Member: vel_m2
  cdr >> ros_message.vel_m2;

  // Member: dir_m2
  cdr >> ros_message.dir_m2;

  return true;
}  // NOLINT(readability/fn_size)


size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_motor_msgs
get_serialized_size(
  const motor_msgs::msg::EncoderData & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: vel_m1
  {
    size_t item_size = sizeof(ros_message.vel_m1);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: dir_m1
  {
    size_t item_size = sizeof(ros_message.dir_m1);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: vel_m2
  {
    size_t item_size = sizeof(ros_message.vel_m2);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: dir_m2
  {
    size_t item_size = sizeof(ros_message.dir_m2);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_motor_msgs
max_serialized_size_EncoderData(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // Member: vel_m1
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // Member: dir_m1
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // Member: vel_m2
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // Member: dir_m2
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = motor_msgs::msg::EncoderData;
    is_plain =
      (
      offsetof(DataType, dir_m2) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_motor_msgs
cdr_serialize_key(
  const motor_msgs::msg::EncoderData & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: vel_m1
  cdr << ros_message.vel_m1;

  // Member: dir_m1
  cdr << ros_message.dir_m1;

  // Member: vel_m2
  cdr << ros_message.vel_m2;

  // Member: dir_m2
  cdr << ros_message.dir_m2;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_motor_msgs
get_serialized_size_key(
  const motor_msgs::msg::EncoderData & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: vel_m1
  {
    size_t item_size = sizeof(ros_message.vel_m1);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: dir_m1
  {
    size_t item_size = sizeof(ros_message.dir_m1);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: vel_m2
  {
    size_t item_size = sizeof(ros_message.vel_m2);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: dir_m2
  {
    size_t item_size = sizeof(ros_message.dir_m2);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_motor_msgs
max_serialized_size_key_EncoderData(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // Member: vel_m1
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: dir_m1
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: vel_m2
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: dir_m2
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = motor_msgs::msg::EncoderData;
    is_plain =
      (
      offsetof(DataType, dir_m2) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}


static bool _EncoderData__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const motor_msgs::msg::EncoderData *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _EncoderData__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<motor_msgs::msg::EncoderData *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _EncoderData__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const motor_msgs::msg::EncoderData *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _EncoderData__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_EncoderData(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _EncoderData__callbacks = {
  "motor_msgs::msg",
  "EncoderData",
  _EncoderData__cdr_serialize,
  _EncoderData__cdr_deserialize,
  _EncoderData__get_serialized_size,
  _EncoderData__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _EncoderData__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_EncoderData__callbacks,
  get_message_typesupport_handle_function,
  &motor_msgs__msg__EncoderData__get_type_hash,
  &motor_msgs__msg__EncoderData__get_type_description,
  &motor_msgs__msg__EncoderData__get_type_description_sources,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace motor_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_motor_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<motor_msgs::msg::EncoderData>()
{
  return &motor_msgs::msg::typesupport_fastrtps_cpp::_EncoderData__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, motor_msgs, msg, EncoderData)() {
  return &motor_msgs::msg::typesupport_fastrtps_cpp::_EncoderData__handle;
}

#ifdef __cplusplus
}
#endif
