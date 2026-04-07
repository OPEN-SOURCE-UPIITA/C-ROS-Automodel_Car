// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from motor_msgs:msg/EncoderData.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "motor_msgs/msg/encoder_data.hpp"


#ifndef MOTOR_MSGS__MSG__DETAIL__ENCODER_DATA__TRAITS_HPP_
#define MOTOR_MSGS__MSG__DETAIL__ENCODER_DATA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "motor_msgs/msg/detail/encoder_data__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace motor_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const EncoderData & msg,
  std::ostream & out)
{
  out << "{";
  // member: vel_m1
  {
    out << "vel_m1: ";
    rosidl_generator_traits::value_to_yaml(msg.vel_m1, out);
    out << ", ";
  }

  // member: dir_m1
  {
    out << "dir_m1: ";
    rosidl_generator_traits::value_to_yaml(msg.dir_m1, out);
    out << ", ";
  }

  // member: vel_m2
  {
    out << "vel_m2: ";
    rosidl_generator_traits::value_to_yaml(msg.vel_m2, out);
    out << ", ";
  }

  // member: dir_m2
  {
    out << "dir_m2: ";
    rosidl_generator_traits::value_to_yaml(msg.dir_m2, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const EncoderData & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: vel_m1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vel_m1: ";
    rosidl_generator_traits::value_to_yaml(msg.vel_m1, out);
    out << "\n";
  }

  // member: dir_m1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "dir_m1: ";
    rosidl_generator_traits::value_to_yaml(msg.dir_m1, out);
    out << "\n";
  }

  // member: vel_m2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vel_m2: ";
    rosidl_generator_traits::value_to_yaml(msg.vel_m2, out);
    out << "\n";
  }

  // member: dir_m2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "dir_m2: ";
    rosidl_generator_traits::value_to_yaml(msg.dir_m2, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const EncoderData & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace motor_msgs

namespace rosidl_generator_traits
{

[[deprecated("use motor_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const motor_msgs::msg::EncoderData & msg,
  std::ostream & out, size_t indentation = 0)
{
  motor_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use motor_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const motor_msgs::msg::EncoderData & msg)
{
  return motor_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<motor_msgs::msg::EncoderData>()
{
  return "motor_msgs::msg::EncoderData";
}

template<>
inline const char * name<motor_msgs::msg::EncoderData>()
{
  return "motor_msgs/msg/EncoderData";
}

template<>
struct has_fixed_size<motor_msgs::msg::EncoderData>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<motor_msgs::msg::EncoderData>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<motor_msgs::msg::EncoderData>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MOTOR_MSGS__MSG__DETAIL__ENCODER_DATA__TRAITS_HPP_
