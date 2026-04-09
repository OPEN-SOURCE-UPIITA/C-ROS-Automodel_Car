// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from motor_msgs:msg/MotorCommand.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "motor_msgs/msg/motor_command.hpp"


#ifndef MOTOR_MSGS__MSG__DETAIL__MOTOR_COMMAND__TRAITS_HPP_
#define MOTOR_MSGS__MSG__DETAIL__MOTOR_COMMAND__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "motor_msgs/msg/detail/motor_command__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace motor_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const MotorCommand & msg,
  std::ostream & out)
{
  out << "{";
  // member: dir_dc
  {
    out << "dir_dc: ";
    rosidl_generator_traits::value_to_yaml(msg.dir_dc, out);
    out << ", ";
  }

  // member: speed_dc
  {
    out << "speed_dc: ";
    rosidl_generator_traits::value_to_yaml(msg.speed_dc, out);
    out << ", ";
  }

  // member: dir_servo
  {
    out << "dir_servo: ";
    rosidl_generator_traits::value_to_yaml(msg.dir_servo, out);
    out << ", ";
  }

  // member: stop_lights
  {
    out << "stop_lights: ";
    rosidl_generator_traits::value_to_yaml(msg.stop_lights, out);
    out << ", ";
  }

  // member: turn_signals
  {
    out << "turn_signals: ";
    rosidl_generator_traits::value_to_yaml(msg.turn_signals, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MotorCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: dir_dc
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "dir_dc: ";
    rosidl_generator_traits::value_to_yaml(msg.dir_dc, out);
    out << "\n";
  }

  // member: speed_dc
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "speed_dc: ";
    rosidl_generator_traits::value_to_yaml(msg.speed_dc, out);
    out << "\n";
  }

  // member: dir_servo
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "dir_servo: ";
    rosidl_generator_traits::value_to_yaml(msg.dir_servo, out);
    out << "\n";
  }

  // member: stop_lights
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stop_lights: ";
    rosidl_generator_traits::value_to_yaml(msg.stop_lights, out);
    out << "\n";
  }

  // member: turn_signals
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "turn_signals: ";
    rosidl_generator_traits::value_to_yaml(msg.turn_signals, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MotorCommand & msg, bool use_flow_style = false)
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
  const motor_msgs::msg::MotorCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  motor_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use motor_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const motor_msgs::msg::MotorCommand & msg)
{
  return motor_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<motor_msgs::msg::MotorCommand>()
{
  return "motor_msgs::msg::MotorCommand";
}

template<>
inline const char * name<motor_msgs::msg::MotorCommand>()
{
  return "motor_msgs/msg/MotorCommand";
}

template<>
struct has_fixed_size<motor_msgs::msg::MotorCommand>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<motor_msgs::msg::MotorCommand>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<motor_msgs::msg::MotorCommand>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MOTOR_MSGS__MSG__DETAIL__MOTOR_COMMAND__TRAITS_HPP_
