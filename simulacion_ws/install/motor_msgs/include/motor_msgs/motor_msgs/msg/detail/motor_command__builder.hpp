// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from motor_msgs:msg/MotorCommand.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "motor_msgs/msg/motor_command.hpp"


#ifndef MOTOR_MSGS__MSG__DETAIL__MOTOR_COMMAND__BUILDER_HPP_
#define MOTOR_MSGS__MSG__DETAIL__MOTOR_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "motor_msgs/msg/detail/motor_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace motor_msgs
{

namespace msg
{

namespace builder
{

class Init_MotorCommand_dir_servo
{
public:
  explicit Init_MotorCommand_dir_servo(::motor_msgs::msg::MotorCommand & msg)
  : msg_(msg)
  {}
  ::motor_msgs::msg::MotorCommand dir_servo(::motor_msgs::msg::MotorCommand::_dir_servo_type arg)
  {
    msg_.dir_servo = std::move(arg);
    return std::move(msg_);
  }

private:
  ::motor_msgs::msg::MotorCommand msg_;
};

class Init_MotorCommand_speed_dc
{
public:
  explicit Init_MotorCommand_speed_dc(::motor_msgs::msg::MotorCommand & msg)
  : msg_(msg)
  {}
  Init_MotorCommand_dir_servo speed_dc(::motor_msgs::msg::MotorCommand::_speed_dc_type arg)
  {
    msg_.speed_dc = std::move(arg);
    return Init_MotorCommand_dir_servo(msg_);
  }

private:
  ::motor_msgs::msg::MotorCommand msg_;
};

class Init_MotorCommand_dir_dc
{
public:
  Init_MotorCommand_dir_dc()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotorCommand_speed_dc dir_dc(::motor_msgs::msg::MotorCommand::_dir_dc_type arg)
  {
    msg_.dir_dc = std::move(arg);
    return Init_MotorCommand_speed_dc(msg_);
  }

private:
  ::motor_msgs::msg::MotorCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::motor_msgs::msg::MotorCommand>()
{
  return motor_msgs::msg::builder::Init_MotorCommand_dir_dc();
}

}  // namespace motor_msgs

#endif  // MOTOR_MSGS__MSG__DETAIL__MOTOR_COMMAND__BUILDER_HPP_
