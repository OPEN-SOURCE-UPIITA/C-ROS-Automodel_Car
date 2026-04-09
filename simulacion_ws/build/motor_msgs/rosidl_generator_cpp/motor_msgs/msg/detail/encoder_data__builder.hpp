// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from motor_msgs:msg/EncoderData.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "motor_msgs/msg/encoder_data.hpp"


#ifndef MOTOR_MSGS__MSG__DETAIL__ENCODER_DATA__BUILDER_HPP_
#define MOTOR_MSGS__MSG__DETAIL__ENCODER_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "motor_msgs/msg/detail/encoder_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace motor_msgs
{

namespace msg
{

namespace builder
{

class Init_EncoderData_dir_m2
{
public:
  explicit Init_EncoderData_dir_m2(::motor_msgs::msg::EncoderData & msg)
  : msg_(msg)
  {}
  ::motor_msgs::msg::EncoderData dir_m2(::motor_msgs::msg::EncoderData::_dir_m2_type arg)
  {
    msg_.dir_m2 = std::move(arg);
    return std::move(msg_);
  }

private:
  ::motor_msgs::msg::EncoderData msg_;
};

class Init_EncoderData_vel_m2
{
public:
  explicit Init_EncoderData_vel_m2(::motor_msgs::msg::EncoderData & msg)
  : msg_(msg)
  {}
  Init_EncoderData_dir_m2 vel_m2(::motor_msgs::msg::EncoderData::_vel_m2_type arg)
  {
    msg_.vel_m2 = std::move(arg);
    return Init_EncoderData_dir_m2(msg_);
  }

private:
  ::motor_msgs::msg::EncoderData msg_;
};

class Init_EncoderData_dir_m1
{
public:
  explicit Init_EncoderData_dir_m1(::motor_msgs::msg::EncoderData & msg)
  : msg_(msg)
  {}
  Init_EncoderData_vel_m2 dir_m1(::motor_msgs::msg::EncoderData::_dir_m1_type arg)
  {
    msg_.dir_m1 = std::move(arg);
    return Init_EncoderData_vel_m2(msg_);
  }

private:
  ::motor_msgs::msg::EncoderData msg_;
};

class Init_EncoderData_vel_m1
{
public:
  Init_EncoderData_vel_m1()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_EncoderData_dir_m1 vel_m1(::motor_msgs::msg::EncoderData::_vel_m1_type arg)
  {
    msg_.vel_m1 = std::move(arg);
    return Init_EncoderData_dir_m1(msg_);
  }

private:
  ::motor_msgs::msg::EncoderData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::motor_msgs::msg::EncoderData>()
{
  return motor_msgs::msg::builder::Init_EncoderData_vel_m1();
}

}  // namespace motor_msgs

#endif  // MOTOR_MSGS__MSG__DETAIL__ENCODER_DATA__BUILDER_HPP_
