// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from motor_msgs:msg/MotorCommand.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "motor_msgs/msg/motor_command.hpp"


#ifndef MOTOR_MSGS__MSG__DETAIL__MOTOR_COMMAND__STRUCT_HPP_
#define MOTOR_MSGS__MSG__DETAIL__MOTOR_COMMAND__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__motor_msgs__msg__MotorCommand __attribute__((deprecated))
#else
# define DEPRECATED__motor_msgs__msg__MotorCommand __declspec(deprecated)
#endif

namespace motor_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MotorCommand_
{
  using Type = MotorCommand_<ContainerAllocator>;

  explicit MotorCommand_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->dir_dc = 0;
      this->speed_dc = 0;
      this->dir_servo = 0;
      this->stop_lights = 0;
      this->turn_signals = 0;
    }
  }

  explicit MotorCommand_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->dir_dc = 0;
      this->speed_dc = 0;
      this->dir_servo = 0;
      this->stop_lights = 0;
      this->turn_signals = 0;
    }
  }

  // field types and members
  using _dir_dc_type =
    int8_t;
  _dir_dc_type dir_dc;
  using _speed_dc_type =
    int8_t;
  _speed_dc_type speed_dc;
  using _dir_servo_type =
    int16_t;
  _dir_servo_type dir_servo;
  using _stop_lights_type =
    int8_t;
  _stop_lights_type stop_lights;
  using _turn_signals_type =
    int8_t;
  _turn_signals_type turn_signals;

  // setters for named parameter idiom
  Type & set__dir_dc(
    const int8_t & _arg)
  {
    this->dir_dc = _arg;
    return *this;
  }
  Type & set__speed_dc(
    const int8_t & _arg)
  {
    this->speed_dc = _arg;
    return *this;
  }
  Type & set__dir_servo(
    const int16_t & _arg)
  {
    this->dir_servo = _arg;
    return *this;
  }
  Type & set__stop_lights(
    const int8_t & _arg)
  {
    this->stop_lights = _arg;
    return *this;
  }
  Type & set__turn_signals(
    const int8_t & _arg)
  {
    this->turn_signals = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    motor_msgs::msg::MotorCommand_<ContainerAllocator> *;
  using ConstRawPtr =
    const motor_msgs::msg::MotorCommand_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<motor_msgs::msg::MotorCommand_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<motor_msgs::msg::MotorCommand_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      motor_msgs::msg::MotorCommand_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<motor_msgs::msg::MotorCommand_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      motor_msgs::msg::MotorCommand_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<motor_msgs::msg::MotorCommand_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<motor_msgs::msg::MotorCommand_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<motor_msgs::msg::MotorCommand_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__motor_msgs__msg__MotorCommand
    std::shared_ptr<motor_msgs::msg::MotorCommand_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__motor_msgs__msg__MotorCommand
    std::shared_ptr<motor_msgs::msg::MotorCommand_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MotorCommand_ & other) const
  {
    if (this->dir_dc != other.dir_dc) {
      return false;
    }
    if (this->speed_dc != other.speed_dc) {
      return false;
    }
    if (this->dir_servo != other.dir_servo) {
      return false;
    }
    if (this->stop_lights != other.stop_lights) {
      return false;
    }
    if (this->turn_signals != other.turn_signals) {
      return false;
    }
    return true;
  }
  bool operator!=(const MotorCommand_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MotorCommand_

// alias to use template instance with default allocator
using MotorCommand =
  motor_msgs::msg::MotorCommand_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace motor_msgs

#endif  // MOTOR_MSGS__MSG__DETAIL__MOTOR_COMMAND__STRUCT_HPP_
