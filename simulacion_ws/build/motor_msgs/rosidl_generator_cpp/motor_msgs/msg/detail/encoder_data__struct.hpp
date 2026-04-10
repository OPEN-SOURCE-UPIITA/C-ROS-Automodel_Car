// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from motor_msgs:msg/EncoderData.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "motor_msgs/msg/encoder_data.hpp"


#ifndef MOTOR_MSGS__MSG__DETAIL__ENCODER_DATA__STRUCT_HPP_
#define MOTOR_MSGS__MSG__DETAIL__ENCODER_DATA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__motor_msgs__msg__EncoderData __attribute__((deprecated))
#else
# define DEPRECATED__motor_msgs__msg__EncoderData __declspec(deprecated)
#endif

namespace motor_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct EncoderData_
{
  using Type = EncoderData_<ContainerAllocator>;

  explicit EncoderData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->vel_m1 = 0l;
      this->dir_m1 = 0;
      this->vel_m2 = 0l;
      this->dir_m2 = 0;
    }
  }

  explicit EncoderData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->vel_m1 = 0l;
      this->dir_m1 = 0;
      this->vel_m2 = 0l;
      this->dir_m2 = 0;
    }
  }

  // field types and members
  using _vel_m1_type =
    int32_t;
  _vel_m1_type vel_m1;
  using _dir_m1_type =
    int8_t;
  _dir_m1_type dir_m1;
  using _vel_m2_type =
    int32_t;
  _vel_m2_type vel_m2;
  using _dir_m2_type =
    int8_t;
  _dir_m2_type dir_m2;

  // setters for named parameter idiom
  Type & set__vel_m1(
    const int32_t & _arg)
  {
    this->vel_m1 = _arg;
    return *this;
  }
  Type & set__dir_m1(
    const int8_t & _arg)
  {
    this->dir_m1 = _arg;
    return *this;
  }
  Type & set__vel_m2(
    const int32_t & _arg)
  {
    this->vel_m2 = _arg;
    return *this;
  }
  Type & set__dir_m2(
    const int8_t & _arg)
  {
    this->dir_m2 = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    motor_msgs::msg::EncoderData_<ContainerAllocator> *;
  using ConstRawPtr =
    const motor_msgs::msg::EncoderData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<motor_msgs::msg::EncoderData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<motor_msgs::msg::EncoderData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      motor_msgs::msg::EncoderData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<motor_msgs::msg::EncoderData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      motor_msgs::msg::EncoderData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<motor_msgs::msg::EncoderData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<motor_msgs::msg::EncoderData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<motor_msgs::msg::EncoderData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__motor_msgs__msg__EncoderData
    std::shared_ptr<motor_msgs::msg::EncoderData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__motor_msgs__msg__EncoderData
    std::shared_ptr<motor_msgs::msg::EncoderData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const EncoderData_ & other) const
  {
    if (this->vel_m1 != other.vel_m1) {
      return false;
    }
    if (this->dir_m1 != other.dir_m1) {
      return false;
    }
    if (this->vel_m2 != other.vel_m2) {
      return false;
    }
    if (this->dir_m2 != other.dir_m2) {
      return false;
    }
    return true;
  }
  bool operator!=(const EncoderData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct EncoderData_

// alias to use template instance with default allocator
using EncoderData =
  motor_msgs::msg::EncoderData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace motor_msgs

#endif  // MOTOR_MSGS__MSG__DETAIL__ENCODER_DATA__STRUCT_HPP_
