// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from motor_msgs:msg/MotorCommand.idl
// generated code does not contain a copyright notice
#include "motor_msgs/msg/detail/motor_command__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
motor_msgs__msg__MotorCommand__init(motor_msgs__msg__MotorCommand * msg)
{
  if (!msg) {
    return false;
  }
  // dir_dc
  // speed_dc
  // dir_servo
  // stop_lights
  // turn_signals
  return true;
}

void
motor_msgs__msg__MotorCommand__fini(motor_msgs__msg__MotorCommand * msg)
{
  if (!msg) {
    return;
  }
  // dir_dc
  // speed_dc
  // dir_servo
  // stop_lights
  // turn_signals
}

bool
motor_msgs__msg__MotorCommand__are_equal(const motor_msgs__msg__MotorCommand * lhs, const motor_msgs__msg__MotorCommand * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // dir_dc
  if (lhs->dir_dc != rhs->dir_dc) {
    return false;
  }
  // speed_dc
  if (lhs->speed_dc != rhs->speed_dc) {
    return false;
  }
  // dir_servo
  if (lhs->dir_servo != rhs->dir_servo) {
    return false;
  }
  // stop_lights
  if (lhs->stop_lights != rhs->stop_lights) {
    return false;
  }
  // turn_signals
  if (lhs->turn_signals != rhs->turn_signals) {
    return false;
  }
  return true;
}

bool
motor_msgs__msg__MotorCommand__copy(
  const motor_msgs__msg__MotorCommand * input,
  motor_msgs__msg__MotorCommand * output)
{
  if (!input || !output) {
    return false;
  }
  // dir_dc
  output->dir_dc = input->dir_dc;
  // speed_dc
  output->speed_dc = input->speed_dc;
  // dir_servo
  output->dir_servo = input->dir_servo;
  // stop_lights
  output->stop_lights = input->stop_lights;
  // turn_signals
  output->turn_signals = input->turn_signals;
  return true;
}

motor_msgs__msg__MotorCommand *
motor_msgs__msg__MotorCommand__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  motor_msgs__msg__MotorCommand * msg = (motor_msgs__msg__MotorCommand *)allocator.allocate(sizeof(motor_msgs__msg__MotorCommand), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(motor_msgs__msg__MotorCommand));
  bool success = motor_msgs__msg__MotorCommand__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
motor_msgs__msg__MotorCommand__destroy(motor_msgs__msg__MotorCommand * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    motor_msgs__msg__MotorCommand__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
motor_msgs__msg__MotorCommand__Sequence__init(motor_msgs__msg__MotorCommand__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  motor_msgs__msg__MotorCommand * data = NULL;

  if (size) {
    data = (motor_msgs__msg__MotorCommand *)allocator.zero_allocate(size, sizeof(motor_msgs__msg__MotorCommand), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = motor_msgs__msg__MotorCommand__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        motor_msgs__msg__MotorCommand__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
motor_msgs__msg__MotorCommand__Sequence__fini(motor_msgs__msg__MotorCommand__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      motor_msgs__msg__MotorCommand__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

motor_msgs__msg__MotorCommand__Sequence *
motor_msgs__msg__MotorCommand__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  motor_msgs__msg__MotorCommand__Sequence * array = (motor_msgs__msg__MotorCommand__Sequence *)allocator.allocate(sizeof(motor_msgs__msg__MotorCommand__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = motor_msgs__msg__MotorCommand__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
motor_msgs__msg__MotorCommand__Sequence__destroy(motor_msgs__msg__MotorCommand__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    motor_msgs__msg__MotorCommand__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
motor_msgs__msg__MotorCommand__Sequence__are_equal(const motor_msgs__msg__MotorCommand__Sequence * lhs, const motor_msgs__msg__MotorCommand__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!motor_msgs__msg__MotorCommand__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
motor_msgs__msg__MotorCommand__Sequence__copy(
  const motor_msgs__msg__MotorCommand__Sequence * input,
  motor_msgs__msg__MotorCommand__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(motor_msgs__msg__MotorCommand);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    motor_msgs__msg__MotorCommand * data =
      (motor_msgs__msg__MotorCommand *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!motor_msgs__msg__MotorCommand__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          motor_msgs__msg__MotorCommand__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!motor_msgs__msg__MotorCommand__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
