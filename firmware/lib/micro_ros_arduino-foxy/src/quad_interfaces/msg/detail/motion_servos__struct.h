// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from quad_interfaces:msg/MotionServos.idl
// generated code does not contain a copyright notice

#ifndef QUAD_INTERFACES__MSG__DETAIL__MOTION_SERVOS__STRUCT_H_
#define QUAD_INTERFACES__MSG__DETAIL__MOTION_SERVOS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/MotionServos in the package quad_interfaces.
typedef struct quad_interfaces__msg__MotionServos
{
  bool enable[12];
  int16_t pulse_width[12];
} quad_interfaces__msg__MotionServos;

// Struct for a sequence of quad_interfaces__msg__MotionServos.
typedef struct quad_interfaces__msg__MotionServos__Sequence
{
  quad_interfaces__msg__MotionServos * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} quad_interfaces__msg__MotionServos__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // QUAD_INTERFACES__MSG__DETAIL__MOTION_SERVOS__STRUCT_H_
