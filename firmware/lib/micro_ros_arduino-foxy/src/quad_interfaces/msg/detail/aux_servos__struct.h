// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from quad_interfaces:msg/AuxServos.idl
// generated code does not contain a copyright notice

#ifndef QUAD_INTERFACES__MSG__DETAIL__AUX_SERVOS__STRUCT_H_
#define QUAD_INTERFACES__MSG__DETAIL__AUX_SERVOS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/AuxServos in the package quad_interfaces.
typedef struct quad_interfaces__msg__AuxServos
{
  bool enable[4];
  float angle[4];
} quad_interfaces__msg__AuxServos;

// Struct for a sequence of quad_interfaces__msg__AuxServos.
typedef struct quad_interfaces__msg__AuxServos__Sequence
{
  quad_interfaces__msg__AuxServos * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} quad_interfaces__msg__AuxServos__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // QUAD_INTERFACES__MSG__DETAIL__AUX_SERVOS__STRUCT_H_
