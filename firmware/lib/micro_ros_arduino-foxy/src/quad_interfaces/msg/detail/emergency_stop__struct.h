// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from quad_interfaces:msg/EmergencyStop.idl
// generated code does not contain a copyright notice

#ifndef QUAD_INTERFACES__MSG__DETAIL__EMERGENCY_STOP__STRUCT_H_
#define QUAD_INTERFACES__MSG__DETAIL__EMERGENCY_STOP__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/EmergencyStop in the package quad_interfaces.
typedef struct quad_interfaces__msg__EmergencyStop
{
  bool emergency_stop;
} quad_interfaces__msg__EmergencyStop;

// Struct for a sequence of quad_interfaces__msg__EmergencyStop.
typedef struct quad_interfaces__msg__EmergencyStop__Sequence
{
  quad_interfaces__msg__EmergencyStop * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} quad_interfaces__msg__EmergencyStop__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // QUAD_INTERFACES__MSG__DETAIL__EMERGENCY_STOP__STRUCT_H_
