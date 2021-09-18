// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from quad_interfaces:msg/AuxServos.idl
// generated code does not contain a copyright notice

#ifndef QUAD_INTERFACES__MSG__DETAIL__AUX_SERVOS__FUNCTIONS_H_
#define QUAD_INTERFACES__MSG__DETAIL__AUX_SERVOS__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "quad_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "quad_interfaces/msg/detail/aux_servos__struct.h"

/// Initialize msg/AuxServos message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * quad_interfaces__msg__AuxServos
 * )) before or use
 * quad_interfaces__msg__AuxServos__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_quad_interfaces
bool
quad_interfaces__msg__AuxServos__init(quad_interfaces__msg__AuxServos * msg);

/// Finalize msg/AuxServos message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_quad_interfaces
void
quad_interfaces__msg__AuxServos__fini(quad_interfaces__msg__AuxServos * msg);

/// Create msg/AuxServos message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * quad_interfaces__msg__AuxServos__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_quad_interfaces
quad_interfaces__msg__AuxServos *
quad_interfaces__msg__AuxServos__create();

/// Destroy msg/AuxServos message.
/**
 * It calls
 * quad_interfaces__msg__AuxServos__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_quad_interfaces
void
quad_interfaces__msg__AuxServos__destroy(quad_interfaces__msg__AuxServos * msg);


/// Initialize array of msg/AuxServos messages.
/**
 * It allocates the memory for the number of elements and calls
 * quad_interfaces__msg__AuxServos__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_quad_interfaces
bool
quad_interfaces__msg__AuxServos__Sequence__init(quad_interfaces__msg__AuxServos__Sequence * array, size_t size);

/// Finalize array of msg/AuxServos messages.
/**
 * It calls
 * quad_interfaces__msg__AuxServos__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_quad_interfaces
void
quad_interfaces__msg__AuxServos__Sequence__fini(quad_interfaces__msg__AuxServos__Sequence * array);

/// Create array of msg/AuxServos messages.
/**
 * It allocates the memory for the array and calls
 * quad_interfaces__msg__AuxServos__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_quad_interfaces
quad_interfaces__msg__AuxServos__Sequence *
quad_interfaces__msg__AuxServos__Sequence__create(size_t size);

/// Destroy array of msg/AuxServos messages.
/**
 * It calls
 * quad_interfaces__msg__AuxServos__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_quad_interfaces
void
quad_interfaces__msg__AuxServos__Sequence__destroy(quad_interfaces__msg__AuxServos__Sequence * array);

#ifdef __cplusplus
}
#endif

#endif  // QUAD_INTERFACES__MSG__DETAIL__AUX_SERVOS__FUNCTIONS_H_
