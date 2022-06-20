@# Included from rosidl_generator_c/resource/idl__functions.h.em
@{
from rosidl_generator_c import idl_enumeration_type_sequence_to_c_typename
from rosidl_generator_c import idl_enumeration_type_to_c_typename
from rosidl_generator_c import idl_structure_type_sequence_to_c_typename
from rosidl_generator_c import idl_structure_type_to_c_typename
from rosidl_generator_c import interface_path_to_string

message_typename = idl_structure_type_to_c_typename(message.structure.namespaced_type)
array_typename = idl_structure_type_sequence_to_c_typename(
    message.structure.namespaced_type)
}@
@#######################################################################
@# message functions
@#######################################################################
/// Initialize @(interface_path_to_string(interface_path)) message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * @(message_typename)
 * )) before or use
 * @(message_typename)__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_@(package_name)
bool
@(message_typename)__init(@(message_typename) * msg);

/// Finalize @(interface_path_to_string(interface_path)) message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_@(package_name)
void
@(message_typename)__fini(@(message_typename) * msg);

/// Create @(interface_path_to_string(interface_path)) message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * @(message_typename)__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_@(package_name)
@(message_typename) *
@(message_typename)__create();

/// Destroy @(interface_path_to_string(interface_path)) message.
/**
 * It calls
 * @(message_typename)__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_@(package_name)
void
@(message_typename)__destroy(@(message_typename) * msg);

/// Check for @(interface_path_to_string(interface_path)) message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_@(package_name)
bool
@(message_typename)__are_equal(const @(message_typename) * lhs, const @(message_typename) * rhs);

/// Copy a @(interface_path_to_string(interface_path)) message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_@(package_name)
bool
@(message_typename)__copy(
  const @(message_typename) * input,
  @(message_typename) * output);

@#######################################################################
@# array functions
@#######################################################################
/// Initialize array of @(interface_path_to_string(interface_path)) messages.
/**
 * It allocates the memory for the number of elements and calls
 * @(message_typename)__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_@(package_name)
bool
@(array_typename)__init(@(array_typename) * array, size_t size);

/// Finalize array of @(interface_path_to_string(interface_path)) messages.
/**
 * It calls
 * @(message_typename)__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_@(package_name)
void
@(array_typename)__fini(@(array_typename) * array);

/// Create array of @(interface_path_to_string(interface_path)) messages.
/**
 * It allocates the memory for the array and calls
 * @(array_typename)__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_@(package_name)
@(array_typename) *
@(array_typename)__create(size_t size);

/// Destroy array of @(interface_path_to_string(interface_path)) messages.
/**
 * It calls
 * @(array_typename)__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_@(package_name)
void
@(array_typename)__destroy(@(array_typename) * array);

/// Check for @(interface_path_to_string(interface_path)) message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_@(package_name)
bool
@(array_typename)__are_equal(const @(array_typename) * lhs, const @(array_typename) * rhs);

/// Copy an array of @(interface_path_to_string(interface_path)) messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_@(package_name)
bool
@(array_typename)__copy(
  const @(array_typename) * input,
  @(array_typename) * output);

@#######################################################################
@# message enums functions
@#######################################################################
@[for enum in message.enumerations]@
@{
__enum_typename = idl_enumeration_type_to_c_typename(enum.enumeration_type)
__enum_sequence_typename = idl_enumeration_type_sequence_to_c_typename(enum.enumeration_type)
}@
/// Initialize sequence of @(__enum_typename).
/**
 * It allocates the memory for the number of elements.
 * \param[in,out] sequence The allocated sequence pointer.
 * \param[in] size The size / capacity of the sequence.
 * \return true if initialization was successful, otherwise false
 * If the sequence pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_@(package_name)
bool
@(__enum_sequence_typename)__init(@(__enum_sequence_typename) * sequence, size_t size);

/// Finalize sequence of @(__enum_typename).
/**
 * \param[in,out] sequence The allocated sequence pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_@(package_name)
void
@(__enum_sequence_typename)__fini(@(__enum_sequence_typename) * sequence);

/// Check for @(__enum_typename) array equality.
/**
 * \param[in] lhs The array on the left hand side of the equality operator.
 * \param[in] rhs The array on the right hand side of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_@(package_name)
bool
@(__enum_sequence_typename)__are_equal(const @(__enum_sequence_typename) * lhs, const @(__enum_sequence_typename) * rhs);

/// Copy an array of @(__enum_typename).
/**
 * This function performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_@(package_name)
bool
@(__enum_sequence_typename)__copy(
  const @(__enum_sequence_typename) * input,
  @(__enum_sequence_typename) * output);
@[  end for]@
