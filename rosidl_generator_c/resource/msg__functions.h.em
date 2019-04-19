@# Included from rosidl_generator_c/resource/idl__functions.h.em
@{
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
