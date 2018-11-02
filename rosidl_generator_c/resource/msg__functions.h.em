// generated from rosidl_generator_c/resource/msg__functions.h.em
// generated code does not contain a copyright notice

@#######################################################################
@# EmPy template for generating <msg>__functions.h files
@#
@# Context:
@#  - spec (rosidl_parser.MessageSpecification)
@#    Parsed specification of the .msg file
@#  - subfolder (string)
@#    The subfolder / subnamespace of the message
@#    Could be 'msg', 'srv' or 'action'
@#  - get_header_filename_from_msg_name (function)
@#######################################################################
@
@{
from rosidl_generator_c import get_typename_of_base_type
from rosidl_generator_c import value_to_c

header_guard_parts = [
    spec.base_type.pkg_name, subfolder,
    get_header_filename_from_msg_name(spec.base_type.type) + '__functions_h']
header_guard_variable = '__'.join([x.upper() for x in header_guard_parts]) + '_'

msg_typename = '%s__%s__%s' % (spec.base_type.pkg_name, subfolder, spec.base_type.type)
sequence_typename = '%s__Sequence' % msg_typename
}@
#ifndef @(header_guard_variable)
#define @(header_guard_variable)

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_generator_c/visibility_control.h"
#include "@(spec.base_type.pkg_name)/msg/rosidl_generator_c__visibility_control.h"

#include "@(spec.base_type.pkg_name)/@(subfolder)/@(get_header_filename_from_msg_name(spec.base_type.type))__struct.h"

@#######################################################################
@# message functions
@#######################################################################
/// Initialize @(spec.base_type.pkg_name)/@(spec.base_type.type) message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(@(msg_typename))) before
 * or use @(msg_typename)__create() to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_@(spec.base_type.pkg_name)
bool
@(msg_typename)__init(@(msg_typename) * msg);

/// Finalize @(spec.base_type.pkg_name)/@(spec.base_type.type) message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_@(spec.base_type.pkg_name)
void
@(msg_typename)__fini(@(msg_typename) * msg);

/// Create @(spec.base_type.pkg_name)/@(spec.base_type.type) message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls @(msg_typename)__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_@(spec.base_type.pkg_name)
@(msg_typename) *
@(msg_typename)__create();

/// Destroy @(spec.base_type.pkg_name)/@(spec.base_type.type) message.
/**
 * It calls @(msg_typename)__fini() and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_@(spec.base_type.pkg_name)
void
@(msg_typename)__destroy(@(msg_typename) * msg);


@#######################################################################
@# array functions
@#######################################################################
/// Initialize array of @(spec.base_type.pkg_name)/@(spec.base_type.type) messages.
/**
 * It allocates the memory for the number of elements and
 * calls @(msg_typename)__init() for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_@(spec.base_type.pkg_name)
bool
@(sequence_typename)__init(@(sequence_typename) * array, size_t size);

/// Finalize array of @(spec.base_type.pkg_name)/@(spec.base_type.type) messages.
/**
 * It calls @(msg_typename)__fini() for each element of the array and
 * frees the memory for the number of elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_@(spec.base_type.pkg_name)
void
@(sequence_typename)__fini(@(sequence_typename) * array);

/// Create array of @(spec.base_type.pkg_name)/@(spec.base_type.type) messages.
/**
 * It allocates the memory for the array and
 * calls @(sequence_typename)__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_@(spec.base_type.pkg_name)
@(sequence_typename) *
@(sequence_typename)__create(size_t size);

/// Destroy array of @(spec.base_type.pkg_name)/@(spec.base_type.type) messages.
/**
 * It calls @(sequence_typename)__fini() on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_@(spec.base_type.pkg_name)
void
@(sequence_typename)__destroy(@(sequence_typename) * array);

#ifdef __cplusplus
}
#endif

#endif  // @(header_guard_variable)
