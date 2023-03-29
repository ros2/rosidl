# rosidl_runtime_c Features

## Primitive sequence definitions initializations and finalizations

`rosidl_runtime_c` provides default definitions, as well as initialization and finalization functions for the supported rosidl primitive sequence types.
These functions are defined in the following headers:

* `primitives_sequence.h`: C struct definitions for all supported primitive sequence types.
* `primitive_sequence_functions.h`: initialization and finalization functionality for the above defined sequences.
* `sequence_bound.h`: Struct designed to track the upper bounds size of a sequence. Currently unused.

## String and u16string definitions, initializations and finalizations

Like the sequences described above, `rosidl_runtime_c` provides definitions, as well as initialization and finalization functions for 8-bit and 16-bit character strings.

* `string.h`: C struct definition for an 8-bit string.
* `string_functions.h`: initialization and finalization functionality for the rosidl 8-bit string.
* `string_bound.h`: Designed to track the upper bounds size of a string. Currently unused.

* `u16string.h`: C struct definition for a 16-bit string.
* `u16string_functions.h`: initialization and finalization functionality for the rosidl 16-bit string.

## Typesupport structs

This package defines several structs which are utilized for providing access to properties and functionality of different rosidl types.
They are defined for the three main ROS 2 rosidl types:
* Messages (`message_type_support_struct.h`)
* Services (`service_type_support_struct.h`)
* Actions (`action_type_support_struct.h`)

## Type Description Interface Structs

A copy of the output of `rosidl_generator_c` for `type_description_interfaces` messages.
The copy is needed to avoid a circular dependency - `type_description_interfaces` would be used directly for code generation, except as an interface package like any other, it depends on `rosidl_runtime_c` to have its code generated.

Provides base runtime representations of the descriptions of interface types in C.
These can be used in code generation to embed descriptions of types, and can be used for programmatic creation of dynamic types.
The types are renamed so that they do not overlap with `type_description_interfaces` - and can be used in their code generation so that even those messages are self-describing.

Note that end users will typically fetch and use the `type_description_interfaces__msg__TypeDescription` and its members instead of the `rosidl_runtime_c` types.
Functions will be provided in `rcl` that convert between `rosidl_runtime_c__type_description__` and `type_description_interfaces__msg__` types.

See `rosidl/scripts/copy_type_descripiton_generated_sources.bash` for usage and details of how sources are copied, and what modifications are made.
