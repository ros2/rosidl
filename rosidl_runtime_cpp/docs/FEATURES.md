# rosidl_runtime_cpp Features

## Typesupport function handler

`rosidl_runtime_cpp` provides several templated function declarations for getting the type support handler.
They are meant to be defined in each rosidl definition's C++ generated code.
They are defined for both the `rosidl_runtime_cpp` and `rosidl_typesupport_cpp` namespaces.

`rosidl_runtime_cpp`
* Messages (`message_type_support_decl.hpp`)
* Services (`service_type_support_decl.hpp`)
* Actions (`action_type_support_decl.hpp`)

`rosidl_typesupport_cpp`
* Messages (`message_type_support.hpp`)
* Services (`service_type_support.hpp`)
* Actions (`action_type_support.hpp`)

## C++ Type Traits

`rosidl_runtime_c` provides numerous C++ Type Traits for rosidl interfaces in `traits.hpp`.
These type traits are useful for providing compile-time checks of rosidl C++ types.

## Bounded Vector

Provided in `bounded_vector.hpp` is an implementation of a std::vector like container class, that has a bounded capacity.
This class is utilized by rosidl types that declare a bounded sequence.

## Type Description Interface Structs

A copy of the output of `rosidl_generator_cpp` for `type_description_interfaces` messages.
The copy is needed to avoid a circular dependency - `type_description_interfaces` would be used directly for code generation, except as an interface package like any other, it depends on `rosidl_runtime_cpp` to have its code generated.

Provides base runtime representations of the descriptions of interface types in C++.
These can be used in code generation to embed descriptions of types, and can be used for programmatic creation of dynamic types.
The types are renamed so that they do not overlap with `type_description_interfaces` - and can be used in their code generation so that even those messages are self-describing.

Note that end users will typically fetch and use the `type_description_interfaces::msg::TypeDescription` and its members instead of the `rosidl_runtime_cpp` types.
Functions will be provided in `rclcpp` that convert between `rosidl_runtime_cpp::type_description` and `type_description_interfaces::msg` types.

See `rosidl/scripts/copy_type_descripiton_generated_sources.bash` for usage and details of how sources are copied/updated, and what modifications are made.
