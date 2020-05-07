# `rosidl_runtime_cpp` Features

## Typesupport function handler  

`rosidl_runtime_cpp` provides several templated function declarations for getting the type support handler.
They are meant to be defined in each rosidl definition's C++ generated code.
They are defined for both the `rosidl_runtime_cpp` and `rosidl_typesupport_cpp` namespaces.

`rosidl_runtime_cpp`
* [Messages](../include/rosidl_runtime_cpp/message_type_support_decl.hpp)
* [Services](../include/rosidl_runtime_cpp/service_type_support_decl.hpp)
* [Actions](../include/rosidl_runtime_cpp/action_type_support_decl.hpp)

`rosidl_typesupport_cpp`
* [Messages](../include/rosidl_typesupport_cpp/message_type_support.hpp)
* [Services](../include/rosidl_typesupport_cpp/service_type_support.hpp)
* [Actions](../include/rosidl_typesupport_cpp/action_type_support.hpp)

## C++ Type Traits

`rosidl_runtime_c` provides numerous C++ Type Traits for rosidl interfaces in [traits.hpp](../include/rosidl_runtime_cpp/traits.hpp).
These type traits are useful for providing compile-time checks of rosidl C++ types.

## Bounded Vector

Provided in [bounded_vector.hpp](../include/rosidl_runtime_cpp/bounded_vector.hpp) is an implementation of a std::vector like container class, that has a bounded capacity.
This class is utilized by rosidl types that declare a bounded sequence.
