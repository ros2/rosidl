^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosidl_generator_cpp
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.2.2 (2022-04-28)
------------------

2.2.1 (2021-04-06)
------------------

2.2.0 (2021-03-18)
------------------
* Expose C++ code generation via rosidl generate CLI (`#570 <https://github.com/ros2/rosidl/issues/570>`_)
* Contributors: Michel Hidalgo

2.1.0 (2021-03-09)
------------------
* Switch to std::allocator_traits. (`#564 <https://github.com/ros2/rosidl/issues/564>`_)
* Contributors: Chris Lalancette

2.0.3 (2021-01-25)
------------------

2.0.2 (2020-12-08)
------------------
* Remove unnecessary assert on pointer created with new (`#555 <https://github.com/ros2/rosidl/issues/555>`_)
* Use ASSERT_TRUE to check for nullptr. (`#543 <https://github.com/ros2/rosidl/issues/543>`_)
* Update the maintainers of this repository. (`#536 <https://github.com/ros2/rosidl/issues/536>`_)
* Contributors: Chris Lalancette, Stephen Brawner

2.0.1 (2020-09-28)
------------------

2.0.0 (2020-09-24)
------------------
* Add to_yaml() function for C++ messages (`#527 <https://github.com/ros2/rosidl/issues/527>`_)
* Contributors: Devin Bonnie, Dirk Thomas

1.1.0 (2020-08-17)
------------------
* Add function for getting a types fully qualified name (`#514 <https://github.com/ros2/rosidl/issues/514>`_)
* Declaring is_message in namespace rosidl_generator_traits (`#512 <https://github.com/ros2/rosidl/issues/512>`_)
* Contributors: Jacob Perron, Sebastian Höffner

1.0.1 (2020-06-03)
------------------

1.0.0 (2020-05-22)
------------------

0.9.2 (2020-05-19)
------------------

0.9.1 (2020-05-08)
------------------
* move test which only uses rosidl_runtime_cpp into that package (`#481 <https://github.com/ros2/rosidl/issues/481>`_)
* Contributors: Dirk Thomas

0.9.0 (2020-04-24)
------------------
* Export targets in addition to include directories / libraries (`#473 <https://github.com/ros2/rosidl/issues/473>`_)
* Move non-entry point headers into detail subdirectory (`#461 <https://github.com/ros2/rosidl/issues/461>`_)
* Only export ament_cmake_core instead of ament_cmake (`#459 <https://github.com/ros2/rosidl/issues/459>`_)
* Rename rosidl_namespace_cpp namespace (`#456 <https://github.com/ros2/rosidl/issues/456>`_)
* Split rosidl_generator_c and rosidl_generator_cpp in two: rosidl_generator_x and rosidl_runtime_x (`#442 <https://github.com/ros2/rosidl/issues/442>`_)
* Add a utility for rigorously initializing a message instance (`#448 <https://github.com/ros2/rosidl/issues/448>`_)
* Avoid setter for empty struct dummy member (`#455 <https://github.com/ros2/rosidl/issues/455>`_)
* Code style only: wrap after open parenthesis if not in one line (`#435 <https://github.com/ros2/rosidl/issues/435>`_)
* Use f-string (`#436 <https://github.com/ros2/rosidl/issues/436>`_)
* Contributors: Alejandro Hernández Cordero, Dirk Thomas, Grey

0.8.2 (2020-01-17)
------------------

0.8.1 (2019-10-23)
------------------
* Add is_message trait in support of tf2 conversions (`#412 <https://github.com/ros2/rosidl/issues/412>`_)
* Contributors: Michael Carroll

0.8.0 (2019-09-24)
------------------
* Update guard against common Windows preprocessor definitions (`#401 <https://github.com/ros2/rosidl/issues/401>`_)
* Update tests for new message types in test_interface_files (`#397 <https://github.com/ros2/rosidl/issues/397>`_)
* use latin-1 encoding when reading/writing .idl files, prepend BOM to generated C/C++ files when necessary (`#391 <https://github.com/ros2/rosidl/issues/391>`_)
* Add emplace_back, move_assignment to BoundedVector (`#385 <https://github.com/ros2/rosidl/issues/385>`_)
* fix cpp generator and introspection ts for long double (`#383 <https://github.com/ros2/rosidl/issues/383>`_)
* Contributors: Dirk Thomas, Jacob Perron, Siddharth Kucheria, cho3
