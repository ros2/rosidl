^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosidl_generator_cpp
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
