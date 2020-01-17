^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosidl_generator_c
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.8.2 (2020-01-17)
------------------
* Fix double free issue when initialization is failed (`#423 <https://github.com/ros2/rosidl/issues/423>`_)
* Contributors: DongheeYe

0.8.1 (2019-10-23)
------------------

0.8.0 (2019-09-24)
------------------
* [rosidl_generator_c] Updated tests for new msg types from test_interface_files (`#398 <https://github.com/ros2/rosidl/issues/398>`_)
* use latin-1 encoding when reading/writing .idl files, prepend BOM to generated C/C++ files when necessary (`#391 <https://github.com/ros2/rosidl/issues/391>`_)
* Set _FOUND to trick ament_target_dependencies() for test (`#396 <https://github.com/ros2/rosidl/issues/396>`_)
* Contributors: Dirk Thomas, Shane Loretz, Siddharth Kucheria
