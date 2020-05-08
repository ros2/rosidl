^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosidl_generator_c
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.9.1 (2020-05-08)
------------------

0.9.0 (2020-04-24)
------------------
* Export targets in addition to include directories / libraries (`#473 <https://github.com/ros2/rosidl/issues/473>`_)
* Move non-entry point headers into detail subdirectory (`#461 <https://github.com/ros2/rosidl/issues/461>`_)
* Rename rosidl_generator_c 'namespace' to rosidl_runtime_c (`#458 <https://github.com/ros2/rosidl/issues/458>`_)
* Only export ament_cmake_core instead of ament_cmake (`#459 <https://github.com/ros2/rosidl/issues/459>`_)
* Split rosidl_generator_c and rosidl_generator_cpp in two: rosidl_generator_x and rosidl_runtime_x (`#442 <https://github.com/ros2/rosidl/issues/442>`_)
* Added rosidl_generator_c as a member of group rosidl_runtime_packages (`#440 <https://github.com/ros2/rosidl/issues/440>`_)
* Style update to match uncrustify with explicit language (`#439 <https://github.com/ros2/rosidl/issues/439>`_)
* Code style only: wrap after open parenthesis if not in one line (`#435 <https://github.com/ros2/rosidl/issues/435>`_)
* Use f-string (`#436 <https://github.com/ros2/rosidl/issues/436>`_)
* Move repeated logic for C include prefix into common function (`#432 <https://github.com/ros2/rosidl/issues/432>`_)
* Contributors: Alejandro Hernández Cordero, Dirk Thomas, Jacob Perron

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
