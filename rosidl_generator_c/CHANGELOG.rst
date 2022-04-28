^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosidl_generator_c
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.2.2 (2022-04-28)
------------------
* Galactic backport of `#648 <https://github.com/ros2/rosidl/issues/648>`_ and `#650 <https://github.com/ros2/rosidl/issues/650>`_ (`#668 <https://github.com/ros2/rosidl/issues/668>`_)
* Contributors: Nikolai Morin

2.2.1 (2021-04-06)
------------------

2.2.0 (2021-03-18)
------------------
* Expose C code generation via rosidl generate CLI (`#569 <https://github.com/ros2/rosidl/issues/569>`_)
* Contributors: Michel Hidalgo

2.1.0 (2021-03-09)
------------------

2.0.3 (2021-01-25)
------------------

2.0.2 (2020-12-08)
------------------
* Strip action service suffixes from C include prefix (`#538 <https://github.com/ros2/rosidl/issues/538>`_)
* Update the maintainers of this repository. (`#536 <https://github.com/ros2/rosidl/issues/536>`_)
* Contributors: Chris Lalancette, Jacob Perron

2.0.1 (2020-09-28)
------------------

2.0.0 (2020-09-24)
------------------
* Fix the declared language for a few packages (`#530 <https://github.com/ros2/rosidl/issues/530>`_)
* Contributors: Scott K Logan

1.1.0 (2020-08-17)
------------------
* Do not depend on rosidl_runtime_c when tests are disabled (`#503 <https://github.com/ros2/rosidl/issues/503>`_)
* Contributors: Ben Wolsieffer

1.0.1 (2020-06-03)
------------------

1.0.0 (2020-05-22)
------------------

0.9.2 (2020-05-19)
------------------

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
