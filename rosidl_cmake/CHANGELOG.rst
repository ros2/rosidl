^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosidl_cmake
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

4.6.2 (2024-05-13)
------------------

4.6.1 (2024-04-24)
------------------

4.6.0 (2024-04-16)
------------------

4.5.2 (2024-03-27)
------------------
* Improve deprecation notice of rosidl_target_interface to give a hint on how to update the code (`#788 <https://github.com/ros2/rosidl/issues/788>`_)
* Contributors: Alexis Paques

4.5.1 (2024-02-07)
------------------

4.5.0 (2023-12-26)
------------------

4.4.2 (2023-11-06)
------------------

4.4.1 (2023-10-04)
------------------

4.4.0 (2023-09-07)
------------------
* Add rosidl_find_package_idl helper function (`#754 <https://github.com/ros2/rosidl/issues/754>`_)
* Contributors: Mike Purvis

4.3.1 (2023-08-21)
------------------

4.3.0 (2023-07-11)
------------------
* Remove unused splitting of .srv files in CMake (`#753 <https://github.com/ros2/rosidl/issues/753>`_)
* Contributors: Shane Loretz

4.2.0 (2023-06-07)
------------------

4.1.1 (2023-05-11)
------------------

4.1.0 (2023-04-28)
------------------

4.0.0 (2023-04-11)
------------------
* Type Description Codegen and Typesupport  (rep2011) (`#727 <https://github.com/ros2/rosidl/issues/727>`_)
* Type hash in interface codegen (rep2011) (`#722 <https://github.com/ros2/rosidl/issues/722>`_)
* Contributors: Emerson Knapp

3.4.0 (2023-02-13)
------------------
* [service introspection] generate service_event messages (`#700 <https://github.com/ros2/rosidl/issues/700>`_)
* [rolling] Update maintainers - 2022-11-07 (`#717 <https://github.com/ros2/rosidl/issues/717>`_)
* Contributors: Audrow Nash, Brian

3.3.1 (2022-11-02)
------------------
* Skip rosidl_generate_interfaces dependency export on SKIP_INSTALL. (`#708 <https://github.com/ros2/rosidl/issues/708>`_)
* Contributors: Chris Lalancette

3.3.0 (2022-09-08)
------------------
* Move rosidl_cmake Python module to a new package rosidl_pycommon (`#696 <https://github.com/ros2/rosidl/issues/696>`_)
  Deprecate the Python module in rosidl_cmake and move the implementation to the new package rosidl_pycommon.
* Contributors: Jacob Perron

3.2.1 (2022-06-21)
------------------
* Fix comment in camel case conversion function (`#683 <https://github.com/ros2/rosidl/issues/683>`_)
* Protect rosidl_target_interfaces from using NOTFOUND in include_directories (`#679 <https://github.com/ros2/rosidl/issues/679>`_)
* Contributors: Jose Luis Rivero, Shane Loretz

3.2.0 (2022-05-04)
------------------

3.1.3 (2022-04-08)
------------------

3.1.2 (2022-04-05)
------------------

3.1.1 (2022-03-28)
------------------
* Make rosidl_get_typesupport_target return -NOTFOUND instead of FATAL_ERROR (`#672 <https://github.com/ros2/rosidl/issues/672>`_)
* Contributors: Shane Loretz

3.1.0 (2022-03-01)
------------------
* Add introspection typesupport tests for C/C++ messages (`#651 <https://github.com/ros2/rosidl/issues/651>`_)
* Contributors: Michel Hidalgo

3.0.1 (2022-01-13)
------------------
* Use target output name for exporting typesupport library (`#625 <https://github.com/ros2/rosidl/issues/625>`_)
* Contributors: Jonathan Selling

3.0.0 (2021-11-05)
------------------
* Update package maintainers (`#624 <https://github.com/ros2/rosidl/issues/624>`_)
* Contributors: Michel Hidalgo

2.5.0 (2021-08-10)
------------------
* Revert "Bundle and ensure the exportation of rosidl generated targets" (`#611 <https://github.com/ros2/rosidl/issues/611>`_)
* Add rosidl_get_typesupport_target and deprecate rosidl_target_interfaces (`#606 <https://github.com/ros2/rosidl/issues/606>`_)
* Contributors: Michel Hidalgo, Shane Loretz

2.4.0 (2021-07-12)
------------------
* Bundle and ensure the exportation of rosidl generated targets (`#601 <https://github.com/ros2/rosidl/issues/601>`_)
* Contributors: Michel Hidalgo

2.3.0 (2021-06-11)
------------------

2.2.1 (2021-04-06)
------------------

2.2.0 (2021-03-18)
------------------

2.1.0 (2021-03-09)
------------------
* Shorten some excessively long lines of CMake (`#571 <https://github.com/ros2/rosidl/issues/571>`_)
* Contributors: Scott K Logan

2.0.3 (2021-01-25)
------------------

2.0.2 (2020-12-08)
------------------
* Update the maintainers of this repository. (`#536 <https://github.com/ros2/rosidl/issues/536>`_)
* Contributors: Chris Lalancette

2.0.1 (2020-09-28)
------------------

2.0.0 (2020-09-24)
------------------

1.1.0 (2020-08-17)
------------------
* Modifications to python generator lib to return generated files (`#511 <https://github.com/ros2/rosidl/issues/511>`_)
* Contributors: Alex Tyshka

1.0.1 (2020-06-03)
------------------

1.0.0 (2020-05-22)
------------------
* Fix variable suffix in rosidl_export_typesupport_targets (`#483 <https://github.com/ros2/rosidl/issues/483>`_)
* Contributors: Ivan Santiago Paunovic

0.9.2 (2020-05-19)
------------------

0.9.1 (2020-05-08)
------------------
* use typesuport targets instead of libraries (`#478 <https://github.com/ros2/rosidl/issues/478>`_)
* Contributors: Dirk Thomas

0.9.0 (2020-04-24)
------------------
* Use f-string (`#436 <https://github.com/ros2/rosidl/issues/436>`_)
* Contributors: Dirk Thomas

0.8.2 (2020-01-17)
------------------

0.8.1 (2019-10-23)
------------------

0.8.0 (2019-09-24)
------------------
* use latin-1 encoding when reading/writing .idl files, prepend BOM to generated C/C++ files when necessary (`#391 <https://github.com/ros2/rosidl/issues/391>`_)
* fix CMake linter warning (`#382 <https://github.com/ros2/rosidl/issues/382>`_)
* Contributors: Dirk Thomas
