^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosidl_typesupport_introspection_c
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

4.3.1 (2023-08-21)
------------------
* update comment (`#757 <https://github.com/ros2/rosidl/issues/757>`_)
* Contributors: Chen Lihui

4.3.0 (2023-07-11)
------------------

4.2.0 (2023-06-07)
------------------

4.1.1 (2023-05-11)
------------------

4.1.0 (2023-04-28)
------------------

4.0.0 (2023-04-11)
------------------
* Type Description Codegen and Typesupport  (rep2011) (`#727 <https://github.com/ros2/rosidl/issues/727>`_)
* Expose type hash on typesupports (rep2011) (`#729 <https://github.com/ros2/rosidl/issues/729>`_)
* Type hash in interface codegen (rep2011) (`#722 <https://github.com/ros2/rosidl/issues/722>`_)
* Contributors: Emerson Knapp

3.4.0 (2023-02-13)
------------------
* [service introspection] generate service_event messages (`#700 <https://github.com/ros2/rosidl/issues/700>`_)
* [rolling] Update maintainers - 2022-11-07 (`#717 <https://github.com/ros2/rosidl/issues/717>`_)
* Contributors: Audrow Nash, Brian

3.3.1 (2022-11-02)
------------------

3.3.0 (2022-09-08)
------------------
* Move rosidl_cmake Python module to a new package rosidl_pycommon (`#696 <https://github.com/ros2/rosidl/issues/696>`_)
  Deprecate the Python module in rosidl_cmake and move the implementation to the new package rosidl_pycommon.
* Fix build export dependencies in C introspection package (`#695 <https://github.com/ros2/rosidl/issues/695>`_)
* Add namespaced ALIAS target to easily consume generated libraries via add_subdirectory (`#605 <https://github.com/ros2/rosidl/issues/605>`_)
* Contributors: Jacob Perron, Silvio Traversaro

3.2.1 (2022-06-21)
------------------

3.2.0 (2022-05-04)
------------------

3.1.3 (2022-04-08)
------------------

3.1.2 (2022-04-05)
------------------

3.1.1 (2022-03-28)
------------------
* Install generated headers to include/${PROJECT_NAME} (`#670 <https://github.com/ros2/rosidl/issues/670>`_)
* Misc cleanup in the rosidl generator extensions (`#662 <https://github.com/ros2/rosidl/issues/662>`_)
* Contributors: Shane Loretz

3.1.0 (2022-03-01)
------------------
* De-duplicate Quality Level from README and QUALITY_DECLARATION (`#661 <https://github.com/ros2/rosidl/issues/661>`_)
* Update Quality declaration to level 1 in README for instrospection pkgs (`#659 <https://github.com/ros2/rosidl/issues/659>`_)
* Install headers to include/${PROJECT_NAME} (`#658 <https://github.com/ros2/rosidl/issues/658>`_)
* Move rosidl_typesupport_introspection_cpp quality declaration to Q1 (`#657 <https://github.com/ros2/rosidl/issues/657>`_)
* Move rosidl_typesupport_introspection_c quality declaration to Q1  (`#656 <https://github.com/ros2/rosidl/issues/656>`_)
* add documentation for generators and API (`#646 <https://github.com/ros2/rosidl/issues/646>`_)
* Rework nested types' items introspection in C and C++ (`#652 <https://github.com/ros2/rosidl/issues/652>`_)
* Contributors: Jose Luis Rivero, Michel Hidalgo, Shane Loretz

3.0.1 (2022-01-13)
------------------

3.0.0 (2021-11-05)
------------------
* Fix up the documentation for rosidl_typesupport_introspection_c (`#628 <https://github.com/ros2/rosidl/issues/628>`_)
* Update package maintainers (`#624 <https://github.com/ros2/rosidl/issues/624>`_)
* Quality Declaration for typesupport_introspection (`#621 <https://github.com/ros2/rosidl/issues/621>`_)
* Make rosidl packages use FindPython3 instead of FindPythonInterp (`#612 <https://github.com/ros2/rosidl/issues/612>`_)
* Contributors: Chris Lalancette, Michel Hidalgo, Shane Loretz, eboasson

2.5.0 (2021-08-10)
------------------
* Revert "Bundle and ensure the exportation of rosidl generated targets" (`#611 <https://github.com/ros2/rosidl/issues/611>`_)
* Contributors: Michel Hidalgo

2.4.0 (2021-07-12)
------------------
* Bundle and ensure the exportation of rosidl generated targets (`#601 <https://github.com/ros2/rosidl/issues/601>`_)
* Update function prefix (`#596 <https://github.com/ros2/rosidl/issues/596>`_)
* Contributors: Michel Hidalgo, Pablo Garrido

2.3.0 (2021-06-11)
------------------

2.2.1 (2021-04-06)
------------------

2.2.0 (2021-03-18)
------------------
* Expose C introspection typesupport generation via rosidl generate CLI (`#572 <https://github.com/ros2/rosidl/issues/572>`_)
* Contributors: Michel Hidalgo

2.1.0 (2021-03-09)
------------------

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
* Fix get_function and get_const_function semantics for arrays (`#531 <https://github.com/ros2/rosidl/issues/531>`_)
* Fix the declared language for a few packages (`#530 <https://github.com/ros2/rosidl/issues/530>`_)
* Contributors: Ivan Santiago Paunovic, Scott K Logan

1.1.0 (2020-08-17)
------------------

1.0.1 (2020-06-03)
------------------

1.0.0 (2020-05-22)
------------------
* Fix variable suffix in rosidl_export_typesupport_targets (`#483 <https://github.com/ros2/rosidl/issues/483>`_)
* Contributors: Ivan Santiago Paunovic

0.9.2 (2020-05-19)
------------------
* Force extension points to be registered in order (`#485 <https://github.com/ros2/rosidl/issues/485>`_)
* Contributors: Ivan Santiago Paunovic

0.9.1 (2020-05-08)
------------------
* use typesuport targets instead of libraries (`#478 <https://github.com/ros2/rosidl/issues/478>`_)
* Contributors: Dirk Thomas

0.9.0 (2020-04-24)
------------------
* Export missing targets for single typesupport build, avoid exposing build directories in include dirs (`#477 <https://github.com/ros2/rosidl/issues/477>`_)
* Export targets in addition to include directories / libraries (`#471 <https://github.com/ros2/rosidl/issues/471>`_)
* Fix build with single introspection typesupport (`#470 <https://github.com/ros2/rosidl/issues/470>`_)
* Rename rosidl_runtime_c_message_initialization to rosidl_runtime_c__message_initialization (`#464 <https://github.com/ros2/rosidl/issues/464>`_)
* Move non-entry point headers into detail subdirectory (`#461 <https://github.com/ros2/rosidl/issues/461>`_)
* Rename rosidl_generator_c 'namespace' to rosidl_runtime_c (`#458 <https://github.com/ros2/rosidl/issues/458>`_)
* Split rosidl_generator_c and rosidl_generator_cpp in two: rosidl_generator_x and rosidl_runtime_x (`#442 <https://github.com/ros2/rosidl/issues/442>`_)
* Export typesupport library in a separate cmake variable (`#453 <https://github.com/ros2/rosidl/issues/453>`_)
* Style update to match uncrustify with explicit language (`#439 <https://github.com/ros2/rosidl/issues/439>`_)
* Move repeated logic for C include prefix into common function (`#432 <https://github.com/ros2/rosidl/issues/432>`_)
* Contributors: Alejandro Hernández Cordero, Dirk Thomas, Ivan Santiago Paunovic, Jacob Perron

0.8.2 (2020-01-17)
------------------

0.8.1 (2019-10-23)
------------------
* Add init and fini function for creating introspection messages (`#416 <https://github.com/ros2/rosidl/issues/416>`_)
* Contributors: Karsten Knese

0.8.0 (2019-09-24)
------------------
* [rosidl_typesupport_introspection_c] Use message namespaced type name as function prefix (`#387 <https://github.com/ros2/rosidl/issues/387>`_)
* fix cpp generator and introspection ts for long double (`#383 <https://github.com/ros2/rosidl/issues/383>`_)
* Contributors: Dirk Thomas, Jacob Perron
