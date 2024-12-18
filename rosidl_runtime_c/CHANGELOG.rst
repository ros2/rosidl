^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosidl_runtime_c
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

4.6.5 (2024-12-18)
------------------

4.6.4 (2024-09-06)
------------------

4.6.3 (2024-06-27)
------------------

4.6.2 (2024-05-13)
------------------

4.6.1 (2024-04-24)
------------------

4.6.0 (2024-04-16)
------------------

4.5.2 (2024-03-27)
------------------

4.5.1 (2024-02-07)
------------------

4.5.0 (2023-12-26)
------------------
* Switch to target_link_libraries. (`#776 <https://github.com/ros2/rosidl/issues/776>`_)
* Contributors: Chris Lalancette

4.4.2 (2023-11-06)
------------------

4.4.1 (2023-10-04)
------------------

4.4.0 (2023-09-07)
------------------

4.3.1 (2023-08-21)
------------------
* Set the C++ version to 17. (`#761 <https://github.com/ros2/rosidl/issues/761>`_)
* Contributors: Chris Lalancette

4.3.0 (2023-07-11)
------------------

4.2.0 (2023-06-07)
------------------

4.1.1 (2023-05-11)
------------------
* Mark _ in benchmark tests as unused. (`#741 <https://github.com/ros2/rosidl/issues/741>`_)
  This helps clang static analysis.
* Contributors: Chris Lalancette

4.1.0 (2023-04-28)
------------------

4.0.0 (2023-04-11)
------------------
* Dynamic Subscription (BONUS: Allocators): rosidl (`#737 <https://github.com/ros2/rosidl/issues/737>`_)
* Runtime Interface Reflection: rosidl (`#728 <https://github.com/ros2/rosidl/issues/728>`_)
* Type Description Codegen and Typesupport  (rep2011) (`#727 <https://github.com/ros2/rosidl/issues/727>`_)
* Copied type_description_interfaces structs (rep2011) (`#732 <https://github.com/ros2/rosidl/issues/732>`_)
* Expose type hash on typesupports (rep2011) (`#729 <https://github.com/ros2/rosidl/issues/729>`_)
* Type hash in interface codegen (rep2011) (`#722 <https://github.com/ros2/rosidl/issues/722>`_)
* Contributors: Emerson Knapp, methylDragon

3.4.0 (2023-02-13)
------------------
* [service introspection] generate service_event messages (`#700 <https://github.com/ros2/rosidl/issues/700>`_)
* [rolling] Update maintainers - 2022-11-07 (`#717 <https://github.com/ros2/rosidl/issues/717>`_)
* Contributors: Audrow Nash, Brian

3.3.1 (2022-11-02)
------------------

3.3.0 (2022-09-08)
------------------

3.2.1 (2022-06-21)
------------------

3.2.0 (2022-05-04)
------------------

3.1.3 (2022-04-08)
------------------

3.1.2 (2022-04-05)
------------------
* Fix error handling when copying C sequence messages (`#671 <https://github.com/ros2/rosidl/issues/671>`_)
* Contributors: Michel Hidalgo

3.1.1 (2022-03-28)
------------------
* Set the output size unconditionally when copying sequences (`#669 <https://github.com/ros2/rosidl/issues/669>`_)
* Contributors: Nikolai Morin

3.1.0 (2022-03-01)
------------------
* De-duplicate Quality Level from README and QUALITY_DECLARATION (`#661 <https://github.com/ros2/rosidl/issues/661>`_)
* Install headers to include/${PROJECT_NAME} (`#658 <https://github.com/ros2/rosidl/issues/658>`_)
* Implement copy function for C messages (`#650 <https://github.com/ros2/rosidl/issues/650>`_)
* Implement equality operator function for C messages. (`#648 <https://github.com/ros2/rosidl/issues/648>`_)
* Contributors: Jose Luis Rivero, Michel Hidalgo, Shane Loretz

3.0.1 (2022-01-13)
------------------
* Set CXX standard to 17 (`#635 <https://github.com/ros2/rosidl/issues/635>`_)
* Contributors: Øystein Sture

3.0.0 (2021-11-05)
------------------
* Update package maintainers (`#624 <https://github.com/ros2/rosidl/issues/624>`_)
* Contributors: Michel Hidalgo

2.5.0 (2021-08-10)
------------------

2.4.0 (2021-07-12)
------------------

2.3.0 (2021-06-11)
------------------
* Use RCUtils allocators in rosidl_generator_c (`#584 <https://github.com/ros2/rosidl/issues/584>`_)
* Contributors: Pablo Garrido

2.2.1 (2021-04-06)
------------------
* updating quality declaration links (re: `ros2/docs.ros2.org#52 <https://github.com/ros2/docs.ros2.org/issues/52>`_) (`#581 <https://github.com/ros2/rosidl/issues/581>`_)
* Contributors: shonigmann

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
* Update and add package.xml descriptions to README (`#553 <https://github.com/ros2/rosidl/issues/553>`_)
* Fix item number in QD (`#546 <https://github.com/ros2/rosidl/issues/546>`_)
* Update the maintainers of this repository. (`#536 <https://github.com/ros2/rosidl/issues/536>`_)
* Contributors: Chris Lalancette, Louise Poubel, Shane Loretz

2.0.1 (2020-09-28)
------------------
* Add rcutils dependency. (`#534 <https://github.com/ros2/rosidl/issues/534>`_)
* QD: Add links to hosted API docs (`#533 <https://github.com/ros2/rosidl/issues/533>`_)
* Updated Quality Level to 1 (`#532 <https://github.com/ros2/rosidl/issues/532>`_)
* Add benchmarks for rosidl_runtime\_* packages (`#521 <https://github.com/ros2/rosidl/issues/521>`_)
* Contributors: Alejandro Hernández Cordero, Chris Lalancette, Louise Poubel, Scott K Logan

2.0.0 (2020-09-24)
------------------
* Fix the declared language for a few packages (`#530 <https://github.com/ros2/rosidl/issues/530>`_)
* Add fault injection macros and test (`#509 <https://github.com/ros2/rosidl/issues/509>`_)
* Contributors: Scott K Logan, brawner

1.1.0 (2020-08-17)
------------------
* Update rosidl_runtime_c QD to QL 2 (`#500 <https://github.com/ros2/rosidl/issues/500>`_)
* Contributors: Stephen Brawner

1.0.1 (2020-06-03)
------------------
* Add Security Vulnerability Policy pointing to REP-2006 (`#494 <https://github.com/ros2/rosidl/issues/494>`_)
* QD Update Version Stability to stable version (`#495 <https://github.com/ros2/rosidl/issues/495>`_)
* Contributors: Alejandro Hernández Cordero, Chris Lalancette

1.0.0 (2020-05-22)
------------------

0.9.2 (2020-05-19)
------------------
* Update API documentation (`#488 <https://github.com/ros2/rosidl/issues/488>`_)
* Add tests (`#484 <https://github.com/ros2/rosidl/issues/484>`_)
* Add feature documentation (`#482 <https://github.com/ros2/rosidl/issues/482>`_)
* Contributors: brawner

0.9.1 (2020-05-08)
------------------
* Package READMEs and QUALITY_DECLARATIONS for runtime packages (`#480 <https://github.com/ros2/rosidl/issues/480>`_)
* Documentation: action, message, service typesupport and message bounds (`#472 <https://github.com/ros2/rosidl/issues/472>`_)
* Added doxyfile in rosidl_runtime_c and rosidl_runtime_cpp (`#474 <https://github.com/ros2/rosidl/issues/474>`_)
* Contributors: Alejandro Hernández Cordero, brawner

0.9.0 (2020-04-24)
------------------
* Rename message_bounds structure for consistency (`#475 <https://github.com/ros2/rosidl/issues/475>`_)
* Rename rosidl_runtime_c__String__bounds to singular (`#476 <https://github.com/ros2/rosidl/issues/476>`_)
* Document string structs and sequence functions (`#466 <https://github.com/ros2/rosidl/issues/466>`_)
* Export targets in addition to include directories / libraries (`#465 <https://github.com/ros2/rosidl/issues/465>`_)
* Rename rosidl_runtime_c_message_initialization to rosidl_runtime_c__message_initialization (`#464 <https://github.com/ros2/rosidl/issues/464>`_)
* Rename rosidl_generator_c 'namespace' to rosidl_runtime_c (`#458 <https://github.com/ros2/rosidl/issues/458>`_)
* Split rosidl_generator_c and rosidl_generator_cpp in two: rosidl_generator_x and rosidl_runtime_x (`#442 <https://github.com/ros2/rosidl/issues/442>`_)
* Contributors: Alejandro Hernández Cordero, Dirk Thomas, Michael Carroll

0.8.2 (2020-01-17)
------------------

0.8.1 (2019-10-23)
------------------

0.8.0 (2019-09-24)
------------------

0.7.3 (2019-05-29)
------------------

0.7.2 (2019-05-20)
------------------

0.7.1 (2019-05-08)
------------------

0.7.0 (2019-04-12)
------------------

0.6.3 (2019-02-07)
------------------

0.6.2 (2018-12-07)
------------------

0.6.1 (2018-12-06)
------------------

0.6.0 (2018-11-15)
------------------

0.5.1 (2018-06-28)
------------------

0.5.0 (2018-06-23)
------------------

0.4.0 (2017-12-08)
------------------
