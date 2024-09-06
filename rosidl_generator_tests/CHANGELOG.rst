^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosidl_generator_tests
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

4.6.4 (2024-09-06)
------------------
* Silence one more gcc false-positive. (`#814 <https://github.com/ros2/rosidl/issues/814>`_) (`#817 <https://github.com/ros2/rosidl/issues/817>`_)
  gcc 13 has false positives around array-bounds and
  stringop-overflow, so suppress them here while generating
  test cases.
  (cherry picked from commit 86fa39823b6ec7d2e24940647c214a4082beab79)
  Co-authored-by: Chris Lalancette <clalancette@gmail.com>
* Contributors: mergify[bot]

4.6.3 (2024-06-27)
------------------

4.6.2 (2024-05-13)
------------------

4.6.1 (2024-04-24)
------------------
* Fixed warnings - strict-prototypes (`#800 <https://github.com/ros2/rosidl/issues/800>`_) (`#802 <https://github.com/ros2/rosidl/issues/802>`_)
* Contributors: mergify[bot]

4.6.0 (2024-04-16)
------------------
* Increased the cpplint timeout to 300 seconds (`#797 <https://github.com/ros2/rosidl/issues/797>`_)
* Contributors: Alejandro Hernández Cordero

4.5.2 (2024-03-27)
------------------
* Fixes for modern uncrustify. (`#793 <https://github.com/ros2/rosidl/issues/793>`_)
* Contributors: Chris Lalancette

4.5.1 (2024-02-07)
------------------

4.5.0 (2023-12-26)
------------------

4.4.2 (2023-11-06)
------------------
* Fix constant generation for C++ floats (`#772 <https://github.com/ros2/rosidl/issues/772>`_)
* Contributors: Chris Lalancette

4.4.1 (2023-10-04)
------------------

4.4.0 (2023-09-07)
------------------

4.3.1 (2023-08-21)
------------------
* Fix same named types overriding typesources (`#759 <https://github.com/ros2/rosidl/issues/759>`_)
* Contributors: Emerson Knapp

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
* Type hash in interface codegen (rep2011) (`#722 <https://github.com/ros2/rosidl/issues/722>`_)
* Contributors: Emerson Knapp

3.4.0 (2023-02-13)
------------------
* [service introspection] generate service_event messages (`#700 <https://github.com/ros2/rosidl/issues/700>`_)
  * add service event message
* [rolling] Update maintainers - 2022-11-07 (`#717 <https://github.com/ros2/rosidl/issues/717>`_)
* Contributors: Audrow Nash, Brian

3.3.1 (2022-11-02)
------------------

3.3.0 (2022-09-08)
------------------
* Move rosidl_generator_c/cpp tests to a separate package (`#701 <https://github.com/ros2/rosidl/issues/701>`_)
* Contributors: Jacob Perron
