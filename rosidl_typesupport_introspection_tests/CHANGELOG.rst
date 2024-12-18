^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosidl_typesupport_introspection_tests
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

4.6.5 (2024-12-18)
------------------
* Suppress false positive warnings from gcc. (`#811 <https://github.com/ros2/rosidl/issues/811>`_) (`#827 <https://github.com/ros2/rosidl/issues/827>`_)
  This is one of the last warnings that is preventing us
  from getting to yellow in release builds.
  (cherry picked from commit ec7745ecedfe9bf9776d041a9d87a5ca3b88d18e)
  Co-authored-by: Chris Lalancette <clalancette@gmail.com>
* Contributors: mergify[bot]

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
* One last uncrustify fix for newer uncrustify. (`#795 <https://github.com/ros2/rosidl/issues/795>`_)
* Contributors: Chris Lalancette

4.5.2 (2024-03-27)
------------------

4.5.1 (2024-02-07)
------------------

4.5.0 (2023-12-26)
------------------

4.4.2 (2023-11-06)
------------------

4.4.1 (2023-10-04)
------------------
* Disable zero-variadic-macro-arguments warning when using clang. (`#768 <https://github.com/ros2/rosidl/issues/768>`_)
* Contributors: Chris Lalancette

4.4.0 (2023-09-07)
------------------
* Fixed C++20 warning implicit capture of this in lambda (`#766 <https://github.com/ros2/rosidl/issues/766>`_)
* Contributors: AiVerisimilitude

4.3.1 (2023-08-21)
------------------

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
* Fix a few more clang analysis problems. (`#731 <https://github.com/ros2/rosidl/issues/731>`_)
  In particular, make sure to mark the fact that we are
  C++17 (as the emplace_back signature changed), and also
  add in a few more (void)_ for benchmark tests.
* Contributors: Chris Lalancette

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

3.1.1 (2022-03-28)
------------------

3.1.0 (2022-03-01)
------------------
* Bump `rosidl_typesupport_introspection_tests` coverage (`#655 <https://github.com/ros2/rosidl/issues/655>`_)
* Add introspection typesupport tests for C/C++ services (`#653 <https://github.com/ros2/rosidl/issues/653>`_)
* Add introspection typesupport tests for C/C++ messages (`#651 <https://github.com/ros2/rosidl/issues/651>`_)
* Contributors: Michel Hidalgo

3.0.1 (2022-01-13)
------------------

3.0.0 (2021-11-05)
------------------

2.5.0 (2021-08-10)
------------------

2.4.0 (2021-07-12)
------------------

2.3.0 (2021-06-11)
------------------

2.2.1 (2021-04-06)
------------------

2.2.0 (2021-03-18)
------------------

2.1.0 (2021-03-09)
------------------

2.0.3 (2021-01-25)
------------------

2.0.2 (2020-12-08)
------------------

2.0.1 (2020-09-28)
------------------

2.0.0 (2020-09-24)
------------------

1.1.0 (2020-08-17)
------------------

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
