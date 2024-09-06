^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosidl_parser
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Adding interfaces to support `@key` annotation (`#796 <https://github.com/ros2/rosidl/issues/796>`_)
  Co-authored-by: Mario Dominguez <mariodominguez@eprosima.com>
* Contributors: Miguel Company

4.5.2 (2024-03-27)
------------------
* Small fix for newer flake8 compatibility. (`#792 <https://github.com/ros2/rosidl/issues/792>`_)
* Contributors: Chris Lalancette

4.5.1 (2024-02-07)
------------------
* Remove unnecessary parentheses. (`#783 <https://github.com/ros2/rosidl/issues/783>`_)
* Contributors: Chris Lalancette

4.5.0 (2023-12-26)
------------------

4.4.2 (2023-11-06)
------------------

4.4.1 (2023-10-04)
------------------

4.4.0 (2023-09-07)
------------------

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

3.4.0 (2023-02-13)
------------------
* [service introspection] generate service_event messages (`#700 <https://github.com/ros2/rosidl/issues/700>`_)
* [rolling] Update maintainers - 2022-11-07 (`#717 <https://github.com/ros2/rosidl/issues/717>`_)
* Contributors: Audrow Nash, Brian

3.3.1 (2022-11-02)
------------------

3.3.0 (2022-09-08)
------------------
* Always include whitespace in string literals (`#688 <https://github.com/ros2/rosidl/issues/688>`_)
* Contributors: Shane Loretz

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
* Set maybe_placeholders to False for lark 1.+ compatibility (`#664 <https://github.com/ros2/rosidl/issues/664>`_)
* Generate documentation in generated C header files based on ROS interfaces comments (`#593 <https://github.com/ros2/rosidl/issues/593>`_)
* Contributors: Ivan Santiago Paunovic, Shane Loretz

3.0.1 (2022-01-13)
------------------
* Pass comments in ros interface constants to the .idl generated files (`#630 <https://github.com/ros2/rosidl/issues/630>`_)
* Contributors: Ivan Santiago Paunovic

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
* Fix escaping in string literals (`#595 <https://github.com/ros2/rosidl/issues/595>`_)
* Contributors: Ivan Santiago Paunovic

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
* Update and add package.xml descriptions to README (`#553 <https://github.com/ros2/rosidl/issues/553>`_)
* Finish support for fixed-point literals.
* Fix parsing of small floats.
* Update the maintainers of this repository. (`#536 <https://github.com/ros2/rosidl/issues/536>`_)
* Contributors: Chris Lalancette, Shane Loretz

2.0.1 (2020-09-28)
------------------

2.0.0 (2020-09-24)
------------------

1.1.0 (2020-08-17)
------------------
* Allow zero length string constants (`#507 <https://github.com/ros2/rosidl/issues/507>`_)
* Add pytest.ini so tests succeed locally (`#502 <https://github.com/ros2/rosidl/issues/502>`_)
* Contributors: Chris Lalancette, Dirk Thomas

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
* Use f-string (`#436 <https://github.com/ros2/rosidl/issues/436>`_)
* Contributors: Dirk Thomas

0.8.2 (2020-01-17)
------------------
* Allow 'get_const_expr_value' to parse either literals or scoped_names… (`#430 <https://github.com/ros2/rosidl/issues/430>`_)
* Use imperative mood in constructor docstring. (`#425 <https://github.com/ros2/rosidl/issues/425>`_)
* Contributors: Steven! Ragnarök, kylemarcey

0.8.1 (2019-10-23)
------------------

0.8.0 (2019-09-24)
------------------
* support adjacent string literals, use them for multi-line comments (`#410 <https://github.com/ros2/rosidl/issues/410>`_)
* fix parsing empty string literal (`#409 <https://github.com/ros2/rosidl/issues/409>`_)
* add constant for member name in empty structs (`#389 <https://github.com/ros2/rosidl/issues/389>`_)
* Contributors: Dirk Thomas
