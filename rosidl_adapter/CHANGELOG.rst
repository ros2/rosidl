^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosidl_adapter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.1 (2021-04-14)
------------------
* Hex constants and tab as whitespace support (`#568 <https://github.com/ros2/rosidl/issues/568>`_)
  * Treat \t as whitespace (`#557 <https://github.com/ros2/rosidl/issues/557>`_)
  * Support hex constants in msg files (`#559 <https://github.com/ros2/rosidl/issues/559>`_)
* Contributors: Dereck Wonnacott

1.2.0 (2020-12-08)
------------------

1.1.0 (2020-08-17)
------------------
* Refactor regex for valid package/field names (`#508 <https://github.com/ros2/rosidl/issues/508>`_)
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

0.8.1 (2019-10-23)
------------------

0.8.0 (2019-09-24)
------------------
* Improve parser error messages (`#415 <https://github.com/ros2/rosidl/issues/415>`_)
* support adjacent string literals, use them for multi-line comments (`#410 <https://github.com/ros2/rosidl/issues/410>`_)
* avoid zero length comment when the comment only contains a unit (`#411 <https://github.com/ros2/rosidl/issues/411>`_)
* use latin-1 encoding when reading/writing .idl files, prepend BOM to generated C/C++ files when necessary (`#391 <https://github.com/ros2/rosidl/issues/391>`_)
* fix error msg asserts due to change in pytest (`#393 <https://github.com/ros2/rosidl/issues/393>`_)
* open interface files with utf-8 encoding (`#390 <https://github.com/ros2/rosidl/issues/390>`_)
* Contributors: Dirk Thomas, Jacob Perron, William Woodall
