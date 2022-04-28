^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosidl_adapter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.2.2 (2022-04-28)
------------------
* Fix escaping in string literals (`#595 <https://github.com/ros2/rosidl/issues/595>`_) (`#617 <https://github.com/ros2/rosidl/issues/617>`_)
* Contributors: Jacob Perron

2.2.1 (2021-04-06)
------------------

2.2.0 (2021-03-18)
------------------
* Expose .msg/.srv/.action to .idl conversion via rosidl translate CLI (`#576 <https://github.com/ros2/rosidl/issues/576>`_)
* Contributors: Michel Hidalgo

2.1.0 (2021-03-09)
------------------
* Support hex constants in msg files (`#559 <https://github.com/ros2/rosidl/issues/559>`_)
* Contributors: Dereck Wonnacott

2.0.3 (2021-01-25)
------------------
* Treat \t as whitespace (`#557 <https://github.com/ros2/rosidl/issues/557>`_)
* Contributors: Dereck Wonnacott

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
