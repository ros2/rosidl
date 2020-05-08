^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosidl_adapter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
