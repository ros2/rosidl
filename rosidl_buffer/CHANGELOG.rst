^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosidl_buffer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

5.3.2 (2026-11-03)
------------------
* Use the SPDX identifier Apache-2.0 in license declarations (`#1003 <https://github.com/ros2/rosidl/issues/1003>`_)
* Suppress GCC false positives in buffer resize test (`#980 <https://github.com/ros2/rosidl/issues/980>`_)
* use C++ 20 in default. (`#976 <https://github.com/ros2/rosidl/issues/976>`_)
* Contributors: CY Chen, Michael Carroll, Tomoya Fujita

5.3.1 (2026-06-05)
------------------
* Remove tests in rosidl_buffer for unconventional cases (`#969 <https://github.com/ros2/rosidl/issues/969>`_)
* Avoid warnings from use of default constructed iterators (`#966 <https://github.com/ros2/rosidl/issues/966>`_)
* Contributors: CY Chen, Miguel Company

5.3.0 (2026-05-06)
------------------

5.2.0 (2026-04-24)
------------------
* Add missing std::vector compatible APIs to rosidl::Buffer (`#959 <https://github.com/ros2/rosidl/issues/959>`_)
* Contributors: CY Chen

5.1.5 (2026-04-24)
------------------

5.1.4 (2026-04-09)
------------------
* Bump rosidl_buffer min CMake version (`#956 <https://github.com/ros2/rosidl/issues/956>`_)
* Update rosidl cpp path to emit rosidl::Buffer for uint8[] type (`#942 <https://github.com/ros2/rosidl/issues/942>`_)
* Add cstdint include to c_helpers.cpp (`#953 <https://github.com/ros2/rosidl/issues/953>`_)
* Add rosidl_buffer and rosidl_buffer_backend for native Buffer type support (`#941 <https://github.com/ros2/rosidl/issues/941>`_)
* Contributors: CY Chen, Shane Loretz, mosfet80
