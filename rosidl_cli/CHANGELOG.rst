^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosidl_cli
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

4.6.7 (2025-12-09)
------------------
* fix setuptools deprecations (`#877 <https://github.com/ros2/rosidl/issues/877>`_) (`#896 <https://github.com/ros2/rosidl/issues/896>`_)
* Contributors: mergify[bot]

4.6.6 (2025-08-05)
------------------
* rosidl_cli: Add type description support (backport `#857 <https://github.com/ros2/rosidl/issues/857>`_) (`#867 <https://github.com/ros2/rosidl/issues/867>`_)
  * rosidl_cli: Add type description support (`#857 <https://github.com/ros2/rosidl/issues/857>`_)
  * Add type description support to rosidl_cli
  * Fix typing annotation
  * Please flake8, mypy
  * Please flake8 take 2
  ---------
  Co-authored-by: Michel Hidalgo <michel@ekumenlabs.com>
  Co-authored-by: mergify[bot] <37929162+mergify[bot]@users.noreply.github.com>
  Co-authored-by: Christophe Bedard <bedard.christophe@gmail.com>
  (cherry picked from commit c9a3084d8570ee7d1f3a35b81d7cf9259aee6dbc)
  # Conflicts:
  #	rosidl_cli/rosidl_cli/cli.py
  #	rosidl_cli/rosidl_cli/command/generate/api.py
  #	rosidl_cli/rosidl_cli/command/generate/extensions.py
  #	rosidl_cli/rosidl_cli/command/helpers.py
  * Solve conflicts (`#868 <https://github.com/ros2/rosidl/issues/868>`_)
  * Please flake8 (`#870 <https://github.com/ros2/rosidl/issues/870>`_)
  ---------
  Co-authored-by: Francisco Rossi <50388097+frneer@users.noreply.github.com>
  Co-authored-by: Michel Hidalgo <michel@ekumenlabs.com>
* Contributors: mergify[bot]

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
* Fix warnings (`#726 <https://github.com/ros2/rosidl/issues/726>`_)
* Contributors: Yadu

3.4.0 (2023-02-13)
------------------
* [rolling] Update maintainers - 2022-11-07 (`#717 <https://github.com/ros2/rosidl/issues/717>`_)
* Contributors: Audrow Nash

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
* Fix importlib_metdata warning with Python 3.10. (`#674 <https://github.com/ros2/rosidl/issues/674>`_)
* Contributors: Chris Lalancette

3.1.2 (2022-04-05)
------------------

3.1.1 (2022-03-28)
------------------

3.1.0 (2022-03-01)
------------------

3.0.1 (2022-01-13)
------------------
* Update maintainers to Michel Hidalgo and Shane Loretz (`#633 <https://github.com/ros2/rosidl/issues/633>`_)
* Contributors: Audrow Nash

3.0.0 (2021-11-05)
------------------
* Update package maintainers (`#624 <https://github.com/ros2/rosidl/issues/624>`_)
* Contributors: Michel Hidalgo

2.5.0 (2021-08-10)
------------------

2.4.0 (2021-07-12)
------------------
* Support passing keyword arguments to rosidl CLI extensions (`#597 <https://github.com/ros2/rosidl/issues/597>`_)
* Add missing f for format string (`#600 <https://github.com/ros2/rosidl/issues/600>`_)
* Contributors: Michel Hidalgo, Shane Loretz

2.3.0 (2021-06-11)
------------------

2.2.1 (2021-04-06)
------------------

2.2.0 (2021-03-18)
------------------

2.1.0 (2021-03-09)
------------------
* Align rosidl_cli package version with the rest of the repo. (`#579 <https://github.com/ros2/rosidl/issues/579>`_)
* Expose an API for each rosidl CLI command.  (`#577 <https://github.com/ros2/rosidl/issues/577>`_)
* Add rosidl translate CLI. (`#575 <https://github.com/ros2/rosidl/issues/575>`_)
* Add rosidl generate CLI. (`#567 <https://github.com/ros2/rosidl/issues/567>`_)
* Contributors: Michel Hidalgo, Shane Loretz
