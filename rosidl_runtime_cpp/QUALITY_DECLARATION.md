This document is a declaration of software quality for the `rosidl_runtime_cpp` package, based on the guidelines in [REP-2004](https://www.ros.org/reps/rep-2004.html).

# `rosidl_runtime_cpp` Quality Declaration

The package `rosidl_runtime_cpp` claims to be in the **Quality Level 4** category.

Below are the rationales, notes, and caveats for this claim, organized by each requirement listed in the [Package Requirements for Quality Level 4 in REP-2004](https://www.ros.org/reps/rep-2004.html).

## Version Policy [1]

### Version Scheme [1.i]

`rosidl_runtime_cpp` uses `semver` according to the recommendation for ROS Core packages in the [ROS 2 Developer Guide](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/#versioning).

### Version Stability [1.ii]

`rosidl_runtime_cpp` is not yet at a stable version, i.e. `>= 1.0.0`.

### Public API Declaration [1.iii]

All symbols in the installed headers are considered part of the public API.

All installed headers are in the `include` directory of the package, headers in any other folders are not installed and considered private.

### API Stability Within a Released ROS Distribution [1.iv]/[1.vi]

`rosidl_runtime_cpp` will not break public API within a released ROS distribution, i.e. no major releases once the ROS distribution is released.

### ABI Stability Within a Released ROS Distribution [1.v]/[1.vi]

`rosidl_runtime_cpp` contains C++ code and therefore must be concerned with ABI stability, and will maintain ABI stability within a ROS distribution.

## Change Control Process [2]

`rosidl_runtime_cpp` follows the recommended guidelines for ROS Core packages in the [ROS 2 Developer Guide](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/#package-requirements).

### Change Requests [2.i]

This package requires that all changes occur through a pull request.

### Contributor Origin [2.ii]

This package uses DCO as its confirmation of contributor origin policy.
More information can be found in [CONTRIBUTING](../CONTRIBUTING.md).

### Peer Review Policy [2.iii]

Following the recommended guidelines for ROS Core packages, all pull request have at least 1 peer review.

### Continuous Integration [2.iv]

All pull requests must pass CI on all [tier 1 platforms](https://www.ros.org/reps/rep-2000.html#support-tiers).

Currently nightly results can be seen here:
* [linux-aarch64_release](https://ci.ros2.org/view/nightly/job/nightly_linux-aarch64_release/lastBuild/testReport/rosidl_runtime_cpp/)
* [linux_release](https://ci.ros2.org/view/nightly/job/nightly_linux_release/lastBuild/testReport/rosidl_runtime_cpp/)
* [mac_osx_release](https://ci.ros2.org/view/nightly/job/nightly_osx_release/lastBuild/testReport/rosidl_runtime_cpp/)
* [windows_release](https://ci.ros2.org/view/nightly/job/nightly_win_rel/lastBuild/testReport/rosidl_runtime_cpp/)

### Documentation Policy [2.v]

All pull requests must resolve related documentation changes before merging.

## Documentation [3]

### Feature Documentation [3.i]

`rosidl_runtime_cpp` does not have any feature documentation and it will need to be added for higher quality levels.

### Public API Documentation [3.ii]

`rosidl_runtime_cpp` has some public API documentation.

### License [3.iii]

The license for `rosidl_runtime_cpp` is Apache 2.0, and a summary is in each source file, the type is declared in the [package.xml](package.xml) manifest file, and a full copy of the license is in the [LICENSE](../LICENSE) file.

There is an automated test which runs a linter that ensures each file has a license statement.

### Copyright Statements [3.iv]

The copyright holders each provide a statement of copyright in each source code file in `rosidl_runtime_cpp`.

There is an automated test which runs a linter that ensures each file has at least one copyright statement.

Most recent test results can be found [here](https://ci.ros2.org/job/nightly_linux_release/lastBuild/testReport/rosidl_runtime_cpp/copyright)

## Testing [4]

### Feature Testing [4.i]

There are currently no public features undergoing tests.

### Public API Testing [4.ii]

There are currently no tests for the public API.

### Coverage [4.iv]

`rosidl_runtime_cpp` does not currently track test coverage.

### Performance [4.iv]

`rosidl_runtime_cpp` does not currently have performance tests.

### Linters and Static Analysis [4.v]

`rosidl_runtime_cpp` uses and passes all the standard linters and static analysis tools for a C++ package as described in the [ROS 2 Developer Guide](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/#linters).

Results of the linting tests can be found [here](https://ci.ros2.org/job/nightly_linux_release/lastBuild/testReport/rosidl_runtime_cpp/).

## Dependencies [5]

### Direct Runtime ROS Dependencies [5.i/5.ii]

`rosidl_runtime_cpp` does not have any runtime ROS dependencies.

It has "buildtool" dependencies, which do not affect the resulting quality of the package, because they do not contribute to the public library API.
It also has several test dependencies, which do not affect the resulting quality of the package, because they are only used to build and run the test code.

### Direct Runtime Non-ROS Dependencies [5.iii]

`rosidl_runtime_cpp` does not have any runtime non-ROS dependencies.

## Platform Support [6]

`rosidl_runtime_cpp` supports all of the tier 1 platforms as described in [REP-2000](https://www.ros.org/reps/rep-2000.html#support-tiers), and tests each change against all of them.

Currently nightly results can be seen here:
* [linux-aarch64_release](https://ci.ros2.org/view/nightly/job/nightly_linux-aarch64_release/lastBuild/testReport/rosidl_runtime_cpp/)
* [linux_release](https://ci.ros2.org/view/nightly/job/nightly_linux_release/lastBuild/testReport/rosidl_runtime_cpp/)
* [mac_osx_release](https://ci.ros2.org/view/nightly/job/nightly_osx_release/lastBuild/testReport/rosidl_runtime_cpp/)
* [windows_release](https://ci.ros2.org/view/nightly/job/nightly_win_rel/lastBuild/testReport/rosidl_runtime_cpp/)

# Security [7]

## Vulnerability Disclosure Policy [7.i]

This package does not yet have a Vulnerability Disclosure Policy.
