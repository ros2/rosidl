#!/bin/bash
set -euxo pipefail

# Description:
# Copy the structs defined in type_description_interfaces to rosidl_runtime_c and rosidl_runtime_cpp
# so that they can be used in code generation to describe interface types
#
# Usage:
# First, `colcon build --packages-up-to type_description_interfaces` from the latest sources.
# Set the following environment variables (script will fail if unset):
# BUILD_DIR - path to the build output of `type_description_interfaces`
# ROSIDL_SRC_DIR - path to the `rosidl` repository, where type description files will be placed
#
# Example:
# BUILD_DIR=build/type_description_interfaces ROSIDL_SRC_DIR=src/ros2/rosidl src/ros2/rosidl/scripts/copy_type_description_generated_sources.bash

C_DETAIL=$BUILD_DIR/rosidl_generator_c/type_description_interfaces/msg/detail/
C_INCLUDE_DEST=$ROSIDL_SRC_DIR/rosidl_runtime_c/include/rosidl_runtime_c/type_description/
C_SRC_DEST=$ROSIDL_SRC_DIR/rosidl_runtime_c/src/type_description/

CPP_DETAIL=$BUILD_DIR/rosidl_generator_cpp/type_description_interfaces/msg/detail/
CPP_INCLUDE_DEST=$ROSIDL_SRC_DIR/rosidl_runtime_cpp/include/rosidl_runtime_cpp/type_description/

# C structs
mkdir -p $C_INCLUDE_DEST
rm -f $C_INCLUDE_DEST/*.h
mkdir -p $C_SRC_DEST
rm -f $C_SRC_DEST/*.c

cp $C_DETAIL/*__struct.h $C_INCLUDE_DEST/
cp $C_DETAIL/*__functions.h $C_INCLUDE_DEST/
cp $C_DETAIL/*__functions.c $C_SRC_DEST/

# add copy notice
sed -i '1s/^/\/\/ DO NOT EDIT MANUALLY - this copied file managed by copy_type_description_generated_sources.bash\n/' $C_INCLUDE_DEST/*.h $C_SRC_DEST/*.c
# remove unecessary includes (before doing replacements)
sed -i '/type_description_interfaces\/msg\/rosidl_generator_c__visibility_control.h/d' $C_INCLUDE_DEST/*.h $C_SRC_DEST/*.c
sed -i '/#include "rosidl_runtime_c\/type_description\/type_description__struct.h/d' $C_INCLUDE_DEST/*.h $C_SRC_DEST/*.c
# include guards
sed -i -e 's/TYPE_DESCRIPTION_INTERFACES__MSG__DETAIL__/ROSIDL_RUNTIME_C__TYPE_DESCRIPTION__/g' $C_INCLUDE_DEST/*.h $C_SRC_DEST/*.c
# visibility macros
sed -i -e 's/ROSIDL_GENERATOR_C_PUBLIC_type_description_interfaces/ROSIDL_GENERATOR_C_PUBLIC/g' $C_INCLUDE_DEST/*.h $C_SRC_DEST/*.c
# namespace prefixes
sed -i -e 's/type_description_interfaces__msg__/rosidl_runtime_c__type_description__/g' $C_INCLUDE_DEST/*.h $C_SRC_DEST/*.c
# includes
sed -i -e 's/type_description_interfaces\/msg\/detail/rosidl_runtime_c\/type_description/g' $C_INCLUDE_DEST/*.h $C_SRC_DEST/*.c
# sed -i -e 's/extern const rosidl_runtime_c__type_description__TypeDescription/\/\/ extern const rosidl_runtime_c__type_description__TypeDescription/' $C_INCLUDE_DEST/*.h $C_SRC_DEST/*.c

# C++ structs
mkdir -p $CPP_INCLUDE_DEST
rm -f $CPP_INCLUDE_DEST/*.hpp

cp $CPP_DETAIL/*__struct.hpp $CPP_INCLUDE_DEST

pushd $CPP_INCLUDE_DEST
# add copy notice
sed -i '1s/^/\/\/ DO NOT EDIT MANUALLY - this copied file managed by copy_type_description_generated_sources.bash\n/' *.hpp
# include guards
sed -i -e 's/TYPE_DESCRIPTION_INTERFACES__MSG__DETAIL__/ROSIDL_RUNTIME_CPP__TYPE_DESCRIPTION__/g' *.hpp
# pkg namespace
sed -i -e 's/namespace type_description_interfaces/namespace rosidl_runtime_cpp/g' *.hpp
# msg namespace
sed -i -e 's/namespace msg/namespace type_description/g' *.hpp
# includes
sed -i -e 's/type_description_interfaces\/msg\/detail/rosidl_runtime_cpp\/type_description/g' *.hpp
# type references
sed -i -e 's/type_description_interfaces::msg::/rosidl_runtime_cpp::type_description::/g' *.hpp
# macros
sed -i -e 's/type_description_interfaces__msg__/rosidl_runtime_cpp__type_description__/g' *.hpp
# sed -i -e 's/static const rosidl_runtime_cpp::type_description::TypeDescription/\/\/ static const rosidl_runtime_cpp::type_description::TypeDescription/' *.hpp
popd
