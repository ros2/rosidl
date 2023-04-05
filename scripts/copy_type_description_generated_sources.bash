#!/bin/bash
set -euxo pipefail

# Description:
# Copy the structs defined in type_description_interfaces to rosidl_runtime_c and rosidl_runtime_cpp
# so that they can be used in code generation to describe interface types.
# The copy is needed to avoid a circular dependency - type_description_interfaces would be used
# directly for code generation, except as an interface package like any other, it depends on
# rosidl_runtime to have its code generated.
#
# Usage:
# First, `colcon build --packages-up-to type_description_interfaces` from the latest sources.
# Set the following environment variables (script will fail if unset):
# BUILD_DIR - path to the build output of `type_description_interfaces`
# RCL_INTERFACES_SRC_DIR - path to the `rcl_interfaces` repository, where type description interface definitions come from
#
# Example:
# BUILD_DIR=build/type_description_interfaces ROSIDL_SRC_DIR=src/ros2/rosidl src/ros2/rosidl/scripts/copy_type_description_generated_sources.bash
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

C_DETAIL_SUBPATH=rosidl_generator_c/type_description_interfaces/msg/detail
CPP_DETAIL_SUBPATH=rosidl_generator_cpp/type_description_interfaces/msg/detail

C_DETAIL=$BUILD_DIR/$C_DETAIL_SUBPATH
C_INCLUDE_DEST=$ROSIDL_SRC_DIR/rosidl_runtime_c/include/rosidl_runtime_c/type_description
C_SRC_DEST=$ROSIDL_SRC_DIR/rosidl_runtime_c/src/type_description

CPP_DETAIL=$BUILD_DIR/$CPP_DETAIL_SUBPATH
CPP_INCLUDE_DEST=$ROSIDL_SRC_DIR/rosidl_runtime_cpp/include/rosidl_runtime_cpp/type_description

_all_copied=""

# C structs
mkdir -p $C_INCLUDE_DEST
rm -f $C_INCLUDE_DEST/*.h
mkdir -p $C_SRC_DEST
rm -f $C_SRC_DEST/*.c

cp $C_DETAIL/*__struct.h $C_INCLUDE_DEST/
cp $C_DETAIL/*__description.c $C_SRC_DEST/
cp $C_DETAIL/*__functions.h $C_INCLUDE_DEST/
cp $C_DETAIL/*__functions.c $C_SRC_DEST/
_all_copied="$_all_copied $C_DETAIL_SUBPATH/*__struct.h $C_DETAIL_SUBPATH/*__functions.h $C_DETAIL_SUBPATH/*__functions.c $C_DETAIL_SUBPATH/*__description.c"

# add copy notice
sed -i '1s/^/\/\/ DO NOT EDIT MANUALLY - this copied file managed by copy_type_description_generated_sources.bash\n/' $C_INCLUDE_DEST/*.h $C_SRC_DEST/*.c
# remove unnecessary includes (before doing replacements)
sed -i '/type_description_interfaces\/msg\/rosidl_generator_c__visibility_control.h/d' $C_INCLUDE_DEST/*.h $C_SRC_DEST/*.c
sed -i '/#include "rosidl_runtime_c\/type_description\/type_description__struct.h/d' $C_INCLUDE_DEST/*.h $C_SRC_DEST/*.c
sed -i '/#include "rosidl_runtime_c\/type_description\/type_source__struct.h/d' $C_INCLUDE_DEST/*.h $C_SRC_DEST/*.c
# include guards
sed -i -e 's/TYPE_DESCRIPTION_INTERFACES__MSG__DETAIL__/ROSIDL_RUNTIME_C__TYPE_DESCRIPTION__/g' $C_INCLUDE_DEST/*.h $C_SRC_DEST/*.c
# visibility macros
sed -i -e 's/ROSIDL_GENERATOR_C_PUBLIC_type_description_interfaces/ROSIDL_GENERATOR_C_PUBLIC/g' $C_INCLUDE_DEST/*.h $C_SRC_DEST/*.c
# namespace prefixes
sed -i -e 's/type_description_interfaces__msg__/rosidl_runtime_c__type_description__/g' $C_INCLUDE_DEST/*.h $C_SRC_DEST/*.c
# includes
sed -i -e 's/type_description_interfaces\/msg\/detail/rosidl_runtime_c\/type_description/g' $C_INCLUDE_DEST/*.h $C_SRC_DEST/*.c

# C++ structs
mkdir -p $CPP_INCLUDE_DEST
rm -f $CPP_INCLUDE_DEST/*.hpp

cp $CPP_DETAIL/*__struct.hpp $CPP_INCLUDE_DEST
_all_copied="$_all_copied $CPP_DETAIL_SUBPATH/*__struct.hpp"

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
popd

# Create fingerprint file for the safety githook
rm -f $SCRIPT_DIR/type_description.fingerprint
cat << EOF > $SCRIPT_DIR/type_description.fingerprint
# DO NOT EDIT MANUALLY - managed by scripts/copy_type_description_generated_sources.bash
# INTENTIONALLY CHANGING THIS FILE OUTSIDE THE SCRIPT WILL RESULT IN UNDEFINED BEHAVIOR FOR ALL OF ROS 2 CORE
EOF
pushd $BUILD_DIR
sha256sum --tag $_all_copied >> $SCRIPT_DIR/type_description.fingerprint
popd
