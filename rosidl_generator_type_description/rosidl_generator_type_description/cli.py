# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import pathlib

from ament_index_python import get_package_share_directory
from rosidl_cli.command.hash.extensions import HashCommandExtension
from rosidl_cli.command.helpers import (
    generator_arguments_file,
    legacy_generator_arguments,
    package_name_from_interface_file_path,
    split_idl_interface_files,
)
from rosidl_cli.command.translate.api import translate

from rosidl_generator_type_description import generate_type_hash


def package_paths_from_include_paths(include_paths):
    """
    Collect package paths, typically share paths, from include paths.

    Package paths are absolute paths prefixed by the name of package followed by a colon ':'.
    """
    return list(
        {
            f'{package_name_from_interface_file_path(path)}:{path.parents[1]}'
            for include_path in map(os.path.abspath, include_paths)
            for path in pathlib.Path(include_path).glob('**/*.idl')
        }
    )


class HashTypeDescription(HashCommandExtension):
    def generate_type_hashes(
        self,
        package_name,
        interface_files,
        include_paths,
        output_path,
    ):
        package_share_path = \
            pathlib.Path(get_package_share_directory('rosidl_generator_type_description'))
        templates_path = package_share_path / 'resource'

        idl_interface_files, non_idl_interface_files = split_idl_interface_files(interface_files)
        if non_idl_interface_files:
            idl_interface_files.extend(translate(
                package_name=package_name,
                interface_files=non_idl_interface_files,
                include_paths=include_paths,
                output_format='idl',
                output_path=output_path / 'tmp',
            ))

        include_path_tuples = package_paths_from_include_paths(include_paths)

        # Generate code
        with generator_arguments_file(
            **legacy_generator_arguments(
                package_name=package_name,
                interface_files=idl_interface_files,
                include_paths=include_paths,
                templates_path=templates_path,
                output_path=output_path,
            ),
            include_paths=include_path_tuples
        ) as path_to_arguments_file:
            return generate_type_hash(path_to_arguments_file)
