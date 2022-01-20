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

import pathlib

from ament_index_python import get_package_share_directory

from rosidl_cli.command.generate.extensions import GenerateCommandExtension
from rosidl_cli.command.helpers import legacy_generator_arguments_file
from rosidl_cli.command.translate.api import translate

from rosidl_typesupport_introspection_cpp import generate_cpp


class GenerateIntrospectionCppTypesupport(GenerateCommandExtension):
    """Make up a CLI for convenient access to the generators of type support."""

    def generate(
        self,
        package_name,
        interface_files,
        include_paths,
        output_path
    ):
        package_share_path = pathlib.Path(
            get_package_share_directory('rosidl_typesupport_introspection_cpp'))

        templates_path = package_share_path / 'resource'

        # Normalize interface definition format to .idl
        idl_interface_files = []
        non_idl_interface_files = []
        for path in interface_files:
            if not path.endswith('.idl'):
                non_idl_interface_files.append(path)
            else:
                idl_interface_files.append(path)
        if non_idl_interface_files:
            idl_interface_files.extend(translate(
                package_name=package_name,
                interface_files=non_idl_interface_files,
                include_paths=include_paths,
                output_format='idl',
                output_path=output_path / 'tmp',
            ))

        # Generate typesupport code
        with legacy_generator_arguments_file(
            package_name=package_name,
            interface_files=idl_interface_files,
            include_paths=include_paths,
            templates_path=templates_path,
            output_path=output_path
        ) as path_to_arguments_file:
            return generate_cpp(path_to_arguments_file)
