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
from rosidl_cli.command.hash.api import generate_type_hashes
from rosidl_cli.command.helpers import (
    build_type_description_tuples,
    generate_visibility_control_file,
    generator_arguments_file,
    legacy_generator_arguments,
    split_idl_interface_files,
)
from rosidl_cli.command.translate.api import translate

from rosidl_generator_cpp import generate_cpp


class GenerateCpp(GenerateCommandExtension):

    def generate(
        self,
        package_name,
        interface_files,
        include_paths,
        output_path,
        type_description_files=None
    ):
        package_share_path = \
            pathlib.Path(get_package_share_directory('rosidl_generator_cpp'))
        templates_path = package_share_path / 'resource'

        # Normalize interface definition format to .idl
        idl_interface_files, non_idl_interface_files = split_idl_interface_files(interface_files)
        if non_idl_interface_files:
            idl_interface_files.extend(translate(
                package_name=package_name,
                interface_files=non_idl_interface_files,
                include_paths=include_paths,
                output_format='idl',
                output_path=output_path / 'tmp',
            ))

        if not type_description_files:
            type_description_files = generate_type_hashes(
                package_name=package_name,
                interface_files=idl_interface_files,
                include_paths=include_paths,
                output_path=output_path
            )

        type_description_tuples = build_type_description_tuples(
            idl_interface_files, type_description_files
        )

        generated_files = []
        # Generate visibility control file
        visibility_control_file_template_path = \
            templates_path / 'rosidl_generator_cpp__visibility_control.hpp.in'
        visibility_control_file_path = \
            output_path / 'msg' / 'rosidl_generator_cpp__visibility_control.hpp'

        generate_visibility_control_file(
            package_name=package_name,
            template_path=visibility_control_file_template_path,
            output_path=visibility_control_file_path
        )
        generated_files.append(visibility_control_file_path)

        # Generate code
        with generator_arguments_file(
            **legacy_generator_arguments(
                package_name=package_name,
                interface_files=idl_interface_files,
                include_paths=include_paths,
                templates_path=templates_path,
                output_path=output_path
            ),
            type_description_tuples=type_description_tuples,
        ) as path_to_arguments_file:
            return generate_cpp(path_to_arguments_file)
