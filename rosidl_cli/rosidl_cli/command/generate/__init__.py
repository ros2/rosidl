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

from rosidl_cli.command import Command

from .extensions import load_type_extensions
from .extensions import load_typesupport_extensions


class GenerateCommand(Command):
    """Generate source code from ROS interface files."""

    name = 'generate'

    def add_arguments(self, parser, cli_name):
        parser.add_argument(
            '-o', '--output-path', metavar='PATH', default=os.getcwd(),
            help=('Path to directory to hold generated source code files. '
                  "Defaults to '.'."))
        parser.add_argument(
            '-t', '--type', metavar='TYPE_SPEC',
            dest='type_specs', action='append', default=[],
            help=('Target type representations for generation, backed by a '
                  'tool extension. Specified by name plus an optional PEP440 '
                  'version specifier set may be provided.'))
        parser.add_argument(
            '-ts', '--type-support', metavar='TYPESUPPORT_SPEC',
            dest='typesupport_specs', action='append', default=[],
            help=('Target type supports for generation, backed by a tool '
                  'extension. Specified by name plus an optional PEP440 '
                  'version specifier set may be provided.'))
        parser.add_argument(
            '-I', '--include-path', metavar='PATH',
            dest='include_paths', action='append', default=[],
            help='Paths to include dependency interface definition files from.')
        parser.add_argument(
            'package_name', help='Name of the package to generate code for')
        parser.add_argument(
            'interface_files', metavar='interface_file', nargs='+',
            help=('Normalized relative path to a ROS interface definition file. '
                  "If prefixed by another path followed by a colon ':', "
                  'path resolution is performed against such path.'))

    def main(self, *, parser, args):
        extensions = []

        unspecific_generation = \
            not args.type_specs and not args.typesupport_specs

        if args.type_specs or unspecific_generation:
            extensions.extend(load_type_extensions(
                specs=args.type_specs,
                strict=not unspecific_generation))

        if args.typesupport_specs or unspecific_generation:
            extensions.extend(load_typesupport_extensions(
                specs=args.typesupport_specs,
                strict=not unspecific_generation))

        if unspecific_generation and not extensions:
            return 'No type nor typesupport extensions were found'

        if len(extensions) > 1:
            for extension in extensions:
                extension.generate(
                    args.package_name, args.interface_files, args.include_paths,
                    output_path=os.path.join(args.output_path, extension.name))
        else:
            extensions[0].generate(
                args.package_name, args.interface_files,
                args.include_paths, args.output_path
            )
