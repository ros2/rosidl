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

import collections
import os
import pathlib

from rosidl_cli.command import Command

from .extensions import load_translate_extensions


class TranslateCommand(Command):
    """Translate interface definition files from one format to another."""

    name = 'translate'

    def add_arguments(self, parser):
        parser.add_argument(
            '-o', '--output-path', metavar='PATH',
            type=pathlib.Path, default=pathlib.Path.cwd(),
            help=('Path to directory to hold translated interface definition'
                  "files. Defaults to '.'."))
        parser.add_argument(
            '--use', '--translator', metavar='TRANSLATOR_SPEC',
            dest='translator_specs', action='append', default=[],
            help=('Translators to be used. If none is given, '
                  'suitable available ones will be used.')
        )
        parser.add_argument(
            '--to', '--output-format', required=True,
            metavar='FORMAT', dest='output_format',
            help='Output format for translate interface definition files.'
        )
        parser.add_argument(
            '--from', '--input-format', default=None,
            metavar='FORMAT', dest='input_format',
            help=('Input format for all source interface definition files. '
                  'If not given, file extensions will be used to deduce '
                  'the format of each interface definition file.')
        )
        parser.add_argument(
            '-I', '--include-path', metavar='PATH', type=pathlib.Path,
            dest='include_paths', action='append', default=[],
            help='Paths to include dependency interface definition files from.'
        )
        parser.add_argument(
            'package_name',
            help='Name of the package all interface files belong to')
        parser.add_argument(
            'interface_files', metavar='interface_file', nargs='+',
            help=('Normalized relative path to an interface definition file. '
                  "If prefixed by another path followed by a colon ':', "
                  'path resolution is performed against such path.')
        )

    def main(self, *, args):
        extensions = load_translate_extensions(
            specs=args.translator_specs,
            strict=any(args.translator_specs)
        )
        if not extensions:
            return 'No translate extensions found'

        if not args.input_format:
            interface_files_per_format = collections.defaultdict(list)
            for interface_file in args.interface_files:
                input_format = os.path.splitext(interface_file)[-1][1:]
                interface_files_per_format[input_format].append(interface_file)
        else:
            interface_files_per_format = {
                args.input_format: args.interface_files}

        for input_format, interface_files in interface_files_per_format.items():
            extension = next((
                extension for extension in extensions
                if extension.input_format == input_format and \
                extension.output_format == args.output_format
            ), None)

            if not extension:
                return (f"Translation from '{input_format}' to "
                        f"'{args.output_format}' is not supported")

            extension.translate(
                args.package_name, interface_files,
                args.include_paths, args.output_path)
