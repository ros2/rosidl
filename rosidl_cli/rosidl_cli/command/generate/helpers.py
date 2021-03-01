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

import contextlib
import json
import os
import pathlib
import tempfile


def package_name_from_interface_file_path(path):
    """
    Derive ROS package name from a ROS interface definition file path.

    This function assumes ROS interface definition files follow the typical
    ``rosidl`` install space layout i.e. 'package_name/subfolder/interface.idl'.
    """
    return pathlib.Path(path).resolve().parents[1].name


def dependencies_from_include_paths(include_paths):
    """
    Collect dependencies' ROS interface definition files from include paths.

    Interface definition file paths from dependencies are absolute paths
    prefixed by the name of package they belong to followed by a colon ':'.
    """
    return list({
        f'{package_name_from_interface_file_path(path)}:{path}'
        for include_path in map(pathlib.Path, include_paths)
        for path in include_path.resolve().glob('**/*.idl')
    })


def idl_tuples_from_interface_files(interface_files):
    """
    Express ROS interface definition file paths as IDL tuples.

    An IDL tuple is a relative path prefixed by by an absolute path against
    which to resolve it followed by a colon ':'. This function then applies
    the following logic:
    - If a given path follows this pattern, it is passed through.
    - If a given path is prefixed by a relative path, it is resolved
      relative to the current working directory.
    - If a given path has no prefixes, the current working directory is
      used as prefix.
    """
    idl_tuples = []
    for path in interface_files:
        path_as_string = str(path)
        if ':' not in path_as_string:
            prefix = pathlib.Path.cwd()
            stem = path
        else:
            prefix, _, stem = path_as_string.rpartition(':')
            prefix = pathlib.Path(prefix).resolve()
        stem = pathlib.Path(stem)
        if stem.is_absolute():
            raise ValueError('Interface definition file path '
                             f'{stem} cannot be absolute')
        idl_tuples.append(f'{prefix}:{stem.as_posix()}')
    return idl_tuples


@contextlib.contextmanager
def legacy_generator_arguments_file(
    *,
    package_name,
    interface_files,
    include_paths,
    templates_path,
    output_path
):
    """
    Generate a temporary rosidl generator arguments file.

    :param package_name: Name of the ROS package for which to generate code
    :param interface_files: Relative paths to ROS interface definition files,
      optionally prefixed by another absolute or relative path followed by
      a colon ':'. The former relative paths will be used as a prototype to
      namespace generated code (if applicable).
    :param include_paths: Paths where ROS package dependencies' interface
      definition files may be found
    :param templates_path: Path to the templates directory for the
      generator script this arguments are for
    :param output_path: Path to the output directory for generated code
    """
    idl_tuples = idl_tuples_from_interface_files(interface_files)
    interface_dependencies = dependencies_from_include_paths(include_paths)
    output_path = pathlib.Path(output_path).resolve()
    templates_path = pathlib.Path(templates_path).resolve()
    with tempfile.NamedTemporaryFile(mode='w') as tmp:
        tmp.write(json.dumps({
            'package_name': package_name,
            'output_dir': str(output_path),
            'template_dir': str(templates_path),
            'idl_tuples': idl_tuples,
            'ros_interface_dependencies': interface_dependencies,
            # TODO(hidmic): re-enable output file caching
            'target_dependencies': []
        }))
        tmp.flush()
        yield tmp.name


def generate_visibility_control_file(
    *,
    package_name,
    template_path,
    output_path
):
    """
    Generate a visibility control file from a template.

    :param package_name: Name of the ROS package for which
      to generate the file.
    :param template_path: Path to template visibility control file.
      May contain @PROJECT_NAME@ and @PROJECT_NAME_UPPER@ placeholders,
      to be substituted by the package name, accordingly.
    :param output_path: Path to visibility control file after interpolation.
    """
    os.makedirs(os.path.dirname(output_path), exist_ok=True)

    with open(template_path, 'r') as fd:
        content = fd.read()

    content = content.replace('@PROJECT_NAME@', package_name)
    content = content.replace('@PROJECT_NAME_UPPER@', package_name.upper())

    with open(output_path, 'w') as fd:
        fd.write(content)
