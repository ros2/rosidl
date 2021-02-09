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
import glob
import json
import os
import tempfile


def package_name_from_include_path(path):
    return os.path.basename(os.path.dirname(os.path.dirname(path)))


def dependencies_from_include_paths(include_paths):
    return list({
        f'{package_name_from_include_path(path)}:{path}'
        for include_path in include_paths
        for path in glob.iglob(f'{include_path}/**/*.idl', recursive=True)
    })


def idl_tuples_from_interface_files(interface_files):
    idl_tuples = []
    for path in interface_files:
        if ':' not in path:
            prefix = os.getcwd()
            stem = path
        else:
            prefix, _, stem = path.partition(':')
            prefix = os.path.realpath(prefix)
        if os.path.isabs(stem):
            raise ValueError()
        idl_tuples.append(f'{prefix}:{stem}')
    return idl_tuples


@contextlib.contextmanager
def legacy_generator_arguments_file(
    package_name, interface_files,
    include_paths, templates_path,
    output_path
):
    idl_tuples = idl_tuples_from_interface_files(interface_files)
    interface_dependencies = dependencies_from_include_paths(include_paths)
    with tempfile.NamedTemporaryFile(mode='w') as tmp:
        tmp.write(json.dumps({
            'package_name': package_name,
            'output_dir': output_path,
            'template_dir': templates_path,
            'idl_tuples': idl_tuples,
            'ros_interface_dependencies': interface_dependencies,
            'target_dependencies': []
        }))
        tmp.flush()
        yield tmp.name
