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

from .extensions import load_hash_extensions


def generate_type_hashes(
    *,
    package_name,
    interface_files,
    include_paths=None,
    output_path=None,
):
    """
    Generate type hashes from interface definition files.

    To do so, this function leverages type description hash generation support
    as provided by third-party package extensions.

    Each path to an interface definition file is a relative path optionally
    prefixed by another path followed by a colon ':', against which the first
    relative path is to be resolved.

    The directory structure that these relative paths exhibit will be replicated
    on output (as opposed to the prefix path, which will be ignored).

    :param package_name: name of the package to generate hashes for
    :param interface_files: list of paths to interface definition files
    :param include_paths: optional list of paths to include dependency
        interface definition files from
    :param output_path: optional path to directory to hold generated
        source code files, defaults to the current working directory
    :returns: list of lists of paths to generated hashed json files,
        one group per type or type support extension invoked
    """
    extensions = []
    extensions.extend(load_hash_extensions())

    if include_paths is None:
        include_paths = []
    if output_path is None:
        output_path = pathlib.Path.cwd()
    else:
        pathlib.Path.mkdir(output_path, parents=True, exist_ok=True)

    generated_hashes = []
    for extension in extensions:
        generated_hashes.extend(extension.generate_type_hashes(
            package_name,
            interface_files,
            include_paths,
            output_path=output_path,
        ))

    return generated_hashes
