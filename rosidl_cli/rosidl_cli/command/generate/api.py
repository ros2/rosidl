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

import inspect
import os
import pathlib

from .extensions import load_type_extensions, load_typesupport_extensions


def generate(
    *,
    package_name,
    interface_files,
    include_paths=None,
    output_path=None,
    types=None,
    typesupports=None,
    type_description_files=None
):
    """
    Generate source code from interface definition files.

    To do so, this function leverages type representation and type
    support generation support as provided by third-party package
    extensions.

    Each path to an interface definition file is a relative path optionally
    prefixed by another path followed by a colon ':', against which the first
    relative path is to be resolved.

    The directory structure that these relative paths exhibit will be replicated
    on output (as opposed to the prefix path, which will be ignored).

    If no type representation nor type support is specified, all available ones
    will be generated.

    If more than one type representation or type support is generated, the
    name of each will be appended to the given `output_path` to preclude
    name clashes upon writing source code files.

    :param package_name: name of the package to generate source code for
    :param interface_files: list of paths to interface definition files
    :param include_paths: optional list of paths to include dependency
        interface definition files from
    :param output_path: optional path to directory to hold generated
        source code files, defaults to the current working directory
    :param types: optional list of type representations to generate
    :param typesupports: optional list of type supports to generate
    :param type_description_files: Optional list of paths to type description files
    :returns: list of lists of paths to generated source code files,
        one group per type or type support extension invoked
    """
    extensions = []

    unspecific_generation = not types and not typesupports

    if types or unspecific_generation:
        extensions.extend(load_type_extensions(
            specs=types,
            strict=not unspecific_generation))

    if typesupports or unspecific_generation:
        extensions.extend(load_typesupport_extensions(
            specs=typesupports,
            strict=not unspecific_generation))

    if unspecific_generation and not extensions:
        raise RuntimeError('No type nor typesupport extensions were found')

    if include_paths is None:
        include_paths = []

    if output_path is None:
        output_path = pathlib.Path.cwd()
    else:
        os.makedirs(output_path, exist_ok=True)

    def extra_kwargs(func, **kwargs):
        matched_kwargs = {}
        signature = inspect.signature(func)
        for name, value in kwargs.items():
            if name in signature.parameters:
                if signature.parameters[name].kind not in [
                    inspect.Parameter.POSITIONAL_ONLY,
                    inspect.Parameter.VAR_POSITIONAL,
                    inspect.Parameter.VAR_KEYWORD
                ]:
                    matched_kwargs[name] = value
        return matched_kwargs

    generated_files = []
    if len(extensions) == 1:
        extension = extensions[0]
        generated_files.append(
            extension.generate(
                package_name, interface_files, include_paths,
                output_path=output_path,
                **extra_kwargs(extension.generate, type_description_files=type_description_files)
            )
        )
    else:
        for extension in extensions:
            generated_files.append(
                extension.generate(
                    package_name, interface_files, include_paths,
                    output_path=output_path / extension.name,
                    **extra_kwargs(
                        extension.generate,
                        type_description_files=type_description_files
                    )
                )
            )
    return generated_files
