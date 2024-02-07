# Copyright 2023 Open Source Robotics Foundation, Inc.
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

import json
import os
import pathlib


def read_json_file(input_file):
    with open(input_file, mode='r', encoding='utf-8') as h:
        return json.load(h)


def get_generator_module(module_name, module_path):
    if not os.path.exists(module_path):
        raise

    from importlib.machinery import SourceFileLoader

    loader = SourceFileLoader(module_name, module_path)
    generator_files_module = loader.load_module()
    return generator_files_module


def find_generator_module(module_name, generator_files):
    for generator_file in generator_files:
        generator_file_module_path = os.path.normpath(generator_file)
        generator_file_module = get_generator_module(module_name, generator_file_module_path)

        if hasattr(generator_file_module, 'get_template_mapping'):
            # Found module with expected attributes
            return generator_file_module

    raise ModuleNotFoundError("Could not find generator module for '" + module_name + "' in: " + generator_files)


class GeneratorConfig:

    def __init__(self, arguments_file):
        print(f"\n\nArg file = {arguments_file}\n\n")
        self.arguments_file = arguments_file
        self.arguments = read_json_file(self.arguments_file)

        # Create a unique module name from the arguments file
        module_name = self.arguments_file.rsplit('/', 1)[-1].rsplit('__', 1)[0]

        generator_files_module = find_generator_module(module_name, self.arguments['generator_files'])

        # Get template mapping (required)
        if not hasattr(generator_files_module, 'get_template_mapping'):
            raise NotImplementedError("Missing function 'get_template_mapping()' in generator module for " + module_name)
        self.mapping = generator_files_module.get_template_mapping()
        # Check that templates exist
        template_basepath = pathlib.Path(self.arguments['template_dir'])
        for template_filename in self.mapping.keys():
            assert (template_basepath / template_filename).exists(), \
                'Could not find template: ' + template_filename

        # Additional context (optional)
        self.additional_context = None
        if 'additional_context_file' in self.arguments:
            print(f"Got additional_context_file: {self.arguments['additional_context_file']}")
            self.additional_context = read_json_file(self.arguments['additional_context_file'])

        # Keep case (optional)
        self.keep_case = False
        if hasattr(generator_files_module, 'should_keep_case'):
            self.keep_case = generator_files_module.should_keep_case()

        # Post-process callback (optional)
        self.post_process_callback = None
        if hasattr(generator_files_module, 'post_process_callback'):
            self.post_process_callback = generator_files_module.post_process_callback
