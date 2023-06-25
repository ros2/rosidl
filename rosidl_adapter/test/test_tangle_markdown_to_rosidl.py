# Copyright 2014 Open Source Robotics Foundation, Inc.
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

from rosidl_adapter.parser import tangle_markdown_to_rosidl


DATA_PATH = pathlib.Path(__file__).parent / 'data'


def test_tangle_msg_markdown_to_rosidl():
    test_input_file_path = f'{DATA_PATH}/msg/Test.msg.md'
    expected_output_file_path = f'{DATA_PATH}/msg/Test.msg'
    with open(test_input_file_path, 'r', encoding='utf-8') as h:
        test_input_content = h.read()
    with open(expected_output_file_path, 'r', encoding='utf-8') as h:
        expected_output_content = h.read().strip()
    assert tangle_markdown_to_rosidl(
        test_input_content, 'msg') == expected_output_content


def test_tangle_action_markdown_to_rosidl():
    test_input_file_path = f'{DATA_PATH}/action/Test.action.md'
    expected_output_file_path = f'{DATA_PATH}/action/Test.action'
    with open(test_input_file_path, 'r', encoding='utf-8') as h:
        test_input_content = h.read()
    with open(expected_output_file_path, 'r', encoding='utf-8') as h:
        expected_output_content = h.read().strip()
    print(tangle_markdown_to_rosidl(
        test_input_content, 'action'))
    assert tangle_markdown_to_rosidl(
        test_input_content, 'action') == expected_output_content
