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

from rosidl_adapter.msg import to_idl_literal


def test_to_idl_literal_string():
    assert to_idl_literal('string', 'foo') == r'"foo"'
    assert to_idl_literal('string', 'föö') == r'"f\xf6\xf6"'
    assert to_idl_literal('string', 'fロロ') == r'"f\xe3\x83\xad\xe3\x83\xad"'


def test_to_idl_literal_wstring():
    assert to_idl_literal('wstring', 'foo') == r'L"foo"'
    assert to_idl_literal('wstring', 'föö') == r'L"föö"'
    assert to_idl_literal('string', 'fロロ') == r'"f\u30ed\u30ed"'




    # assert to_idl_literal('sequence<string>', ['föö']) == "('föö',)"
    # assert to_idl_literal('sequence<wstring>', ['föö']) == "('föö',)"

    # assert to_idl_literal('sequence<string>', ['föö', 'bar'])) == "('föö', 'bar')"
    # assert to_idl_literal('sequence<wstring>', ['föö', 'bar'])) == "('föö', 'bar')"
