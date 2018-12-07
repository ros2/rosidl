# Copyright 2018 Open Source Robotics Foundation, Inc.
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

from rosidl_adapter.parser import parse_action_string
from rosidl_adapter.resource import expand_template


def convert_action_to_idl(package_dir, package_name, input_file, output_dir):
    assert package_dir.is_absolute()
    assert not input_file.is_absolute()
    assert input_file.suffix == '.action'

    abs_input_file = package_dir / input_file
    print('Reading input file: {abs_input_file}'.format_map(locals()))
    abs_input_file = package_dir / input_file
    content = abs_input_file.read_text(encoding='utf-8')
    action = parse_action_string(package_name, input_file.stem, content)

    # HACK as long as the return action specification contains implicitly added
    # fields they have to be skipped when generating .idl files
    assert len(action.goal_service.request.fields) >= 1
    assert action.goal_service.request.fields[0].name == 'action_goal_id'
    action.goal_service.request.fields.pop(0)
    assert len(action.goal_service.response.fields) >= 2
    assert action.goal_service.response.fields[0].name == 'accepted'
    assert action.goal_service.response.fields[1].name == 'stamp'
    action.goal_service.response.fields.pop(1)
    action.goal_service.response.fields.pop(0)

    assert len(action.result_service.request.fields) >= 1
    assert action.result_service.request.fields[0].name == 'action_goal_id'
    action.result_service.request.fields.pop(0)
    assert len(action.result_service.response.fields) >= 1
    assert action.result_service.response.fields[0].name == 'action_status'
    action.result_service.response.fields.pop(0)

    assert len(action.feedback.fields) >= 1
    assert action.feedback.fields[0].name == 'action_goal_id'
    action.feedback.fields.pop(0)

    output_file = output_dir / input_file.with_suffix('.idl').name
    abs_output_file = output_file.absolute()
    print('Writing output file: {abs_output_file}'.format_map(locals()))
    data = {
        'pkg_name': package_name,
        'relative_input_file': input_file,
        'action': action,
    }

    expand_template('action.idl.em', data, output_file)
    return output_file
