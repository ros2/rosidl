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

import os

from rosidl_cmake import read_generator_arguments
from rosidl_parser import parse_action_file


def generate_msg_and_srv(generator_arguments_file):
    """Write 'msg' and 'srv' files from 'action' files."""
    args = read_generator_arguments(generator_arguments_file)

    for ros_interface_file in args['ros_interface_files']:
        extension = os.path.splitext(ros_interface_file)[1]
        subfolder = os.path.basename(os.path.dirname(ros_interface_file))
        if extension == '.action':
            action = parse_action_file(args['package_name'], ros_interface_file)

            # create folder if necessary
            os.makedirs(os.path.join(args['output_dir'], subfolder), exist_ok=True)

            generated_folder = os.path.join(args['output_dir'], subfolder)
            for service in [action.goal_service, action.result_service]:
                srv_file = os.path.join(generated_folder, service.srv_name + '.srv')
                req_file = os.path.join(generated_folder, service.srv_name + '_Request.msg')
                rsp_file = os.path.join(generated_folder, service.srv_name + '_Response.msg')
                with open(srv_file, 'w+') as fout:
                    fout.write(str(service))
                with open(req_file, 'w+') as fout:
                    fout.write(str(service.request))
                with open(rsp_file, 'w+') as fout:
                    fout.write(str(service.response))

            generated_file = os.path.join(
                args['output_dir'], subfolder, action.feedback.msg_name + '.msg')
            with open(generated_file, 'w+') as fout:
                fout.write(str(action.feedback))
