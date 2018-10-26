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

from rosidl_cmake import convert_camel_case_to_lower_case_underscore
from rosidl_cmake import expand_template
from rosidl_cmake import get_newest_modification_time
from rosidl_cmake import read_generator_arguments
from rosidl_parser import parse_action_file


def generate_msg_and_srv(generator_arguments_file):
    args = read_generator_arguments(generator_arguments_file)

    for ros_interface_file in args['ros_interface_files']:
        extension = os.path.splitext(ros_interface_file)[1]
        subfolder = os.path.basename(os.path.dirname(ros_interface_file))
        if extension == '.action':
            services, message = parse_action_file(args['package_name'], ros_interface_file)

            # create folder if necessary
            try:
                os.makedirs(os.path.join(args['output_dir'], subfolder))
            except FileExistsError:
                pass

            for service in services:
                generated_file = os.path.join(
                    args['output_dir'], subfolder, service.srv_name + '.srv')
                with open(generated_file, 'w+') as fout:
                    fout.write(str(service))

            generated_file = os.path.join(args['output_dir'], subfolder, message.msg_name + '.msg')
            with open(generated_file, 'w+') as fout:
                fout.write(str(message))
