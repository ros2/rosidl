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

import em
import os

from rosidl_parser import parse_message_file, parse_service_file


def generate_cpp(pkg_name, ros_interface_files, deps, output_dir, template_dir):
    mapping_msgs = {
        os.path.join(template_dir, 'msg_TypeSupport_Introspection.cpp.template'):
        '%s_TypeSupport_Introspection.cpp',
    }

    mapping_srvs = {
        os.path.join(template_dir, 'srv_ServiceTypeSupport_Introspection.cpp.template'):
        '%s_ServiceTypeSupport_Introspection.cpp',
    }

    for template_file in mapping_msgs.keys():
        assert(os.path.exists(template_file))

    for template_file in mapping_srvs.keys():
        assert(os.path.exists(template_file))

    try:
        os.makedirs(output_dir)
    except FileExistsError:
        pass

    for ros_interface_file in ros_interface_files:
        filename, extension = os.path.splitext(ros_interface_file)
        if extension == '.msg':
            spec = parse_message_file(pkg_name, ros_interface_file)
            for template_file, generated_filename in mapping_msgs.items():
                generated_file = os.path.join(output_dir, generated_filename % spec.base_type.type)
                print('Generating MESSAGE: %s' % generated_file)

                try:
                    # TODO only touch generated file if its content actually changes
                    ofile = open(generated_file, 'w')
                    # TODO reuse interpreter
                    interpreter = em.Interpreter(
                        output=ofile,
                        options={
                            em.RAW_OPT: True,
                            em.BUFFERED_OPT: True,
                        },
                        globals={'spec': spec},
                    )
                    interpreter.file(open(template_file))
                    interpreter.shutdown()
                except Exception:
                    os.remove(generated_file)
                    raise

        elif extension == '.srv':
            spec = parse_service_file(pkg_name, ros_interface_file)
            for template_file, generated_filename in mapping_srvs.items():
                generated_file = os.path.join(output_dir, generated_filename % spec.srv_name)
                print('Generating SERVICE: %s' % generated_file)

                try:
                    # TODO only touch generated file if its content actually changes
                    ofile = open(generated_file, 'w')
                    # TODO reuse interpreter
                    interpreter = em.Interpreter(
                        output=ofile,
                        options={
                            em.RAW_OPT: True,
                            em.BUFFERED_OPT: True,
                        },
                        globals={'spec': spec},
                    )
                    interpreter.file(open(template_file))
                    interpreter.shutdown()
                except Exception:
                    os.remove(generated_file)
                    raise

    return 0
