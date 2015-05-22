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
from io import StringIO
import os

from rosidl_cmake import extract_message_types
from rosidl_parser import parse_message_file
from rosidl_parser import parse_service_file
from rosidl_parser import validate_field_types


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

    known_msg_types = extract_message_types(pkg_name, ros_interface_files, deps)

    for ros_interface_file in ros_interface_files:
        filename, extension = os.path.splitext(ros_interface_file)
        if extension == '.msg':
            spec = parse_message_file(pkg_name, ros_interface_file)
            validate_field_types(spec, known_msg_types)
            for template_file, generated_filename in mapping_msgs.items():
                generated_file = os.path.join(output_dir, generated_filename % spec.base_type.type)

                try:
                    output = StringIO()
                    # TODO reuse interpreter
                    interpreter = em.Interpreter(
                        output=output,
                        options={
                            em.RAW_OPT: True,
                            em.BUFFERED_OPT: True,
                        },
                        globals={'spec': spec},
                    )
                    interpreter.file(open(template_file))
                    content = output.getvalue()
                    interpreter.shutdown()
                except Exception:
                    os.remove(generated_file)
                    raise

                # only overwrite file if necessary
                if os.path.exists(generated_file):
                    with open(generated_file, 'r') as h:
                        if h.read() == content:
                            continue
                with open(generated_file, 'w') as h:
                    h.write(content)

        elif extension == '.srv':
            spec = parse_service_file(pkg_name, ros_interface_file)
            validate_field_types(spec, known_msg_types)
            for template_file, generated_filename in mapping_srvs.items():
                generated_file = os.path.join(output_dir, generated_filename % spec.srv_name)

                try:
                    output = StringIO()
                    # TODO reuse interpreter
                    interpreter = em.Interpreter(
                        output=output,
                        options={
                            em.RAW_OPT: True,
                            em.BUFFERED_OPT: True,
                        },
                        globals={'spec': spec},
                    )
                    interpreter.file(open(template_file))
                    content = output.getvalue()
                    interpreter.shutdown()
                except Exception:
                    os.remove(generated_file)
                    raise

                # only overwrite file if necessary
                if os.path.exists(generated_file):
                    with open(generated_file, 'r') as h:
                        if h.read() == content:
                            continue
                with open(generated_file, 'w') as h:
                    h.write(content)

    return 0
