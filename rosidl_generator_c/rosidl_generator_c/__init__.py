# Copyright 2015 Open Source Robotics Foundation, Inc.
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

from rosidl_cmake import convert_camel_case_to_lower_case_underscore
from rosidl_parser import parse_message_file


def generate_c(pkg_name, ros_interface_files, deps, output_dir, template_dir):
    mapping = {
        os.path.join(template_dir, 'msg.h.template'): '%s.h',
        os.path.join(template_dir, 'msg__struct.h.template'): '%s__struct.h',
    }
    for template_file in mapping.keys():
        assert os.path.exists(template_file), 'Could not find template: ' + template_file

    functions = {
        'get_header_filename_from_msg_name': convert_camel_case_to_lower_case_underscore,
    }

    for ros_interface_file in ros_interface_files:
        extension = os.path.splitext(ros_interface_file)[1]
        subfolder = os.path.basename(os.path.dirname(ros_interface_file))
        if extension == '.msg':
            spec = parse_message_file(pkg_name, ros_interface_file)
            for template_file, generated_filename in mapping.items():
                generated_file = os.path.join(
                    output_dir, subfolder, generated_filename %
                    convert_camel_case_to_lower_case_underscore(spec.base_type.type))

                try:
                    output = StringIO()
                    data = {'spec': spec, 'subfolder': subfolder}
                    data.update(functions)
                    # TODO reuse interpreter
                    interpreter = em.Interpreter(
                        output=output,
                        options={
                            em.RAW_OPT: True,
                            em.BUFFERED_OPT: True,
                        },
                        globals=data,
                    )
                    interpreter.file(open(template_file))
                    content = output.getvalue()
                    interpreter.shutdown()
                except Exception:
                    if os.path.exists(generated_file):
                        os.remove(generated_file)
                    raise

                # only overwrite file if necessary
                if os.path.exists(generated_file):
                    with open(generated_file, 'r') as h:
                        if h.read() == content:
                            continue
                try:
                    os.makedirs(os.path.dirname(generated_file))
                except FileExistsError:
                    pass
                with open(generated_file, 'w') as h:
                    h.write(content)
    return 0


MSG_TYPE_TO_C = {
    'bool': 'bool',
    'byte': 'uint8_t',
    'char': 'char',
    'float32': 'float',
    'float64': 'double',
    'uint8': 'uint8_t',
    'int8': 'int8_t',
    'uint16': 'uint16_t',
    'int16': 'int16_t',
    'uint32': 'uint32_t',
    'int32': 'int32_t',
    'uint64': 'uint64_t',
    'int64': 'int64_t',
    'string': "char *",
}


def msg_type_to_c(type_, name_, containing_msg_name='notset'):
    """
    Convert a message type into the C declaration.

    Example input: uint32, std_msgs/String
    Example output: uint32_t, char *

    @param type_: The message type
    @type type_: rosidl_parser.Type
    @param type_: The field name
    @type type_: str
    """
    c_type = None
    if type_.is_primitive_type():
        c_type = MSG_TYPE_TO_C[type_.type]
    else:
        c_type = '%s__%s' % (type_.pkg_name, type_.type)

    if type_.is_array:
        if type_.array_size is None:
            # Dynamic sized array
            return 'ROSIDL_Array__%s %s' % (type_.type, name_)
        else:
            # Static sized array (field specific)
            return 'ROSIDL_Array__%s__%s %s' % \
                (containing_msg_name, name_, name_)
    else:
        return '%s %s' % (c_type, name_)
