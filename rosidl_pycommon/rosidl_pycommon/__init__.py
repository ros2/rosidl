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

import hashlib
from io import StringIO
import json
import os
import pathlib
import re
import sys

import em
from rosidl_parser import definition
from rosidl_parser.parser import parse_idl_file


def convert_camel_case_to_lower_case_underscore(value):
    # insert an underscore before any upper case letter
    # which is followed by a lower case letter
    value = re.sub('(.)([A-Z][a-z]+)', r'\1_\2', value)
    # insert an underscore before any upper case letter
    # which is preseded by a lower case letter or number
    value = re.sub('([a-z0-9])([A-Z])', r'\1_\2', value)
    return value.lower()


def read_generator_arguments(input_file):
    with open(input_file, mode='r', encoding='utf-8') as h:
        return json.load(h)


def get_newest_modification_time(target_dependencies):
    newest_timestamp = None
    for dep in target_dependencies:
        ts = os.path.getmtime(dep)
        if newest_timestamp is None or ts > newest_timestamp:
            newest_timestamp = ts
    return newest_timestamp


# This mapping must match the constants defined in type_description_interfaces/msgs/FieldType.msg
# TODO There is no FieldType.msg definition for the following rosidl_parser.definition types
# * SIGNED_NONEXPLICIT_INTEGER_TYPES = short, long, long long
# * UNSIGNED_NONEXPLICIT_INTEGER_TYPES = unsigned short, unsigned long, unsigned long long
# *
FIELD_TYPES = {
    'nested_type': 0,
    'int8': 1,
    'uint8': 2,
    'int16': 3,
    'uint16': 4,
    'int32': 5,
    'uint32': 6,
    'int64': 7,
    'uint64': 8,
    'float': 9,
    'double': 10,
    'long double': 11,
    'char': 12,
    'wchar': 13,
    'boolean': 14,
    'octet': 15,  # byte
    definition.UnboundedString: 16,
    definition.UnboundedWString: 17,
    # TODO there is no rosidl_parser.definition type for fixed strings (there is array of char, though?)
    # FIXED_STRING = 18
    # FIXED_WSTRING = 19
    definition.BoundedString: 20,
    definition.BoundedWString: 21,
}

NESTED_FIELD_TYPE_OFFSETS = {
    definition.Array: 32,
    definition.BoundedSequence: 64,
    definition.UnboundedSequence: 96,
}


def translate_type_id(value_type, offset, result):
    if isinstance(value_type, definition.BasicType):
        result['type_id'] = FIELD_TYPES[value_type.typename] + offset
    elif isinstance(value_type, definition.AbstractGenericString):
        result['type_id'] = FIELD_TYPES[type(value_type)] + offset
        if value_type.has_maximum_size():
            result['string_length'] = value_type.maximum_size
    elif isinstance(value_type, definition.NamespacedType):
        result['type_id'] = offset
        result['nested_type_name'] = '/'.join(value_type.namespaced_name())
    elif isinstance(value_type, definition.NamedType):
        result['type_id'] = offset
        result['nested_type_name'] = value_type.name
    else:
        raise TypeError('Unknown value type ', value_type)


def field_type(ftype: definition.AbstractType):
    result = {
        'type_id': 0,
        'length': 0,
        'string_length': 0,
        'nested_type_name': '',
    }

    if isinstance(ftype, definition.AbstractNestableType):
        translate_type_id(ftype, 0, result)
    elif isinstance(ftype, definition.AbstractNestedType):
        type_id_offset = NESTED_FIELD_TYPE_OFFSETS[type(ftype)]
        translate_type_id(ftype.value_type, type_id_offset, result)
        if ftype.has_maximum_size():
            try:
                result['length'] = ftype.maximum_size
            except AttributeError:
                result['length'] = ftype.size
    else:
        print(ftype)
        raise Exception('idk that type type!', ftype)

    return result


def field(member: definition.Member):
    return {
        'name': member.name,
        'type': field_type(member.type),
        # skipping default_value
    }


def individual_type_description(msg: definition.Message):
    fields = [field(member) for member in msg.structure.members]
    # referenced_types = [f['type']['nested_type_name'] for f in fields]
    # referenced_types = [f for f in referenced_types if f != '']
    return {
        'type_name': '/'.join(msg.structure.namespaced_type.namespaced_name()),
        'fields': fields
    }


def generate_type_version_hash(id_triplet, idl_files):
    idl = idl_files[id_triplet]

    includes = []
    messages = []
    services = []
    actions = []
    for el in idl.content.elements:
        if isinstance(el, definition.Include):
            includes.append(el)
            print(f'  Include: {el.locator}')
        elif isinstance(el, definition.Message):
            messages.append(el)
            print(f'  Message: {el.structure.namespaced_type.namespaces} / {el.structure.namespaced_type.name}')
        elif isinstance(el, definition.Service):
            services.append(el)
            print(f'  Service: {el.namespaced_type.name}')
        elif isinstance(el, definition.Action):
            actions.append(el)
            print(el)
        else:
            raise Exception(f'Do not know how to hash {el}')

    # Per rosidl_parser.parser.extract_content_from_ast,
    # IDL may have only one of Message, Service, or Action to be parsed
    total_interfaces = len(messages) + len(services) + len(actions)
    if total_interfaces < 1:
        raise Exception('No interface defined in IDL, cannot hash')
    if total_interfaces > 1:
        raise Exception('More than one ROS interface defined in IDL, cannot hash')

    if len(messages):
        serialization_data = {
            'type_description': individual_type_description(messages[0]),
            'referenced_type_descriptions': [],  # TODO referenced type descriptions
        }
        # TODO remove indent
        serialized_type_description = json.dumps(serialization_data, indent=2)
        print(serialized_type_description)
        m = hashlib.sha256()
        m.update(serialized_type_description.encode('utf-8'))
        return m.hexdigest()


def generate_files(
    generator_arguments_file, mapping, additional_context=None,
    keep_case=False, post_process_callback=None
):
    args = read_generator_arguments(generator_arguments_file)

    template_basepath = pathlib.Path(args['template_dir'])
    for template_filename in mapping.keys():
        assert (template_basepath / template_filename).exists(), \
            'Could not find template: ' + template_filename

    latest_target_timestamp = get_newest_modification_time(args['target_dependencies'])
    generated_files = []

    # idl_locators = {}
    idl_ids_to_generate = []
    idl_files = {}
    package_name = args['package_name']
    for idl_tuple in args.get('idl_tuples', []):
        idl_parts = idl_tuple.rsplit(':', 1)
        assert len(idl_parts) == 2
        print(idl_parts)
        locator = definition.IdlLocator(*idl_parts)

        idl_rel_path = pathlib.Path(idl_parts[1])
        namespace = str(idl_rel_path.parent)
        print(idl_rel_path)
        idl_stem = idl_rel_path.stem
        id_triplet = (package_name, namespace, idl_stem)
        try:
            print('Parsing ', id_triplet)
            idl_files[id_triplet] = parse_idl_file(locator)
            idl_ids_to_generate.append(id_triplet)
        except Exception as e:
            print(
                'Error processing idl file: ' +
                str(locator.get_absolute_path()), file=sys.stderr)
            raise(e)

    for interface_dep in args.get('ros_interface_dependencies', []):
        tuple_parts = interface_dep.rsplit(':', 1)
        assert len(tuple_parts) == 2
        print(tuple_parts)
        referenced_package_name, idl_abs_path = tuple_parts

        base_path, sep, rel_path = idl_abs_path.rpartition(referenced_package_name)
        locator = definition.IdlLocator(idl_abs_path, '')

        namespace = pathlib.Path(idl_abs_path).parents[0].name
        idl_stem = pathlib.Path(idl_abs_path).stem
        id_triplet = (referenced_package_name, namespace, idl_stem)
        try:
            print('Parsing ', id_triplet)
            idl_files[id_triplet] = parse_idl_file(locator)
        except Exception as e:
            print(
                'Error processing idl file: ' +
                str(locator.get_absolute_path()), file=sys.stderr)
            raise(e)

    for id_triplet in idl_ids_to_generate:
        _, _, idl_stem = id_triplet
        if not keep_case:
            idl_stem = convert_camel_case_to_lower_case_underscore(idl_stem)

        print(id_triplet)
        type_hash = generate_type_version_hash(id_triplet, idl_files)
        print(type_hash)

        idl_file = idl_files[id_triplet]
        for template_file, generated_filename in mapping.items():
            generated_file = os.path.join(
                args['output_dir'], str(idl_rel_path.parent),
                generated_filename % idl_stem)
            generated_files.append(generated_file)
            data = {
                'package_name': package_name,
                'interface_path': idl_rel_path,
                'content': idl_file.content,
                'type_hash': type_hash,
            }
            if additional_context is not None:
                data.update(additional_context)
            expand_template(
                os.path.basename(template_file), data,
                generated_file, minimum_timestamp=latest_target_timestamp,
                template_basepath=template_basepath,
                post_process_callback=post_process_callback)

    return generated_files


template_prefix_path = []


def get_template_path(template_name):
    global template_prefix_path
    for basepath in template_prefix_path:
        template_path = basepath / template_name
        if template_path.exists():
            return template_path
    raise RuntimeError(f"Failed to find template '{template_name}'")


interpreter = None


def expand_template(
    template_name, data, output_file, minimum_timestamp=None,
    template_basepath=None, post_process_callback=None
):
    # in the legacy API the first argument was the path to the template
    if template_basepath is None:
        template_name = pathlib.Path(template_name)
        template_basepath = template_name.parent
        template_name = template_name.name

    global interpreter
    output = StringIO()
    interpreter = em.Interpreter(
        output=output,
        options={
            em.BUFFERED_OPT: True,
            em.RAW_OPT: True,
        },
    )

    global template_prefix_path
    template_prefix_path.append(template_basepath)
    template_path = get_template_path(template_name)

    # create copy before manipulating
    data = dict(data)
    _add_helper_functions(data)

    try:
        with template_path.open('r') as h:
            template_content = h.read()
            interpreter.invoke(
                'beforeFile', name=template_name, file=h, locals=data)
        interpreter.string(template_content, template_path, locals=data)
        interpreter.invoke('afterFile')
    except Exception as e:  # noqa: F841
        if os.path.exists(output_file):
            os.remove(output_file)
        print(f"{e.__class__.__name__} when expanding '{template_name}' into "
              f"'{output_file}': {e}", file=sys.stderr)
        raise
    finally:
        template_prefix_path.pop()

    content = output.getvalue()
    interpreter.shutdown()

    if post_process_callback:
        content = post_process_callback(content)

    # only overwrite file if necessary
    # which is either when the timestamp is too old or when the content is different
    if os.path.exists(output_file):
        timestamp = os.path.getmtime(output_file)
        if minimum_timestamp is None or timestamp > minimum_timestamp:
            with open(output_file, 'r', encoding='utf-8') as h:
                if h.read() == content:
                    return
    else:
        # create folder if necessary
        try:
            os.makedirs(os.path.dirname(output_file))
        except FileExistsError:
            pass

    with open(output_file, 'w', encoding='utf-8') as h:
        h.write(content)


def _add_helper_functions(data):
    data['TEMPLATE'] = _expand_template


def _expand_template(template_name, **kwargs):
    global interpreter
    template_path = get_template_path(template_name)
    _add_helper_functions(kwargs)
    with template_path.open('r') as h:
        interpreter.invoke(
            'beforeInclude', name=str(template_path), file=h, locals=kwargs)
        content = h.read()
    try:
        interpreter.string(content, str(template_path), kwargs)
    except Exception as e:  # noqa: F841
        print(f"{e.__class__.__name__} in template '{template_path}': {e}",
              file=sys.stderr)
        raise
    interpreter.invoke('afterInclude')
