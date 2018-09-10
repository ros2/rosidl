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

from io import StringIO
import json
import os
import re
import sys

import em

from rosidl_parser import BaseType
from rosidl_parser import PACKAGE_NAME_MESSAGE_TYPE_SEPARATOR


def convert_camel_case_to_lower_case_underscore(value):
    # insert an underscore before any upper case letter
    # which is not followed by another upper case letter
    value = re.sub('(.)([A-Z][a-z]+)', r'\1_\2', value)
    # insert an underscore before any upper case letter
    # which is preseded by a lower case letter or number
    value = re.sub('([a-z0-9])([A-Z])', r'\1_\2', value)
    return value.lower()


def extract_message_types(pkg_name, ros_interface_files, deps):
    types = []

    for ros_interface_file in ros_interface_files:
        base_type = _get_base_type(pkg_name, ros_interface_file)
        if base_type:
            types.append(base_type)

    for dep in deps:
        # only take the first : for separation, as Windows follows with a C:\
        dep_parts = dep.split(':', 1)
        assert len(dep_parts) == 2, "The dependency '%s' must contain a double colon" % dep
        pkg_name = dep_parts[0]
        base_type = _get_base_type(pkg_name, dep_parts[1])
        if base_type:
            types.append(base_type)

    return types


def _get_base_type(pkg_name, idl_path):
    idl_filename = os.path.basename(idl_path)
    msg_name, extension = os.path.splitext(idl_filename)
    if extension != '.msg':
        return None
    return BaseType(pkg_name + PACKAGE_NAME_MESSAGE_TYPE_SEPARATOR + msg_name)


def read_generator_arguments(input_file):
    with open(input_file, 'r') as h:
        return json.load(h)


def get_newest_modification_time(target_dependencies):
    newest_timestamp = None
    for dep in target_dependencies:
        ts = os.path.getmtime(dep)
        if newest_timestamp is None or ts > newest_timestamp:
            newest_timestamp = ts
    return newest_timestamp


expand_template_durations = []


def expand_template(template_file, data, output_file, minimum_timestamp=None):
    global expand_template_durations
    print('XXX', template_file)
    import time
    start = time.time()
    output = StringIO()
    interpreter = CachingInterpreter(
        output=output,
        options={
            em.BUFFERED_OPT: True,
            em.RAW_OPT: True,
        },
        globals=data,
    )
    with open(template_file, 'r') as h:
        try:
            interpreter.file(h)
        except Exception:
            if os.path.exists(output_file):
                os.remove(output_file)
            print("Exception when expanding '%s' into '%s'" %
                  (template_file, output_file), file=sys.stderr)
            raise
    content = output.getvalue()
    interpreter.shutdown()
    end = time.time()
    duration = end - start
    print('expand_template() took %f s' % duration)
    expand_template_durations.append(duration)
    print(
        '  avg %f / %d = %s' % (
            sum(expand_template_durations),
            len(expand_template_durations),
            sum(expand_template_durations) / len(expand_template_durations)))

    # only overwrite file if necessary
    # which is either when the timestamp is too old or when the content is different
    if os.path.exists(output_file):
        timestamp = os.path.getmtime(output_file)
        if minimum_timestamp is None or timestamp > minimum_timestamp:
            with open(output_file, 'r') as h:
                if h.read() == content:
                    return
    else:
        # create folder if necessary
        try:
            os.makedirs(os.path.dirname(output_file))
        except FileExistsError:
            pass

    with open(output_file, 'w') as h:
        h.write(content)


cached_tokens = {}


class CachingInterpreter(em.Interpreter):

    def parse(self, scanner, locals=None):  # noqa: A002
        global cached_tokens
        data = scanner.buffer
        assert isinstance(data, str)
        # try to use cached tokens
        tokens = cached_tokens.get(data)
        if tokens is None or True:
            # collect tokens and cache them
            tokens = []
            while True:
                token = scanner.one()
                if token is None:
                    break
                tokens.append(token)
            if data in cached_tokens:
                _compare(cached_tokens[data], tokens)
            cached_tokens[data] = tokens
        else:
            print('hit')

        # import copy
        # tokens = copy.deepcopy(tokens)

        # reimplement the parse method using the (cached) tokens
        self.invoke('atParse', scanner=scanner, locals=locals)
        for token in tokens:
            self.invoke('atToken', token=token)
            token.run(self, locals)


def _compare(left, right):
    if isinstance(left, str) and isinstance(right, str):
        assert left == right
        return
    if isinstance(left, list) and isinstance(right, list):
        # print('   ', 'list - list')
        assert len(left) == len(right)
        # print('   ', 'len(list)', len(left))
        for i in range(len(left)):
            # print('   ', 'list[i] =', i)
            _compare(left[i], right[i])
        return
    # if type(left) != type(right):
    #     print('   ', type(left), type(right))
    assert type(left) == type(right)
    # print(type(left))
    if isinstance(left, object) and isinstance(right, object):
        if left.__dict__ != right.__dict__:
            # print('   ', 'dict keys...')
            _compare(list(left.__dict__.keys()), list(right.__dict__.keys()))
            for k in left.__dict__.keys():
                _compare(left.__dict__[k], right.__dict__[k])
            return
        assert left.__dict__ == right.__dict__
        return
    assert False
    # left.__dict__
    # right.__dict__
    # assert left == right, '%s: %s != %s' % (type(left), left, right)
