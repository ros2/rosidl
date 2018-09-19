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
import pathlib

from lark import Lark
from lark.lexer import Token
from lark.tree import Tree

from rosidl_parser import Field
from rosidl_parser import MessageSpecification
from rosidl_parser import ServiceSpecification
from rosidl_parser import Type

with open(os.path.join(os.path.dirname(__file__), 'grammar.lark'), 'r') as h:
    grammar = h.read()

parser = Lark(grammar, start='specification')


def parse_idl_file(package_name, namespace, file_):
    with open(file_, 'r') as h:
        content = h.read()
    try:
        return parse_idl_string(
            package_name, namespace, pathlib.Path(file_).stem, content)
    except Exception as e:
        print(file_)
        raise


def parse_idl_string(package_name, namespace, interface_name, idl_string):
    global parser
    tree = parser.parse(idl_string)
    c = count(tree, 'struct_def')

    if c == 1:
        msg = MessageSpecification(package_name, namespace, interface_name, [], [])
        visit_tree(tree, msg)
        return msg

    if c == 2:
        srv = ServiceSpecification(
            package_name, interface_name,
            MessageSpecification(
                package_name, namespace, interface_name + '_Request', [], []),
            MessageSpecification(
                package_name, namespace, interface_name + '_Response', [], []))
        visit_tree(tree, srv)
        return srv

    assert False, 'Unsupported %d: %s' % (c, tree)


def count(tree, data):
    if tree.data == data:
        return 1
    c = 0
    for child in tree.children:
        if isinstance(child, Tree):
            c += count(child, data)
    return c


def visit_tree(tree, spec):
    if tree.data == 'struct_def':
        visit_struct_def(tree, spec)
    for c in tree.children:
        if isinstance(c, Tree):
            visit_tree(c, spec)


def visit_struct_def(tree, spec):
    assert tree.data == 'struct_def'
    assert len(tree.children) >= 1
    c = tree.children[0]
    assert isinstance(c, Token)
    assert c.type == 'IDENTIFIER'
    if isinstance(spec, MessageSpecification):
        assert c.value == spec.msg_name
        msg = spec
    if isinstance(spec, ServiceSpecification):
        if c.value == spec.srv_name + '_Request':
            msg = spec.request
        elif c.value == spec.srv_name + '_Response':
            msg = spec.response
        else:
            assert False

    for c in tree.children[1:]:
        if not isinstance(c, Tree):
            continue
        if c.data == 'member':
            visit_member(c, msg)


def get_scoped_name(tree):
    assert tree.data == 'scoped_name'
    scoped_name = []
    if len(tree.children) == 2:
        c = tree.children[0]
        assert c.data == 'scoped_name'
        scoped_name += get_scoped_name(c)
    c = tree.children[-1]
    assert isinstance(c, Token)
    assert c.type == 'IDENTIFIER'
    scoped_name.append(c.value)
    return scoped_name


def visit_member(tree, msg):
    assert tree.data == 'member'
    assert len(tree.children) == 2

    c = tree.children[0]
    assert c.data == 'type_spec'
    assert len(c.children) == 1
    c = c.children[0]
    if c.data == 'simple_type_spec':
        assert len(c.children) == 1
        c = c.children[0]
        assert c.data == 'scoped_name'
        field_type = '::'.join(get_scoped_name(c))
    else:
        field_type = None

    c = tree.children[1]
    assert c.data == 'declarators'
    assert len(c.children) == 1
    c = c.children[0]
    assert c.data == 'declarator'
    assert len(c.children) == 1
    c = c.children[0]
    assert c.data == 'simple_declarator'
    assert len(c.children) == 1
    c = c.children[0]
    assert isinstance(c, Token)
    assert c.type == 'IDENTIFIER'
    field_name = c.value

    if field_type is not None:
        # TODO extract array typedefs from AST and resolve them correctly
        parts = field_type.split('__')
        try:
            if str(int(parts[-1])) == parts[-1]:
                field_type = '::'.join(parts[:-1]) + '[' + parts[-1] + ']'
        except ValueError:
            pass
        msg.fields.append(
            Field(
                Type(field_type, context_package_name=msg.base_type.pkg_name),
                field_name))
