# Copyright 2026 Open Source Robotics Foundation, Inc.
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

import json
import pathlib
from typing import Any, Dict, Union

from rosidl_parser import definition


def type_to_dict(t: definition.AbstractType) -> Dict[str, Any]:
    if isinstance(t, definition.BasicType):
        return {'type': 'BasicType', 'typename': t.typename}
    elif isinstance(t, definition.NamespacedType):
        return {'type': 'NamespacedType', 'namespaces': t.namespaces, 'name': t.name}
    elif isinstance(t, definition.NamedType):
        return {'type': 'NamedType', 'name': t.name}
    elif isinstance(t, definition.BoundedString):
        return {'type': 'BoundedString', 'maximum_size': t.maximum_size}
    elif isinstance(t, definition.UnboundedString):
        return {'type': 'UnboundedString'}
    elif isinstance(t, definition.BoundedWString):
        return {'type': 'BoundedWString', 'maximum_size': t.maximum_size}
    elif isinstance(t, definition.UnboundedWString):
        return {'type': 'UnboundedWString'}
    elif isinstance(t, definition.Array):
        return {
            'type': 'Array',
            'value_type': type_to_dict(t.value_type),
            'size': t.size,
        }
    elif isinstance(t, definition.BoundedSequence):
        return {
            'type': 'BoundedSequence',
            'value_type': type_to_dict(t.value_type),
            'maximum_size': t.maximum_size,
        }
    elif isinstance(t, definition.UnboundedSequence):
        return {
            'type': 'UnboundedSequence',
            'value_type': type_to_dict(t.value_type),
        }
    raise ValueError(f'Unknown type: {t}')


def dict_to_nestable_type(d: Dict[str, Any]) -> definition.AbstractNestableType:
    val_type = dict_to_type(d)
    assert isinstance(val_type, definition.AbstractNestableType)
    return val_type


def dict_to_type(d: Dict[str, Any]) -> definition.AbstractType:
    kind = d['type']
    if kind == 'BasicType':
        return definition.BasicType(d['typename'])
    elif kind == 'NamespacedType':
        return definition.NamespacedType(d['namespaces'], d['name'])
    elif kind == 'NamedType':
        return definition.NamedType(d['name'])
    elif kind == 'BoundedString':
        return definition.BoundedString(d['maximum_size'])
    elif kind == 'UnboundedString':
        return definition.UnboundedString()
    elif kind == 'BoundedWString':
        return definition.BoundedWString(d['maximum_size'])
    elif kind == 'UnboundedWString':
        return definition.UnboundedWString()
    elif kind == 'Array':
        return definition.Array(dict_to_nestable_type(d['value_type']), d['size'])
    elif kind == 'BoundedSequence':
        return definition.BoundedSequence(
            dict_to_nestable_type(d['value_type']), d['maximum_size'])
    elif kind == 'UnboundedSequence':
        return definition.UnboundedSequence(dict_to_nestable_type(d['value_type']))
    raise ValueError(f'Unknown type kind: {kind}')


def annotation_to_dict(ann: definition.Annotation) -> Dict[str, Any]:
    return {'name': ann.name, 'value': ann.value}


def dict_to_annotation(d: Dict[str, Any]) -> definition.Annotation:
    return definition.Annotation(d['name'], d['value'])


def member_to_dict(m: definition.Member) -> Dict[str, Any]:
    return {
        'type': type_to_dict(m.type),
        'name': m.name,
        'annotations': [annotation_to_dict(a) for a in m.annotations],
    }


def dict_to_member(d: Dict[str, Any]) -> definition.Member:
    m = definition.Member(dict_to_type(d['type']), d['name'])
    m.annotations = [dict_to_annotation(a) for a in d.get('annotations', [])]
    return m


def constant_to_dict(c: definition.Constant) -> Dict[str, Any]:
    return {
        'name': c.name,
        'type': type_to_dict(c.type),
        'value': c.value,
        'annotations': [annotation_to_dict(a) for a in c.annotations],
    }


def dict_to_constant(d: Dict[str, Any]) -> definition.Constant:
    c = definition.Constant(d['name'], dict_to_type(d['type']), d['value'])
    c.annotations = [dict_to_annotation(a) for a in d.get('annotations', [])]
    return c


def structure_to_dict(s: definition.Structure) -> Dict[str, Any]:
    return {
        'namespaced_type': {
            'namespaces': s.namespaced_type.namespaces,
            'name': s.namespaced_type.name,
        },
        'members': [member_to_dict(m) for m in s.members],
        'annotations': [annotation_to_dict(a) for a in s.annotations],
    }


def dict_to_structure(d: Dict[str, Any]) -> definition.Structure:
    nst = definition.NamespacedType(
        d['namespaced_type']['namespaces'], d['namespaced_type']['name'])
    members = [dict_to_member(m) for m in d['members']]
    s = definition.Structure(nst, members=members)
    s.annotations = [dict_to_annotation(a) for a in d.get('annotations', [])]
    return s


def message_to_dict(m: definition.Message) -> Dict[str, Any]:
    return {
        'structure': structure_to_dict(m.structure),
        'constants': [constant_to_dict(c) for c in m.constants],
    }


def dict_to_message(d: Dict[str, Any]) -> definition.Message:
    msg = definition.Message(dict_to_structure(d['structure']))
    msg.constants = [dict_to_constant(c) for c in d.get('constants', [])]
    return msg


def service_to_dict(s: definition.Service) -> Dict[str, Any]:
    return {
        'namespaced_type': {
            'namespaces': s.namespaced_type.namespaces,
            'name': s.namespaced_type.name,
        },
        'request': message_to_dict(s.request_message),
        'response': message_to_dict(s.response_message),
    }


def dict_to_service(d: Dict[str, Any]) -> definition.Service:
    nst = definition.NamespacedType(
        d['namespaced_type']['namespaces'], d['namespaced_type']['name'])
    req = dict_to_message(d['request'])
    resp = dict_to_message(d['response'])
    return definition.Service(nst, req, resp)


def action_to_dict(a: definition.Action) -> Dict[str, Any]:
    return {
        'namespaced_type': {
            'namespaces': a.namespaced_type.namespaces,
            'name': a.namespaced_type.name,
        },
        'goal': message_to_dict(a.goal),
        'result': message_to_dict(a.result),
        'feedback': message_to_dict(a.feedback),
    }


def dict_to_action(d: Dict[str, Any]) -> definition.Action:
    nst = definition.NamespacedType(
        d['namespaced_type']['namespaces'], d['namespaced_type']['name'])
    goal = dict_to_message(d['goal'])
    result = dict_to_message(d['result'])
    feedback = dict_to_message(d['feedback'])
    return definition.Action(nst, goal, result, feedback)


def idl_content_to_dict(content: definition.IdlContent) -> Dict[str, Any]:
    elements: List[Dict[str, Any]] = []
    for el in content.elements:
        if isinstance(el, definition.Include):
            elements.append({'kind': 'Include', 'locator': el.locator})
        elif isinstance(el, definition.Message):
            elements.append({'kind': 'Message', 'data': message_to_dict(el)})
        elif isinstance(el, definition.Service):
            elements.append({'kind': 'Service', 'data': service_to_dict(el)})
        elif isinstance(el, definition.Action):
            elements.append({'kind': 'Action', 'data': action_to_dict(el)})
        else:
            raise ValueError(f'Unknown element type: {el}')
    return {'elements': elements}


def dict_to_idl_content(d: Dict[str, Any]) -> definition.IdlContent:
    content = definition.IdlContent()
    for el in d['elements']:
        kind = el['kind']
        if kind == 'Include':
            content.elements.append(definition.Include(el['locator']))
        elif kind == 'Message':
            content.elements.append(dict_to_message(el['data']))
        elif kind == 'Service':
            content.elements.append(dict_to_service(el['data']))
        elif kind == 'Action':
            content.elements.append(dict_to_action(el['data']))
        else:
            raise ValueError(f'Unknown element kind: {kind}')
    return content


def save_ast_json(content: definition.IdlContent, path: Union[str, pathlib.Path]) -> None:
    """Save an IdlContent AST to a JSON file."""
    p = pathlib.Path(path)
    d = idl_content_to_dict(content)
    p.write_text(json.dumps(d, separators=(',', ':')), encoding='utf-8')


def load_ast_json(path: Union[str, pathlib.Path]) -> definition.IdlContent:
    """Load an IdlContent AST from a JSON file."""
    p = pathlib.Path(path)
    d = json.loads(p.read_text(encoding='utf-8'))
    return dict_to_idl_content(d)
