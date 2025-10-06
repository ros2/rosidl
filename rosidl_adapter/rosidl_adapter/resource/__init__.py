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

from io import StringIO
import os
from pathlib import Path
import sys
from typing import Any, Optional, TypedDict

import em
from rosidl_adapter.parser import ActionSpecification, MessageSpecification, ServiceSpecification


try:
    from em import Configuration
    em_has_configuration = True
except ImportError:
    em_has_configuration = False


class Data(TypedDict):
    pkg_name: str
    relative_input_file: str


class MsgData(Data):
    msg: MessageSpecification


class SrvData(Data):
    srv: ServiceSpecification


class ActionData(Data):
    action: ActionSpecification


def expand_template(template_name: str, data: Data, output_file: Path,
                    encoding: str = 'utf-8') -> None:
    content = evaluate_template(template_name, data)

    if output_file.exists():
        existing_content = output_file.read_text(encoding=encoding)
        if existing_content == content:
            return
    elif output_file.parent:
        os.makedirs(str(output_file.parent), exist_ok=True)

    output_file.write_text(content, encoding=encoding)


_interpreter: Optional[em.Interpreter] = None


def evaluate_template(template_name: str, data: Data) -> str:
    global _interpreter
    # create copy before manipulating
    data_copy = dict(data)
    data_copy['TEMPLATE'] = _evaluate_template

    template_path = os.path.join(os.path.dirname(__file__), template_name)

    output = StringIO()
    try:
        if em_has_configuration:
            config = Configuration(
                defaultRoot=template_path,
                defaultStdout=output,
                deleteOnError=True,
                rawErrors=True,
                useProxy=True)
            _interpreter = em.Interpreter(
                config=config,
                dispatcher=False)
        else:
            _interpreter = em.Interpreter(
                output=output,
                options={
                    em.BUFFERED_OPT: True,
                    em.RAW_OPT: True,
                })
        with open(template_path, 'r') as h:
            content = h.read()
        _interpreter.invoke(
            'beforeFile', name=template_name, file=h, locals=data_copy)
        if em_has_configuration:
            _interpreter.string(content, locals=data_copy)
        else:
            _interpreter.string(content, template_path, locals=data_copy)
        _interpreter.invoke('afterFile')

        return output.getvalue()
    except Exception as e:  # noqa: F841
        print(
            f"{e.__class__.__name__} processing template '{template_name}'",
            file=sys.stderr)
        raise
    finally:
        if _interpreter is not None:
            _interpreter.shutdown()
        _interpreter = None


def _evaluate_template(template_name: str, **kwargs: Any) -> None:
    global _interpreter
    if _interpreter is None:
        raise RuntimeError('_evaluate_template called without running evaluate_template.')

    template_path = os.path.join(os.path.dirname(__file__), template_name)
    with open(template_path, 'r') as h:
        _interpreter.invoke(
            'beforeInclude', name=template_path, file=h, locals=kwargs)
        content = h.read()
    try:
        if em_has_configuration:
            _interpreter.string(content, locals=kwargs)
        else:
            _interpreter.string(content, template_path, kwargs)
    except Exception as e:  # noqa: F841
        print(
            f"{e.__class__.__name__} processing template '{template_name}': "
            f'{e}', file=sys.stderr)
        sys.exit(1)
    _interpreter.invoke('afterInclude')
