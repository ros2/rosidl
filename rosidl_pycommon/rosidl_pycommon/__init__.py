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

import inspect
from io import StringIO
import json
import os
import pathlib
import re
import sys
from typing import Any, Callable, Dict, List, Optional

import em

try:
    from em import Configuration
    em_has_configuration = True
except ImportError:
    em_has_configuration = False

from rosidl_parser.cache import compute_cache_key
from rosidl_parser.cache import get_package_version
from rosidl_parser.cache import restore_files_from_cache
from rosidl_parser.cache import save_files_to_cache
from rosidl_parser.definition import IdlLocator
from rosidl_parser.parser import parse_idl_file


def _get_caller_package_name() -> Optional[str]:
    frame = inspect.currentframe()
    try:
        # Walk up: _get_caller_package_name -> generate_files -> caller
        caller = frame.f_back.f_back if frame and frame.f_back else None
        if caller is None:
            return None
        module = caller.f_globals.get('__name__', '')
        # e.g. 'rosidl_generator_cpp' from 'rosidl_generator_cpp.__init__'
        return module.split('.')[0] if module else None
    finally:
        del frame


def convert_camel_case_to_lower_case_underscore(value: str) -> str:
    # insert an underscore before any upper case letter
    # which is followed by a lower case letter
    value = re.sub('(.)([A-Z][a-z]+)', r'\1_\2', value)
    # insert an underscore before any upper case letter
    # which is preseded by a lower case letter or number
    value = re.sub('([a-z0-9])([A-Z])', r'\1_\2', value)
    return value.lower()


def read_generator_arguments(input_file: str) -> Any:
    with open(input_file, mode='r', encoding='utf-8') as h:
        return json.load(h)


def get_newest_modification_time(
    target_dependencies: List[str]
) -> Optional[float]:
    newest_timestamp = None
    for dep in target_dependencies:
        ts = os.path.getmtime(dep)
        if newest_timestamp is None or ts > newest_timestamp:
            newest_timestamp = ts
    return newest_timestamp


def generate_files(
    generator_arguments_file: str, mapping: Dict[str, str],
    additional_context: Optional[Dict[str, bool]] = None,
    keep_case: bool = False, post_process_callback: Optional[Callable[[str], str]] = None
) -> List[str]:
    args = read_generator_arguments(generator_arguments_file)

    template_basepath = pathlib.Path(args['template_dir'])
    for template_filename in mapping.keys():
        assert (template_basepath / template_filename).exists(), \
            'Could not find template: ' + template_filename

    latest_target_timestamp = get_newest_modification_time(args['target_dependencies'])
    generated_files: List[str] = []
    output_dir = args['output_dir']

    type_description_files = {}
    for description_tuple in args.get('type_description_tuples', []):
        tuple_parts = description_tuple.split(':', 1)
        assert len(tuple_parts) == 2
        type_description_files[tuple_parts[0]] = tuple_parts[1]
    ros_interface_files = {}
    for ros_interface_file in args.get('ros_interface_files',  []):
        p = pathlib.Path(ros_interface_file)
        # e.g. ('msg', 'Empty')
        key = (p.suffix[1:], p.stem)
        ros_interface_files[key] = p

    for idl_tuple in args.get('idl_tuples', []):
        idl_parts = idl_tuple.rsplit(':', 1)
        assert len(idl_parts) == 2
        locator = IdlLocator(*idl_parts)
        idl_rel_path = pathlib.Path(idl_parts[1])

        type_description_info = None
        if type_description_files:
            type_hash_file = type_description_files[idl_parts[1]]
            with open(type_hash_file, 'r') as f:
                type_description_info = json.load(f)

        idl_stem = idl_rel_path.stem
        type_source_key = (idl_rel_path.parts[-2], idl_stem)
        type_source_file = ros_interface_files.get(type_source_key, locator.get_absolute_path())
        if not keep_case:
            idl_stem = convert_camel_case_to_lower_case_underscore(idl_stem)

        output_mapping = {
            template_file: os.path.join(str(idl_rel_path.parent), generated_filename % idl_stem)
            for template_file, generated_filename in mapping.items()
        }
        generator_name = pathlib.Path(generator_arguments_file).stem
        if generator_name.endswith('__arguments'):
            generator_name = generator_name[:-len('__arguments')]
        caller_pkg = _get_caller_package_name()
        cache_context = {
            'package_name': args['package_name'],
            'generator_name': generator_name,
            'rosidl_parser_version': get_package_version('rosidl_parser'),
            'generator_version': get_package_version(caller_pkg or generator_name),
            'output_mapping': output_mapping,
        }
        if additional_context:
            cache_context.update(additional_context)
        cache_key = compute_cache_key(
            locator.get_absolute_path(),
            *[template_basepath / tf for tf in sorted(mapping.keys())],
            cache_context,
            type_description_info
        )
        if cache_key:
            output_files = restore_files_from_cache(cache_key, generator_name, output_dir)
            if output_files:
                generated_files.extend(output_files)
                continue

        try:
            idl_file = parse_idl_file(locator)
            for template_file, rel_generated_file in output_mapping.items():
                generated_file = os.path.join(output_dir, rel_generated_file)
                generated_files.append(generated_file)
                data = {
                    'package_name': args['package_name'],
                    'interface_path': idl_rel_path,
                    'content': idl_file.content,
                    'type_description_info': type_description_info,
                    'type_source_file': type_source_file,
                }
                if additional_context is not None:
                    data.update(additional_context)
                expand_template(
                    os.path.basename(template_file), data,
                    generated_file, minimum_timestamp=latest_target_timestamp,
                    template_basepath=template_basepath,
                    post_process_callback=post_process_callback)
            if cache_key:
                save_files_to_cache(
                    cache_key, generator_name,
                    list(output_mapping.values()), output_dir)
        except Exception as e:
            print(
                'Error processing idl file: ' +
                str(locator.get_absolute_path()), file=sys.stderr)
            raise e

    return generated_files


template_prefix_path: List[pathlib.Path] = []


def get_template_path(template_name: str) -> pathlib.Path:
    global template_prefix_path
    for basepath in template_prefix_path:
        template_path = basepath / template_name
        if template_path.exists():
            return template_path
    raise RuntimeError(f"Failed to find template '{template_name}'")


interpreter = None


def expand_template(
    template_name: str, data: Dict[str, Any], output_file: str,
    minimum_timestamp: Optional[float] = None,
    template_basepath: Optional[pathlib.Path] = None,
    post_process_callback: Optional[Callable[[str], str]] = None
) -> None:
    # in the legacy API the first argument was the path to the template
    if template_basepath is None:
        template_path = pathlib.Path(template_name)
        template_basepath = template_path.parent
        template_name = template_path.name

    global template_prefix_path
    template_prefix_path.append(template_basepath)
    template_path = get_template_path(template_name)

    global interpreter
    output = StringIO()
    if em_has_configuration:
        config = Configuration(
            defaultRoot=template_path,
            defaultStdout=output,
            deleteOnError=True,
            rawErrors=True,
            useProxy=True)
        interpreter = em.Interpreter(
            config=config,
            dispatcher=False)
    else:
        interpreter = em.Interpreter(
            output=output,
            options={
                em.BUFFERED_OPT: True,
                em.RAW_OPT: True,
            },
        )

    # create copy before manipulating
    data = dict(data)
    _add_helper_functions(data)

    try:
        with template_path.open('r') as h:
            template_content = h.read()
            interpreter.invoke(
                'beforeFile', name=template_name, file=h, locals=data)
        if em_has_configuration:
            interpreter.string(template_content, locals=data)
        else:
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


def _add_helper_functions(data: Dict[str, Any]) -> None:
    data['TEMPLATE'] = _expand_template


def _expand_template(template_name: str, **kwargs: Any) -> None:
    global interpreter
    template_path = get_template_path(template_name)
    _add_helper_functions(kwargs)
    if interpreter is None:
        raise RuntimeError('_expand_template called before expand_template')

    with template_path.open('r') as h:
        interpreter.invoke(
            'beforeInclude', name=str(template_path), file=h, locals=kwargs)
        content = h.read()
    try:
        if em_has_configuration:
            interpreter.string(content, locals=kwargs)
        else:
            interpreter.string(content, template_path, locals=kwargs)
    except Exception as e:  # noqa: F841
        print(f"{e.__class__.__name__} in template '{template_path}': {e}",
              file=sys.stderr)
        raise
    interpreter.invoke('afterInclude')
