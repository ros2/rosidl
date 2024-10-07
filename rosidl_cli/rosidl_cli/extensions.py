# Copyright 2021 Open Source Robotics Foundation, Inc.
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

import logging
import re
from typing import Any, Dict, Final, List, Optional, Tuple, TYPE_CHECKING, Union

from rosidl_cli.entry_points import load_entry_points

import yaml  # type: ignore[import]

if TYPE_CHECKING:
    from typing import TypedDict
    from typing_extensions import NotRequired

    class LoadExtensionsArg(TypedDict):
        specs: NotRequired[Optional[List[str]]]
        strict: NotRequired[bool]


logger = logging.getLogger(__name__)


class Extension:
    """A generic extension point."""

    def __init__(self, name: str) -> None:
        self.__name = name

    @property
    def name(self) -> str:
        return self.__name


SPECS_PATTERN: Final = re.compile(r'^(\w+)(?:\[(.+)\])?$')


def parse_extension_specification(spec: str) -> Tuple[Union[str, Any], Union[Dict[Any, Any], Any]]:
    """
    Parse extension specification.

    :param str spec: specification string in
      'name[key0: value0, ...]' or 'name' format.
      Key-value pairs are parsed as YAML dictionaries.
    :returns: a tuple of specification name and
      keyword arguments, if any, as a dict.
    """
    match = SPECS_PATTERN.match(spec)
    if not match:
        raise ValueError(f'{spec} is not a valid spec')
    name = match.group(1)
    kwargs = match.group(2)
    if kwargs is not None:
        try:
            kwargs = yaml.safe_load('{' + kwargs + '}')
        except Exception as e:
            raise ValueError(
                f'{spec} is not a valid spec'
            ) from e
    else:
        kwargs = {}
    return name, kwargs


def load_extensions(group_name: str, *, specs: Optional[List[str]] = None,
                    strict: bool = False) -> List[Extension]:
    """
    Load extensions for a specific group.

    :param group_name: the name of the extension group
    :param specs: an optional collection of extension specs
      (see :py:meth:`parse_extension_specification` for spec format)
    :param strict: whether to raise or warn on error
    :returns: a list of :py:class:`Extension` instances
    """
    extensions: List[Extension] = []

    if specs is not None:
        kwargs = dict(map(
            parse_extension_specification, specs))
        specs = list(kwargs.keys())
    else:
        kwargs = {}

    for name, factory in load_entry_points(
        group_name, specs=specs, strict=strict
    ).items():
        try:
            extensions.append(factory(name, **kwargs.get(name, {})))
        except Exception as e:
            msg = f"Failed to instantiate extension '{name}' "
            which = kwargs.get(name, 'default')
            msg += f"with '{which}' arguments: {e}"
            if strict:
                raise RuntimeError(msg)
            logger.warning(msg)
    return extensions
