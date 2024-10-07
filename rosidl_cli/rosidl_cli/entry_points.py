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

import importlib.metadata as importlib_metadata
import logging
import sys
from typing import Any, Dict, List, Optional, Tuple, Union


logger = logging.getLogger(__name__)


def get_entry_points(group_name: str, *, specs: Optional[List[str]] = None, strict: bool = False
                     ) -> Dict[str, importlib_metadata.EntryPoint]:
    """
    Get entry points from a specific group.

    :param group_name: the name of the entry point group
    :param specs: an optional collection of entry point names to retrieve
    :param strict: whether to raise or warn on error
    :returns: mapping from entry point names to ``EntryPoint`` instances
    """
    if specs is not None:
        specs_set = set(specs)
    else:
        specs_set = None
    entry_points_impl = importlib_metadata.entry_points()
    # Select does not exist until python 3.10
    if sys.version_info >= (3, 10):
        groups = entry_points_impl.select(group=group_name)
    else:
        groups: Union[Tuple[importlib_metadata.EntryPoint, ...],
                      List[importlib_metadata.EntryPoint]] = entry_points_impl.get(group_name, [])

    entry_points: Dict[str, importlib_metadata.EntryPoint] = {}
    for entry_point in groups:
        name = entry_point.name
        if specs_set and name not in specs_set:
            continue
        if name in entry_points:
            msg = (f"Found duplicate entry point '{name}': "
                   f'got {entry_point} and {entry_points[name]}')
            if strict:
                raise RuntimeError(msg)
            logger.warning(msg)
            continue
        entry_points[name] = entry_point
    if specs_set:
        pending = specs_set - set(entry_points)
        if pending:
            msg = 'Some specs could not be met: '
            msg += ', '.join(map(str, pending))
            if strict:
                raise RuntimeError(msg)
            logger.warning(msg)
    return entry_points


def load_entry_points(group_name: str, *, specs: Optional[List[str]],
                      strict: bool = False,
                      ) -> Dict[str, Any]:
    """
    Load entry points for a specific group.

    See :py:meth:`get_entry_points` for further reference on
    additional keyword arguments.

    :param group_name: the name of the entry point group
    :param strict: whether to raise or warn on error
    :returns: mapping from entry point name to loaded entry point
    """
    loaded_entry_points: Dict[str, Any] = {}
    for name, entry_point in get_entry_points(
        group_name, strict=strict, specs=specs
    ).items():
        try:
            loaded_entry_points[name] = entry_point.load()
        except Exception as e:  # noqa: F841
            msg = f"Failed to load entry point '{name}': {e}"
            if strict:
                raise RuntimeError(msg)
            logger.warning(msg)
    return loaded_entry_points
