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

try:
    import importlib.metadata as importlib_metadata
except ModuleNotFoundError:
    import importlib_metadata

from packaging.specifiers import BaseSpecifier
from packaging.specifiers import InvalidSpecifier
from packaging.specifiers import SpecifierSet
from packaging.version import Version


logger = logging.getLogger(__name__)


def normalize_entry_point_specs(specs):
    """
    Normalize a collection of entry point specifications.

    A normalized collection of entry point specifications is mapping from entry
    point names to entry point version :py:class:`BaseSpecifier` instances.
    A denormalized collection may also be an iterable of entry point specification
    strings i.e. an entry point name optionally followed by a PEP440 version specifier.

    :param specs: a (de)normalized collection of entry point specifications
    :type: iterable or dict
    :returns: mapping of entry point name to :py:class:`BaseSpecifier` instances
    :rtype: dict
    """
    if not isinstance(specs, dict):
        pattern = re.compile(r'^([\w.-]+)(.*)')

        normalized_specs = {}
        for spec in specs:
            match = pattern.match(spec)
            if match is None:
                raise ValueError(f'Invalid name in spec: {spec}')
            name = match[1]
            try:
                specifier = SpecifierSet(match[2])
            except InvalidSpecifier:
                raise ValueError(f'Invalid specifier in spec: {spec}')
            if name in normalized_specs:
                specifier &= normalized_specs[name]
            normalized_specs[name] = specifier
        specs = normalized_specs
    for specifier in specs.values():
        if not isinstance(specifier, BaseSpecifier):
            raise TypeError(f"'{specifier}' is not an specifier")
    return specs


def get_entry_points(group_name, *, specs=None, strict=False):
    """
    Get entry points for a specific group.

    See :py:function:`normalize_entry_point_specs()` for further reference
    on entry point specifications.

    :param str group_name: the name of the entry point group
    :param specs: an optional collection of entry point specifications,
    :type: iterable or dict
    :param bool strict: whether to raise or warn on error
    :returns: mapping from entry point names to ``EntryPoint`` instances
    :rtype: dict
    """
    if specs is not None:
        specs = normalize_entry_point_specs(specs)

    entry_points = {}
    entry_points_per_group = importlib_metadata.entry_points()
    for entry_point in entry_points_per_group.get(group_name, []):
        name = entry_point.name
        if name in entry_points:
            msg = (f"Found duplicate entry point '{name}': "
                   'got {entry_point} and {entry_points[name]}')
            if strict:
                raise RuntimeError(msg)
            logger.warning(msg)
            continue
        if specs:
            if name not in specs:
                continue
            version = Version(entry_point.distro.version)
            if version not in specs[name]:
                msg = (f"Spec '{name}{specs[name]}'"
                       f' cannot be met: found {version}')
                if strict:
                    raise RuntimeError(msg)
                logger.warning(msg)
                continue
        entry_points[name] = entry_point
    if specs:
        pending = set(specs) - set(entry_points)
        if pending:
            msg = 'Some specs could not be met: ' + ', '.join([
                f'{name}{specs[name]}' for name in pending
            ])
            if strict:
                raise RuntimeError(msg)
            logger.warning(msg)
    return entry_points


def load_entry_points(group_name, *, strict=False, **kwargs):
    """
    Load entry points for a specific group.

    See :py:function:`get_entry_points` for further reference on
    additional keyword arguments.

    :param str group_name: the name of the entry point group
    :param bool strict: whether to raise or warn on error
    :returns: mapping from entry point name to loaded entry point
    :rtype: dict
    """
    loaded_entry_points = {}
    for name, entry_point in get_entry_points(
        group_name, strict=strict, **kwargs
    ).items():
        try:
            loaded_entry_points[name] = entry_point.load()
        except Exception as e:  # noqa: F841
            msg = f"Failed to load entry point '{name}': {e}"
            if strict:
                raise RuntimeError(msg)
            logger.warning(msg)
    return loaded_entry_points
