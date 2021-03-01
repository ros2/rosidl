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

from rosidl_cli.entry_points import load_entry_points


logger = logging.getLogger(__name__)


class Extension:
    """A generic extension point."""

    def __init__(self, name):
        self.__name = name

    @property
    def name(self):
        return self.__name


def load_extensions(group_name, *, strict=False, **kwargs):
    """
    Load extensions for a specific group.

    See :py:function:`load_entry_points` for further reference on
    additional keyword arguments.

    :param str group_name: the name of the extension group
    :param bool strict: whether to raise or warn on error
    :returns: a list of :py:class:`Extension` instances
    :rtype: list
    """
    extensions = []
    for name, factory in load_entry_points(
        group_name, strict=strict, **kwargs
    ).items():
        try:
            extensions.append(factory(name))
        except Exception as e:
            msg = f"Failed to instantiate extension '{name}': {e}"
            if strict:
                raise RuntimeError(msg)
            logger.warning(msg)
    return extensions
