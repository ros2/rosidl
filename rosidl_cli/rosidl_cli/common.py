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


def get_first_line_doc(any_type):
    if not any_type.__doc__:
        return ''
    lines = any_type.__doc__.splitlines()
    if not lines:
        return ''
    if lines[0]:
        line = lines[0]
    elif len(lines) > 1:
        line = lines[1]
    return line.strip().rstrip('.')
