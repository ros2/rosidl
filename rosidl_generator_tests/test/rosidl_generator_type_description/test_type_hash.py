# Copyright 2023 Open Source Robotics Foundation, Inc.
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
import os
from pathlib import Path

from ament_index_python import get_package_share_directory
import jsonschema


def test_type_hash():
    """Test all rosidl_generator_type_description output files against defined schemas."""
    schema_path = (
        Path(get_package_share_directory('rosidl_generator_type_description')) / 'resource' /
        'HashedTypeDescription.schema.json')
    with schema_path.open('r') as schema_file:
        schema = json.load(schema_file)

    generated_files_dir = Path(os.environ['GENERATED_TEST_FILE_DIR'])
    validated_files = 0
    for namespace in generated_files_dir.iterdir():
        for p in namespace.iterdir():
            assert p.is_file()
            assert p.suffix == '.json'
            with p.open('r') as f:
                instance = json.load(f)
            jsonschema.validate(instance=instance, schema=schema)
            validated_files += 1
    assert validated_files, 'Needed to validate at least one JSON output.'
