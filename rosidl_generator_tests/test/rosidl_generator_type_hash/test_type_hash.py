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
    """Test all rosidl_generator_type_hash output files against defined schemas."""
    schema_dir = Path(get_package_share_directory('rosidl_generator_type_hash')) / 'resource'
    resolver = jsonschema.validators.RefResolver(
        base_uri=f'{schema_dir.as_uri()}/',
        referrer=True,
    )

    generated_files_dir = Path(os.environ['GENERATED_TEST_FILE_DIR'])
    validated_sha256 = 0
    validated_json_in = 0
    validated_json = 0
    for namespace in generated_files_dir.iterdir():
        for p in namespace.iterdir():
            assert p.is_file()
            assert p.suffix == '.json'
            with p.open('r') as f:
                instance = json.load(f)
            subsuffix = p.with_suffix('').suffix
            if subsuffix == '.sha256':
                jsonschema.validate(
                    instance=instance,
                    schema={'$ref': 'TypeVersionHash.schema.json'},
                    resolver=resolver,
                )
                validated_sha256 += 1
            elif subsuffix == '.in':
                jsonschema.validate(
                    instance=instance,
                    schema={'$ref': 'TypeDescriptionIn.schema.json'},
                    resolver=resolver,
                )
                validated_json_in += 1
            elif subsuffix == '':
                jsonschema.validate(
                    instance=instance,
                    schema={'$ref': 'TypeDescription.schema.json'},
                    resolver=resolver,
                )
                validated_json += 1
            else:
                assert False, 'Unknown file type to validate'
    assert validated_sha256, 'Needed to validate at least one of each type of file.'
    assert validated_json_in, 'Needed to validate at least one of each type of file.'
    assert validated_json, 'Needed to validate at least one of each type of file.'
