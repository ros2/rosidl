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

import argparse
import pathlib
import sys
from typing import Optional
import warnings


def generate_grammar_cache(output_file: Optional[pathlib.Path] = None) -> None:
    """Generate a pre-compiled Lark parser cache (grammar.lark.bin) from grammar.lark."""
    try:
        with warnings.catch_warnings():
            warnings.simplefilter('ignore', DeprecationWarning)
            from lark import Lark
    except ImportError:
        print(
            "Error: 'lark' package is required to generate the grammar cache.\n"
            'Please install lark via: pip install lark',
            file=sys.stderr,
        )
        sys.exit(1)

    package_dir = pathlib.Path(__file__).parent
    grammar_path = package_dir / 'grammar.lark'
    if output_file is not None:
        output_path = output_file
    else:
        output_path = package_dir / 'grammar.lark.bin'

    with open(grammar_path, 'r', encoding='utf-8') as f:
        lark_inst = Lark(
            f,
            parser='lalr',
            start=['specification'],
            lexer='contextual',
            postlex=None,
            priority='auto',
            regex=False,
            maybe_placeholders=False,
        )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, 'wb') as out:
        lark_inst.save(out)
    print(f'Successfully generated {output_path}')


def main() -> None:
    parser = argparse.ArgumentParser(
        description='Generate pre-compiled Lark parser cache from grammar.lark.'
    )
    parser.add_argument(
        '-o',
        '--output',
        type=pathlib.Path,
        default=None,
        help='Output file path (defaults to grammar.lark.bin in the package directory).',
    )
    args = parser.parse_args()
    generate_grammar_cache(args.output)


if __name__ == '__main__':
    main()
