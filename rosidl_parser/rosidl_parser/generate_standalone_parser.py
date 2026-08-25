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

import hashlib
import io
import pathlib
import sys


def generate_standalone_parser() -> None:
    """Regenerate _standalone_parser.py from grammar.lark with SHA-256 metadata."""
    try:
        from lark import Lark
        import lark.tools.standalone
    except ImportError:
        print(
            "Error: 'lark' package is required to regenerate the standalone parser.\n"
            'Please install lark via: pip install lark',
            file=sys.stderr,
        )
        sys.exit(1)

    package_dir = pathlib.Path(__file__).parent
    grammar_path = package_dir / 'grammar.lark'
    output_path = package_dir / '_standalone_parser.py'

    grammar_bytes = grammar_path.read_bytes()
    grammar_sha256 = hashlib.sha256(grammar_bytes).hexdigest()

    with open(grammar_path, 'r', encoding='utf-8') as f:
        lark_inst = Lark(
            f,
            parser='lalr',
            start=['specification'],
            lexer='contextual',
            postlex=None,
            priority='auto',
            regex=False,
        )

    out = io.StringIO()
    lark.tools.standalone.gen_standalone(lark_inst, out=out)
    code = out.getvalue()

    header = (
        '# type: ignore\n'
        '# flake8: noqa\n'
        '# SHA-256 of grammar.lark used to generate this file:\n'
        f'_GRAMMAR_SHA256 = {grammar_sha256!r}\n\n'
    )
    # Suppress Python 3.14 sre_parse / sre_constants deprecation warnings
    sre_patch = (
        'import warnings\n'
        'with warnings.catch_warnings():\n'
        '    warnings.simplefilter("ignore", DeprecationWarning)\n'
        '    try:\n'
        '        from re import _parser as sre_parse\n'
        '        from re import _constants as sre_constants\n'
        '    except ImportError:\n'
        '        import sre_parse\n'
        '        import sre_constants\n'
    )
    code = code.replace(
        'import sre_parse\nimport sre_constants\n',
        sre_patch
    )
    final_code = header + code

    output_path.write_text(final_code, encoding='utf-8')
    print(
        f'Successfully generated {output_path} (Grammar SHA-256: {grammar_sha256})'
    )


if __name__ == '__main__':
    generate_standalone_parser()
