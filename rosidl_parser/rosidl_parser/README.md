# `rosidl_parser` IDL Grammar & Standalone Parser

This directory contains the grammar definition, the pre-generated standalone parser, and the regeneration tooling used to parse ROS 2 `.idl` interface files.

---

## Files Overview

- **`grammar.lark`**: The canonical, human-readable EBNF grammar defining the subset of OMG IDL 4.2 supported by ROS 2.
- **`_standalone_parser.py`**: A self-contained, pre-compiled LALR(1) parser generated ahead-of-time from `grammar.lark` using `lark.tools.standalone`.
- **`generate_standalone_parser.py`**: Helper script to regenerate `_standalone_parser.py` from `grammar.lark` and embed its SHA-256 checksum for automated sync verification.
- **`parser.py`**: The main interface for `rosidl_parser`. It imports `_standalone_parser` by default (with a fallback to dynamic compilation via `lark` if `_standalone_parser` is absent) and converts parse trees into `rosidl_parser.definition` data structures.

---

## Why Both `grammar.lark` and `_standalone_parser.py` Exist

1. **Maintainability (`grammar.lark`)**: `grammar.lark` provides a readable, declarative reference for the IDL specification. When adding or modifying IDL grammar rules, developers edit `grammar.lark` directly rather than reverse-engineering the generated parser tables.
2. **Build Performance (`_standalone_parser.py`)**: Dynamic Lark grammar compilation at runtime adds ~60–80 ms of initialization overhead to the first IDL file parsed in each process. The pre-compiled LALR(1) tables eliminate this overhead, reducing cold parse time to ~4 ms per file (~15x speedup).
3. **Zero Runtime Dependencies**: `_standalone_parser.py` is pure Python and does not require `python3-lark-parser` to be installed at runtime or during package builds.

---

## Regenerating the Standalone Parser

Whenever `grammar.lark` is modified, `_standalone_parser.py` must be regenerated.

### Prerequisites

Regeneration requires the `lark` Python package:

```bash
pip install lark
```

### Regeneration Command

Run the regeneration script from the repository root:

```bash
python3 -m rosidl_parser.generate_standalone_parser
```

Or invoke `lark.tools.standalone` directly:

```bash
python3 -m lark.tools.standalone -s specification src/ros2/rosidl/rosidl_parser/rosidl_parser/grammar.lark -o src/ros2/rosidl/rosidl_parser/rosidl_parser/_standalone_parser.py
```

`generate_standalone_parser.py` automatically computes the SHA-256 hash of `grammar.lark` and writes it as `_GRAMMAR_SHA256` at the top of `_standalone_parser.py`.

---

## Automated Sync & Parity Testing

To ensure `grammar.lark` and `_standalone_parser.py` never drift out of sync:

1. **`test_standalone_parser_in_sync`** (in `test/test_parser.py`):
   - Computes the SHA-256 checksum of `grammar.lark` at test time and verifies that it matches `_GRAMMAR_SHA256` in `_standalone_parser.py`.
   - Runs in < 1 ms without requiring Lark, catching any uncommitted grammar changes in standard CI and local unit test runs.

2. **`test_standalone_parser_dynamic_parity`** (in `test/test_parser.py`):
   - When `lark` is available in the test environment, dynamically compiles `grammar.lark` and asserts that dynamic Lark and `_standalone_parser` produce identical ASTs across representative IDL definitions.
