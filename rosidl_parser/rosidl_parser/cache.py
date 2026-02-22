#!/usr/bin/env python3
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
import json
import os
import pathlib
import shutil
import tempfile
from typing import Any, List, Optional, TypedDict

import rosidl_parser.definition as _def

# Build class registry and precomputed slots for safe deserialization
_CLASSES = {}
_SLOTS = {}

for _name in dir(_def):
    _obj = getattr(_def, _name)
    if isinstance(_obj, type):
        _CLASSES[_name] = _obj
        # Precompute all slot names across MRO
        _slots = []
        for _klass in _obj.__mro__:
            _klass_slots = getattr(_klass, '__slots__', ())
            if isinstance(_klass_slots, str):
                _slots.append(_klass_slots)
            else:
                _slots.extend(_klass_slots)
        _SLOTS[_obj] = tuple(_slots)


def _encode(obj):
    if obj is None or isinstance(obj, (bool, int, float, str)):
        return obj
    if isinstance(obj, pathlib.Path):
        return {'__path__': str(obj)}
    if isinstance(obj, list):
        return [_encode(item) for item in obj]
    if isinstance(obj, tuple):
        return {'__tuple__': [_encode(item) for item in obj]}
    if isinstance(obj, dict):
        return {k: _encode(v) for k, v in obj.items()}
    slots = _SLOTS.get(type(obj))
    if slots is None:
        raise TypeError(f'Cannot JSON-encode object of type {type(obj).__name__}')
    data = {'__class__': type(obj).__name__}
    for slot in slots:
        if hasattr(obj, slot):
            data[slot] = _encode(getattr(obj, slot))
    return data


def _decode(data):
    if data is None or isinstance(data, (bool, int, float, str)):
        return data
    if isinstance(data, list):
        return [_decode(item) for item in data]
    if isinstance(data, dict):
        if '__path__' in data:
            return pathlib.Path(data['__path__'])
        if '__tuple__' in data:
            return tuple(_decode(item) for item in data['__tuple__'])
        cls_name = data.get('__class__')
        if cls_name:
            if cls_name not in _CLASSES:
                raise ValueError(f'Unknown class: {cls_name}')
            cls = _CLASSES[cls_name]
            obj = cls.__new__(cls)
            for slot in _SLOTS[cls]:
                if slot in data:
                    object.__setattr__(obj, slot, _decode(data[slot]))
            return obj
        return {k: _decode(v) for k, v in data.items()}
    return data


class CacheConfig(TypedDict):
    cache_dir: Optional[str]
    cache_debug: bool
    cache_max_size: int


class _CacheEntry(TypedDict):
    path: pathlib.Path
    size: int
    mtime: float


_cache_config = None


def get_cache_config() -> CacheConfig:
    global _cache_config
    if _cache_config is not None:
        return _cache_config

    config: CacheConfig = {
        'cache_dir': None,
        'cache_debug': False,
        'cache_max_size': 1073741824
    }

    config_file = os.environ.get('ROSIDL_CACHE_CONFIG')
    if config_file and os.path.exists(config_file):
        with open(config_file, 'r') as f:
            file_config = json.load(f)
            if 'cache_dir' in file_config:
                config['cache_dir'] = file_config['cache_dir']
            if 'cache_debug' in file_config:
                config['cache_debug'] = file_config['cache_debug']
            if 'cache_max_size' in file_config:
                config['cache_max_size'] = file_config['cache_max_size']

    # Override with environment variables
    env_cache_dir = os.environ.get('ROSIDL_CACHE_DIR')
    if env_cache_dir is not None:
        config['cache_dir'] = env_cache_dir
    env_cache_debug = os.environ.get('ROSIDL_CACHE_DEBUG')
    if env_cache_debug is not None:
        config['cache_debug'] = env_cache_debug.lower() in ('1', 'true', 'yes')
    env_cache_max_size = os.environ.get('ROSIDL_CACHE_MAX_SIZE')
    if env_cache_max_size is not None:
        config['cache_max_size'] = int(env_cache_max_size)

    _cache_config = config
    return config


def debug_print(msg: str) -> None:
    if get_cache_config()['cache_debug']:
        print(msg)


def get_cache_dir(subdirectory: str) -> Optional[pathlib.Path]:
    cache_dir_str = get_cache_config()['cache_dir']
    if not cache_dir_str:
        return None

    cache_dir = pathlib.Path(cache_dir_str) / subdirectory
    return cache_dir


def cleanup_cache_if_needed(subdirectory: str):
    config = get_cache_config()
    cache_dir = get_cache_dir(subdirectory)
    max_size = config['cache_max_size']

    if not cache_dir or not cache_dir.exists():
        return

    # Collect cache entry directories with sizes and modification times
    entries: List[_CacheEntry] = []
    total_size = 0

    for entry_dir in cache_dir.iterdir():
        if not entry_dir.is_dir():
            continue

        try:
            dir_size = sum(
                f.stat().st_size for f in entry_dir.rglob('*') if f.is_file()
            )
            mtime = max(
                (f.stat().st_mtime for f in entry_dir.rglob('*') if f.is_file()),
                default=0.0
            )
            entries.append({
                'path': entry_dir,
                'size': dir_size,
                'mtime': mtime
            })
            total_size += dir_size
        except (OSError, PermissionError):
            continue

    if total_size <= max_size:
        return

    # Sort by modification time (oldest first)
    entries.sort(key=lambda e: e['mtime'])

    # Delete oldest entries until we're below 80% of max_size
    target_size = int(max_size * 0.8)
    current_size = total_size

    for entry in entries:
        if current_size <= target_size:
            break

        try:
            shutil.rmtree(entry['path'])
            current_size -= entry['size']
            debug_print(f'[rosidl cache] Removed old cache entry: {entry["path"].name}')
        except (OSError, PermissionError) as e:
            debug_print(f'[rosidl cache] Failed to remove cache entry: {e}')


def compute_cache_key(*args) -> Optional[str]:
    if not get_cache_config()['cache_dir']:
        return None

    hasher = hashlib.sha256()

    for arg in args:
        if isinstance(arg, pathlib.Path):
            with open(arg, 'rb') as f:
                hasher.update(f.read())
        elif isinstance(arg, dict):
            hasher.update(json.dumps(arg, sort_keys=True).encode('utf-8'))
        else:
            hasher.update(json.dumps(arg, sort_keys=True, default=str).encode('utf-8'))

    return hasher.hexdigest()


def save_object_to_cache(cache_key: str, cache_subdir: str, obj: Any) -> None:
    cache_dir = get_cache_dir(cache_subdir)
    if not cache_dir:
        return
    cache_dir.mkdir(parents=True, exist_ok=True)
    cache_entry_dir = cache_dir / cache_key
    tmp_dir = None
    try:
        tmp_dir = pathlib.Path(tempfile.mkdtemp(dir=cache_dir))
        cache_file = tmp_dir / 'object.json'
        with open(cache_file, 'w', encoding='utf-8') as f:
            json.dump(_encode(obj), f)
        if cache_entry_dir.exists():
            shutil.rmtree(cache_entry_dir)
        tmp_dir.rename(cache_entry_dir)
    except Exception as e:
        debug_print(f'[rosidl cache] Failed to save to cache: {e}')
        if tmp_dir and tmp_dir.exists():
            shutil.rmtree(tmp_dir, ignore_errors=True)
        return
    cleanup_cache_if_needed(cache_subdir)


def restore_object_from_cache(cache_key: str, cache_subdir: str) -> Optional[Any]:
    cache_dir = get_cache_dir(cache_subdir)
    if not cache_dir:
        return None
    cache_entry_dir = cache_dir / cache_key
    cache_file = cache_entry_dir / 'object.json'
    if not cache_file.exists():
        return None
    try:
        with open(cache_file, 'r', encoding='utf-8') as f:
            return _decode(json.load(f))
    except Exception as e:
        debug_print(f'[rosidl cache] Failed to load from cache: {e}')
        return None


def save_files_to_cache(
    cache_key: str,
    cache_subdir: str,
    files: list,
    base_dir: str
) -> None:
    cache_dir = get_cache_dir(cache_subdir)
    if not cache_dir:
        return

    cache_dir.mkdir(parents=True, exist_ok=True)
    cache_entry_dir = cache_dir / cache_key
    tmp_dir = None
    try:
        tmp_dir = pathlib.Path(tempfile.mkdtemp(dir=cache_dir))
        base_path = pathlib.Path(base_dir)
        for rel_path in files:
            src = base_path / rel_path
            dest = tmp_dir / rel_path
            dest.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(src, dest)
        if cache_entry_dir.exists():
            shutil.rmtree(cache_entry_dir)
        tmp_dir.rename(cache_entry_dir)
    except Exception as e:
        debug_print(f'[rosidl cache] Failed to save files to cache: {e}')
        if tmp_dir and tmp_dir.exists():
            shutil.rmtree(tmp_dir, ignore_errors=True)
        return

    cleanup_cache_if_needed(cache_subdir)


def restore_files_from_cache(
    cache_key: str,
    cache_subdir: str,
    output_dir: str
) -> Optional[list]:
    cache_dir = get_cache_dir(cache_subdir)
    if not cache_dir:
        return None

    cache_entry_dir = cache_dir / cache_key
    if not cache_entry_dir.exists():
        return None

    output_path = pathlib.Path(output_dir)
    restored_files = []

    for src in cache_entry_dir.rglob('*'):
        if not src.is_file():
            continue
        rel_path = src.relative_to(cache_entry_dir)
        dest = output_path / rel_path
        dest.parent.mkdir(parents=True, exist_ok=True)
        try:
            shutil.copy2(src, dest)
            restored_files.append(str(dest))
        except Exception as e:
            debug_print(f'[rosidl cache] Failed to restore file {rel_path}: {e}')
            return None

    return restored_files
