#!/usr/bin/env python3
"""Check the tracked snapshot without importing ROS or hardware libraries."""

from __future__ import annotations

import hashlib
from pathlib import Path, PurePosixPath


ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    errors = []
    checked = 0
    for number, line in enumerate((ROOT / 'MANIFEST.sha256').read_text(encoding='utf-8').splitlines(), 1):
        digest, separator, name = line.partition('  ')
        relative = PurePosixPath(name)
        if (not separator or len(digest) != 64
                or any(character not in '0123456789abcdef' for character in digest)
                or not name or relative.is_absolute() or '..' in relative.parts
                or '\\' in name or ':' in name):
            errors.append(f'invalid manifest line {number}')
            continue
        path = ROOT.joinpath(*relative.parts)
        if not path.is_file():
            errors.append(f'missing: {name}')
        elif hashlib.sha256(path.read_bytes()).hexdigest() != digest:
            errors.append(f'hash mismatch: {name}')
        checked += 1
    if not checked:
        errors.append('manifest contains no files')
    for error in errors:
        print(f'ERROR {error}')
    if errors:
        return 1
    print(f'OK {checked} manifest entries')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
