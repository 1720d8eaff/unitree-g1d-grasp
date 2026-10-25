#!/usr/bin/env python3
"""Regenerate the project SHA-256 manifest deterministically."""

from __future__ import annotations

import hashlib
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "MANIFEST.sha256"
EXCLUDED_PARTS = {"runs", "__pycache__", ".git"}


def digest(path: Path) -> str:
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(chunk)
    return value.hexdigest()


files = []
for path in ROOT.rglob("*"):
    if not path.is_file() or path == OUTPUT:
        continue
    relative = path.relative_to(ROOT)
    if any(part in EXCLUDED_PARTS for part in relative.parts):
        continue
    files.append(relative)

lines = [f"{digest(ROOT / relative)}  {relative.as_posix()}" for relative in sorted(files)]
OUTPUT.write_text("\n".join(lines) + "\n", encoding="utf-8")
print(f"WROTE {OUTPUT} ({len(lines)} files)")
