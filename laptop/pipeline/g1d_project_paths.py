"""Central paths for the consolidated G1-D project.

The original scripts lived in dated output folders and relied on files beside
the source.  This module keeps immutable inputs under ``data`` and sends every
new planning result to an explicit run directory.
"""

from __future__ import annotations

import os
from pathlib import Path


PIPELINE_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = PIPELINE_DIR.parents[1]

DATA_DIR = PROJECT_ROOT / "data"
RAW_DATA_DIR = DATA_DIR / "raw"
TAUGHT_DATA_DIR = DATA_DIR / "taught"
REFERENCE_DATA_DIR = DATA_DIR / "references"

DEFAULT_RUN_DIR = PROJECT_ROOT / "runs" / "manual"
RUN_DIR = Path(os.environ.get("G1D_RUN_DIR", str(DEFAULT_RUN_DIR))).resolve()


def ensure_run_dir() -> Path:
    RUN_DIR.mkdir(parents=True, exist_ok=True)
    return RUN_DIR

