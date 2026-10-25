#!/usr/bin/env python3
"""Build a board-side transfer archive without deploying it."""

from __future__ import annotations

import hashlib
from pathlib import Path
import tarfile


ROOT = Path(__file__).resolve().parents[1]
VERSION = (ROOT / "VERSION").read_text(encoding="utf-8").strip()
OUTPUT_DIR = ROOT / "dist"
OUTPUT = OUTPUT_DIR / f"G1D_Grasp_Project_board_{VERSION}.tar.gz"
PREFIX = Path("G1D_Grasp_Project")

INCLUDE = [
    Path("VERSION"),
    Path("g1d-board-start"),
    Path("g1d-hand-board-start"),
    Path("config/processes_board.json"),
    Path("config/processes_hand_board.json"),
    Path("config/endpoint_feedback_v2.json"),
    Path("config/onboard_source_provenance.json"),
    Path("config/safety.json"),
    Path("tools/process_supervisor.py"),
    Path("tools/collect_onboard_scripts.sh"),
    Path("onboard/startup"),
    Path("onboard/state"),
    Path("onboard/state_snapshot_20260827"),
    Path("onboard/height"),
    Path("onboard/trajectory_bridge"),
    Path("onboard/vision"),
    Path("hand/onboard"),
    Path("docs/BOARD_DEPLOYMENT.md"),
    Path("docs/START_ENTRY_DEPENDENCIES.md"),
    Path("docs/SCRIPT_INVENTORY.md"),
    Path("docs/SAFETY.md"),
]


OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
with tarfile.open(OUTPUT, "w:gz", format=tarfile.PAX_FORMAT) as archive:
    for relative in INCLUDE:
        source = ROOT / relative
        if not source.exists():
            raise SystemExit(f"missing board bundle input: {relative}")
        archive.add(source, arcname=str(PREFIX / relative), recursive=True)

print(f"WROTE {OUTPUT}")
digest = hashlib.sha256(OUTPUT.read_bytes()).hexdigest()
checksum = OUTPUT.with_suffix(OUTPUT.suffix + ".sha256")
checksum.write_text(f"{digest}  {OUTPUT.name}\n", encoding="utf-8")
print(f"SHA256 {digest}")
print(f"WROTE {checksum}")
