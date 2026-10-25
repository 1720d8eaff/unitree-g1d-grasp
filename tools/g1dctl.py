#!/usr/bin/env python3
"""Safe project manager for the consolidated G1-D grasp stack.

Version 1 deliberately exposes inventory checks, terminal guidance, geometry
activation, and Plan-only.  It does not expose real execution, ARM/DISARM, or
RH56H1 commands.
"""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "config"
ACTIVE_GEOMETRY = CONFIG / "active_geometry.json"
PIPELINE = ROOT / "laptop" / "pipeline"
TABLE_NODE = ROOT / "laptop" / "moveit" / "add_g1_table_collision.py"
MAIN_PLANNER = PIPELINE / "execute_full_taught_to_anygrasp_continuous.py"
CALIBRATION_RECORDER = PIPELINE / "record_left_planner_calibration_teach.py"
TAUGHT_CDR = ROOT / "data" / "taught" / "g1_left_taught_motion.cdr"
TAUGHT_JSON = ROOT / "data" / "taught" / "g1_left_taught_motion.json"
V2_BACKEND = (
    ROOT
    / "onboard"
    / "trajectory_bridge"
    / "g1_d_trajectory_bridge"
    / "trajectory_backend.py"
)
ONBOARD_PROVENANCE = CONFIG / "onboard_source_provenance.json"


def load_json(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def required_files():
    return [
        ACTIVE_GEOMETRY,
        CONFIG / "safety.json",
        CONFIG / "endpoint_feedback_v2.json",
        CONFIG / "external_paths.json",
        CONFIG / "processes_board.json",
        CONFIG / "processes_laptop.json",
        CONFIG / "processes_hand_board.json",
        CONFIG / "processes_hand_laptop.json",
        ONBOARD_PROVENANCE,
        ROOT / "g1d-board-start",
        ROOT / "g1d-laptop-start",
        ROOT / "g1d-hand-board-start",
        ROOT / "g1d-hand-laptop-start",
        ROOT / "laptop" / "startup" / "start_g1_control.sh",
        ROOT / "laptop" / "startup" / "start_g1_moveit.sh",
        ROOT / "tools" / "process_supervisor.py",
        MAIN_PLANNER,
        PIPELINE / "execute_left_taught_then_anygrasp.py",
        PIPELINE / "execute_left_taught_with_auto_start.py",
        PIPELINE / "anygrasp_segment.py",
        PIPELINE / "left_taught_segment.py",
        PIPELINE / "endpoint_diagnostics.py",
        CALIBRATION_RECORDER,
        TABLE_NODE,
        TAUGHT_CDR,
        TAUGHT_JSON,
        ROOT / "data" / "references" / "left_grasp_verified_orientation_reference.json",
        V2_BACKEND,
        ROOT / "hand" / "onboard" / "rh56h1_udp_driver" / "rh56h1_udp_driver" / "driver_node.py",
        ROOT / "hand" / "laptop" / "rh56h1_gateway_v2.py",
        ROOT / "hand" / "onboard" / "rh56h1_virtual_gripper_mapper_v5.py",
        ROOT / "hand" / "onboard" / "rh56h1_virtual_gripper_controller_v4.py",
        ROOT / "hand" / "onboard" / "service_interfaces" / "package.xml",
        ROOT / "hand" / "onboard" / "service_interfaces" / "CMakeLists.txt",
        ROOT / "hand" / "onboard" / "service_interfaces" / "msg" / "SetAngle1.msg",
        ROOT / "onboard" / "startup" / "run_rh56h1_service.sh",
        ROOT / "onboard" / "height" / "g1d_height_15cm.cpp",
        ROOT / "onboard" / "startup" / "start_g1_control.sh",
        ROOT / "onboard" / "startup" / "start_g1_state.sh",
        ROOT / "onboard" / "startup" / "g1_trajectory_bridge_env.sh",
        ROOT / "onboard" / "state" / "unitree_reader.py",
        ROOT / "onboard" / "state" / "g1_d_state_bridge" / "g1_d_state_bridge" / "state_bridge_node.py",
        ROOT / "laptop" / "ros_packages" / "g1_d_moveit_config" / "launch" / "real_robot.launch.py",
        ROOT / "laptop" / "ros_packages" / "g1_d_description" / "g1_d_ros.urdf",
    ]


def run_doctor(require_verified_geometry: bool = False) -> bool:
    print(f"PROJECT_ROOT={ROOT}")
    missing = [path for path in required_files() if not path.is_file()]
    if missing:
        for path in missing:
            print(f"ERROR missing: {path.relative_to(ROOT)}")
        return False

    ok = True
    feedback = load_json(CONFIG / "endpoint_feedback_v2.json")
    actual_backend_hash = sha256(V2_BACKEND)
    expected_backend_hash = feedback["expected_source_sha256"]
    if actual_backend_hash != expected_backend_hash:
        print("ERROR onboard V2 backend hash mismatch")
        ok = False
    else:
        print(f"OK V2 backend sha256={actual_backend_hash}")

    taught = load_json(TAUGHT_JSON)
    actual_taught_hash = sha256(TAUGHT_CDR)
    expected_taught_hash = taught.get("trajectory_sha256")
    if actual_taught_hash != expected_taught_hash:
        print("ERROR taught trajectory hash mismatch")
        ok = False
    else:
        print(
            "OK taught trajectory "
            f"points={taught.get('point_count')} sha256={actual_taught_hash}"
        )

    provenance = load_json(ONBOARD_PROVENANCE)
    board_hashes_ok = True
    archive = provenance["archive"]
    pinned_paths = {archive["path"]: archive["sha256"]}
    pinned_paths.update(provenance["promoted_sources"])
    for relative, expected_hash in pinned_paths.items():
        path = ROOT / relative
        if not path.is_file() or sha256(path) != expected_hash:
            print(f"ERROR live-board source hash mismatch: {relative}")
            board_hashes_ok = False
    if board_hashes_ok:
        print(
            "OK live-board script set "
            f"files={len(provenance['promoted_sources'])} "
            f"export={provenance['export_timestamp']}"
        )
    else:
        ok = False

    geometry = load_json(ACTIVE_GEOMETRY)
    center = geometry.get("work_table", {}).get("center_xyz_m", [])
    geometry_ready = (
        geometry.get("verified") is True
        and len(center) == 3
        and all(value is not None for value in center)
    )
    if geometry_ready:
        print(
            "OK active geometry "
            f"profile={geometry['profile_id']} center={center}"
        )
    else:
        label = "ERROR" if require_verified_geometry else "BLOCKED"
        print(
            f"{label} active geometry is not verified: "
            f"{geometry.get('profile_id')}"
        )
        print("      measure table top Z relative to torso_link before planning")
        if require_verified_geometry:
            ok = False

    print("OK real execution remains disabled in g1dctl v1")
    return ok


def command_doctor(_args):
    raise SystemExit(0 if run_doctor(False) else 1)


def command_terminals(_args):
    project = str(ROOT)
    print("[BOARD — one foreground supervisor, keep DISARMED]")
    print("cd /home/unitree/G1D_Grasp_Project")
    print("./g1d-board-start")
    print("\n[LAPTOP — one foreground supervisor]")
    print(f"cd {project}")
    print("./g1d-laptop-start")
    print("\n[RH56H1 BOARD — separate foreground hand supervisor]")
    print("cd /home/unitree/G1D_Grasp_Project")
    print("./g1d-hand-board-start")
    print("\n[RH56H1 LAPTOP — separate foreground hand supervisor]")
    print(f"cd {project}")
    print("./g1d-hand-laptop-start")
    print("\n[PLAN-ONLY — after both supervisors are healthy]")
    print(f"cd {project}")
    print("./g1dctl plan")


def command_geometry_set(args):
    if not args.confirm_measured:
        raise SystemExit(
            "Refusing update: add --confirm-measured only after a physical measurement"
        )
    if not -2.0 < args.table_top_z_m < 1.0:
        raise SystemExit("table top Z is outside the sanity range (-2.0, 1.0) m")
    geometry = load_json(ACTIVE_GEOMETRY)
    table = geometry["work_table"]
    size_z = float(table["size_xyz_m"][2])
    center = list(table["center_xyz_m"])
    center[2] = args.table_top_z_m - size_z / 2.0
    table["center_xyz_m"] = center
    table["top_z_m"] = args.table_top_z_m
    geometry["verified"] = True
    geometry["profile_id"] = "column_15cm_measured"
    geometry["verified_utc"] = dt.datetime.now(dt.timezone.utc).isoformat()
    geometry["verification_note"] = args.note
    temporary = ACTIVE_GEOMETRY.with_suffix(".json.tmp")
    temporary.write_text(
        json.dumps(geometry, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary, ACTIVE_GEOMETRY)
    print(f"UPDATED {ACTIVE_GEOMETRY}")
    print(f"table_top_z_m={args.table_top_z_m:.6f}")
    print(f"table_center_z_m={center[2]:.6f}")


def command_plan(args):
    if not run_doctor(require_verified_geometry=True):
        raise SystemExit(2)
    stamp = dt.datetime.now().strftime("%Y%m%d-%H%M%S")
    run_dir = ROOT / "runs" / stamp
    run_dir.mkdir(parents=True, exist_ok=False)
    environment = os.environ.copy()
    if args.previous_preview:
        environment["G1D_PREVIOUS_PLANNER_PREVIEW"] = "1"
        print(
            "WARNING PREVIOUS_PLANNER_PREVIEW: restoring the previous "
            "planner policy for RViz comparison only; no execution is exposed"
        )
    environment["G1D_RUN_DIR"] = str(run_dir)
    pipeline_path = str(PIPELINE)
    existing = environment.get("PYTHONPATH")
    environment["PYTHONPATH"] = (
        pipeline_path if not existing else pipeline_path + os.pathsep + existing
    )
    print(f"RUN_DIR={run_dir}")
    subprocess.run(
        [sys.executable, str(TABLE_NODE), "--profile", str(ACTIVE_GEOMETRY)],
        check=True,
        env=environment,
    )
    subprocess.run(
        [sys.executable, str(MAIN_PLANNER)],
        check=True,
        env=environment,
    )
    print("PLAN_ONLY_COMPLETE")
    print(f"Artifacts: {run_dir}")


def command_teach_calibrate(args):
    if not run_doctor(require_verified_geometry=False):
        raise SystemExit(2)
    stamp = dt.datetime.now().strftime("%Y%m%d-%H%M%S")
    run_dir = ROOT / "runs" / f"teach-{stamp}"
    run_dir.mkdir(parents=True, exist_ok=False)
    environment = os.environ.copy()
    environment["G1D_RUN_DIR"] = str(run_dir)
    pipeline_path = str(PIPELINE)
    existing = environment.get("PYTHONPATH")
    environment["PYTHONPATH"] = (
        pipeline_path if not existing else pipeline_path + os.pathsep + existing
    )
    print(f"TEACH_RUN_DIR={run_dir}")
    subprocess.run(
        [
            sys.executable,
            str(CALIBRATION_RECORDER),
            "--sample-hz",
            str(args.sample_hz),
            "--max-duration-sec",
            str(args.max_duration_sec),
        ],
        check=True,
        env=environment,
    )


def command_execute(_args):
    raise SystemExit(
        "Real execution is intentionally unavailable in g1dctl v1. "
        "Complete geometry verification and consolidated Plan-only validation first."
    )


def build_parser():
    parser = argparse.ArgumentParser(prog="g1dctl")
    sub = parser.add_subparsers(dest="command", required=True)

    doctor = sub.add_parser("doctor", help="offline integrity and safety audit")
    doctor.set_defaults(func=command_doctor)

    terminals = sub.add_parser("terminals", help="show current startup commands")
    terminals.set_defaults(func=command_terminals)

    geometry = sub.add_parser(
        "geometry-set", help="activate the measured 15 cm table geometry"
    )
    geometry.add_argument("--table-top-z-m", type=float, required=True)
    geometry.add_argument("--note", required=True)
    geometry.add_argument("--confirm-measured", action="store_true")
    geometry.set_defaults(func=command_geometry_set)

    plan = sub.add_parser("plan", help="add verified table and run Plan-only")
    plan.add_argument(
        "--previous-preview",
        action="store_true",
        help="preview the previous working planner policy; still Plan-only",
    )
    plan.set_defaults(func=command_plan)

    teach = sub.add_parser(
        "teach-calibrate",
        help="read-only full-motion teach recorder for second-stage calibration",
    )
    teach.add_argument("--sample-hz", type=float, default=50.0)
    teach.add_argument("--max-duration-sec", type=float, default=120.0)
    teach.set_defaults(func=command_teach_calibrate)

    execute = sub.add_parser("execute", help="not enabled in project v1")
    execute.set_defaults(func=command_execute)
    return parser


def main():
    args = build_parser().parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
