#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Observe the current arm against the saved full-trajectory endpoint.

This utility is read-only: it subscribes to /joint_states and calls
/compute_fk.  It has no publisher, action client, or ARM/DISARM client.
"""

import json
from pathlib import Path

import rclpy
from moveit_msgs.msg import RobotTrajectory
from rclpy.serialization import deserialize_message

import execute_left_taught_then_anygrasp as pipeline
from endpoint_diagnostics import EndpointDiagnostics, file_sha256
from g1d_project_paths import RUN_DIR, ensure_run_dir


HERE = Path(__file__).resolve().parent
ensure_run_dir()
TRAJECTORY_FILE = RUN_DIR / "g1_left_full_taught_to_anygrasp.cdr"
METADATA_FILE = RUN_DIR / "g1_left_full_taught_to_anygrasp.json"
REPORT_FILE = RUN_DIR / "g1_left_endpoint_diagnostics_observation.json"


def main():
    serialized = TRAJECTORY_FILE.read_bytes()
    metadata = json.loads(METADATA_FILE.read_text(encoding="utf-8"))
    digest = file_sha256(TRAJECTORY_FILE)
    if digest != metadata.get("trajectory_sha256"):
        raise RuntimeError("完整轨迹CDR与JSON的SHA-256不一致")
    trajectory = deserialize_message(serialized, RobotTrajectory)
    joint_trajectory = trajectory.joint_trajectory
    if list(joint_trajectory.joint_names) != pipeline.base.LEFT_ARM_JOINTS:
        raise RuntimeError("完整轨迹左臂关节顺序不匹配")
    if not joint_trajectory.points:
        raise RuntimeError("完整轨迹为空")

    rclpy.init()
    node = pipeline.TaughtThenAnyGrasp()
    try:
        node.get_logger().warn(
            "READ-ONLY ENDPOINT OBSERVATION: no trajectory, hand, "
            "ARM or DISARM command can be sent"
        )
        report = EndpointDiagnostics(
            node, pipeline.base.LEFT_ARM_JOINTS, REPORT_FILE
        ).collect(
            target_positions=joint_trajectory.points[-1].positions,
            trajectory_file=TRAJECTORY_FILE,
            trajectory_sha256=digest,
            tolerance_rad=pipeline.GRASP_ENDPOINT_TOLERANCE_RAD,
            origin_label="standalone_observation_start",
            frozen_target={
                "frame_id": metadata.get("target_frame"),
                "position_m": metadata.get("target_position"),
                "quaternion_xyzw": metadata.get(
                    "target_quaternion_xyzw"
                ),
                "orientation_consistency_mode": metadata.get(
                    "orientation_consistency_mode"
                ),
            },
        )
        node.get_logger().info(
            "READ-ONLY OBSERVATION COMPLETE | "
            f"t0={report['maximum_error_initial_rad']:.6f}rad, "
            f"t5={report['maximum_error_final_rad']:.6f}rad"
        )
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
