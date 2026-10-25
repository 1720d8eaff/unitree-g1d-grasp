#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Execute exactly one previously reviewed visual micro-adjustment."""

import json
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotTrajectory
from rclpy.serialization import deserialize_message

import execute_left_taught_then_anygrasp as pipeline
from endpoint_diagnostics import EndpointDiagnostics, file_sha256
from g1d_project_paths import RUN_DIR, ensure_run_dir


HERE = Path(__file__).resolve().parent
ensure_run_dir()
TRAJECTORY_FILE = RUN_DIR / "g1_left_visual_micro_adjustment.cdr"
METADATA_FILE = RUN_DIR / "g1_left_visual_micro_adjustment.json"
DIAGNOSTICS_FILE = RUN_DIR / "g1_left_visual_micro_adjustment_diagnostics.json"


def pose_from_dict(data):
    result = PoseStamped()
    result.header.frame_id = data["frame_id"]
    p = data["position_m"]
    q = data["quaternion_xyzw"]
    result.pose.position.x, result.pose.position.y, result.pose.position.z = p
    (
        result.pose.orientation.x,
        result.pose.orientation.y,
        result.pose.orientation.z,
        result.pose.orientation.w,
    ) = q
    return result


def main():
    serialized = TRAJECTORY_FILE.read_bytes()
    metadata = json.loads(METADATA_FILE.read_text(encoding="utf-8"))
    digest = file_sha256(TRAJECTORY_FILE)
    if digest != metadata.get("trajectory_sha256"):
        raise RuntimeError("视觉微调CDR与JSON的SHA-256不一致")
    trajectory = deserialize_message(serialized, RobotTrajectory)
    jt = trajectory.joint_trajectory
    if list(jt.joint_names) != pipeline.base.LEFT_ARM_JOINTS or not jt.points:
        raise RuntimeError("视觉微调轨迹关节顺序错误或轨迹为空")

    rclpy.init()
    node = pipeline.TaughtThenAnyGrasp()
    try:
        node.get_logger().warn(
            "REAL MOTION: exact reviewed visual micro-adjustment; "
            "no ARM/DISARM or RH56H1 command"
        )
        node.validate_scene()
        measured = node.wait_fresh_state(clear=True)
        current = dict(zip(measured.name, measured.position))
        errors = {
            name: abs(float(current[name]) - float(expected))
            for name, expected in zip(
                pipeline.base.LEFT_ARM_JOINTS, jt.points[0].positions
            )
        }
        worst = max(errors, key=errors.get)
        if errors[worst] > pipeline.base.PRE_EXECUTE_START_TOLERANCE_RAD:
            raise RuntimeError(
                "视觉微调起点已改变，必须重新Plan-only："
                f"{worst}={errors[worst]:.6f}rad"
            )
        node.validate_path_shape(jt, "saved visual micro-adjustment")
        node.validate_path_collisions(jt, measured, "保存的视觉微调轨迹")
        frozen_visual_target = pose_from_dict(metadata["fresh_visual_target"])
        node.validate_target_still_current(frozen_visual_target)
        node.execute_trajectory(trajectory, "saved visual micro-adjustment")

        report = EndpointDiagnostics(
            node, pipeline.base.LEFT_ARM_JOINTS, DIAGNOSTICS_FILE
        ).collect(
            target_positions=jt.points[-1].positions,
            trajectory_file=TRAJECTORY_FILE,
            trajectory_sha256=digest,
            tolerance_rad=pipeline.GRASP_ENDPOINT_TOLERANCE_RAD,
            origin_label="visual_micro_adjustment_action_result",
            origin_monotonic=time.monotonic(),
            frozen_target=metadata["bounded_micro_target"],
        )
        if not report["all_samples_within_tolerance"]:
            raise RuntimeError(
                "视觉微调执行完成但关节终点未稳定进入0.015rad；"
                "已保存诊断，不得CLOSE或自动继续下一轮"
            )
        node.get_logger().info(
            "VISUAL MICRO-ADJUSTMENT PASSED; stop and inspect physically; "
            "no hand command was sent"
        )
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
