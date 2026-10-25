#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Plan or execute the post-teaching AnyGrasp arm-only stage.

Default mode is strictly plan-only.  ``--execute-saved`` consumes exactly the
saved and previewed RobotTrajectory.  This program contains no ARM/DISARM
service client and sends no RH56H1 command.
"""

import argparse
import copy
import hashlib
import json
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotTrajectory
from rclpy.serialization import deserialize_message

import execute_left_taught_then_anygrasp as pipeline
from g1d_project_paths import RUN_DIR, ensure_run_dir


HERE = Path(__file__).resolve().parent
ensure_run_dir()
TRAJECTORY_FILE = RUN_DIR / "g1_left_measured_taught_end_to_grasp.cdr"
METADATA_FILE = RUN_DIR / "g1_left_measured_taught_end_to_grasp.json"
MAX_GRASP_DETOUR_RAD = 0.30


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "示教完成后的AnyGrasp阶段；默认从当前实测状态重新规划，"
            "--execute-saved只执行已保存和预览的轨迹"
        )
    )
    parser.add_argument(
        "--execute-saved",
        action="store_true",
        help="执行已保存轨迹；程序不会ARM/DISARM或控制灵巧手",
    )
    return parser.parse_args()


def enforce_grasp_detour(metrics):
    violations = {
        name: float(value)
        for name, value in metrics[2].items()
        if float(value) > MAX_GRASP_DETOUR_RAD
    }
    if violations:
        name = max(violations, key=violations.get)
        raise RuntimeError(
            "抓取轨迹绝对绕行超限："
            f"{name}={violations[name]:.6f} rad > "
            f"{MAX_GRASP_DETOUR_RAD:.6f} rad"
        )


def load_saved_artifact():
    serialized = TRAJECTORY_FILE.read_bytes()
    metadata = json.loads(METADATA_FILE.read_text(encoding="utf-8"))
    if metadata.get("artifact_type") != "g1_measured_taught_end_to_anygrasp_a":
        raise RuntimeError("保存的抓取轨迹元数据类型不匹配")
    if metadata.get("candidate") != "A":
        raise RuntimeError("保存的抓取目标不是candidate A")
    if metadata.get("no_hand_command_sent") is not True:
        raise RuntimeError("保存的抓取元数据缺少arm-only声明")
    if hashlib.sha256(serialized).hexdigest() != metadata.get("trajectory_sha256"):
        raise RuntimeError("保存的抓取轨迹SHA-256校验失败")
    trajectory = deserialize_message(serialized, RobotTrajectory)
    joint_trajectory = trajectory.joint_trajectory
    if list(joint_trajectory.joint_names) != pipeline.base.LEFT_ARM_JOINTS:
        raise RuntimeError("保存的抓取轨迹不是严格的左臂7关节轨迹")
    if not joint_trajectory.points:
        raise RuntimeError("保存的抓取轨迹为空")
    if len(joint_trajectory.points) != int(metadata.get("point_count", -1)):
        raise RuntimeError("保存的抓取轨迹点数与元数据不匹配")
    return trajectory, metadata


def target_from_metadata(metadata):
    target = PoseStamped()
    target.header.frame_id = str(metadata["target_frame"])
    position = metadata["target_position"]
    quaternion = metadata["target_quaternion_xyzw"]
    target.pose.position.x = float(position[0])
    target.pose.position.y = float(position[1])
    target.pose.position.z = float(position[2])
    target.pose.orientation.x = float(quaternion[0])
    target.pose.orientation.y = float(quaternion[1])
    target.pose.orientation.z = float(quaternion[2])
    target.pose.orientation.w = float(quaternion[3])
    return target


def check_current_against_first(node, trajectory):
    current = node.wait_fresh_state(clear=True)
    current_map = dict(zip(current.name, current.position))
    missing = [
        name for name in pipeline.base.LEFT_ARM_JOINTS
        if name not in current_map
    ]
    if missing:
        raise RuntimeError(f"当前/joint_states缺少左臂关节：{missing}")
    first = trajectory.joint_trajectory.points[0].positions
    errors = {
        name: abs(float(current_map[name]) - float(expected))
        for name, expected in zip(pipeline.base.LEFT_ARM_JOINTS, first)
    }
    worst_name = max(errors, key=errors.get)
    worst_error = errors[worst_name]
    if worst_error > pipeline.base.PRE_EXECUTE_START_TOLERANCE_RAD:
        raise RuntimeError(
            "当前姿态与保存轨迹起点不匹配，必须重新plan-only："
            f"{worst_name}={worst_error:.6f} rad > "
            f"{pipeline.base.PRE_EXECUTE_START_TOLERANCE_RAD:.6f} rad"
        )
    node.get_logger().info(
        "SAVED START MATCHED | "
        f"max_error={worst_error:.6f}rad, joint={worst_name}"
    )
    return current


def run_plan_only(node):
    node.get_logger().warn(
        "POST-TEACH PLAN ONLY: no ExecuteTrajectory goal will be sent"
    )
    node.validate_scene()
    measured, stability_ranges = node.sample_stable_endpoint()
    target, roll, _, _ = node.acquire_stable_target()
    (
        trajectory,
        start_state,
        planning_time,
        target,
        roll_relaxation,
    ) = node.plan_grasp(measured, target)
    roll += roll_relaxation
    metrics = node.validate_path_shape(
        trajectory.joint_trajectory, "post-teach grasp path"
    )
    enforce_grasp_detour(metrics)
    node.validate_path_collisions(
        trajectory.joint_trajectory, measured, "抓取轨迹"
    )
    node.save_grasp(
        trajectory,
        measured,
        target,
        roll,
        planning_time,
        metrics,
        stability_ranges,
    )
    node.publish_grasp_preview(start_state, trajectory)
    node.get_logger().info(
        "POST-TEACH GRASP PLAN READY | "
        f"points={len(trajectory.joint_trajectory.points)}, "
        f"planning_time={planning_time:.3f}s"
    )
    node.get_logger().info(
        "PLAN ONLY COMPLETE: inspect RViz; no robot command was sent"
    )


def run_execute_saved(node):
    trajectory, metadata = load_saved_artifact()
    node.get_logger().warn(
        "ARM-ONLY SAVED EXECUTION MODE: this program will not ARM/DISARM"
    )
    node.validate_scene()
    current = check_current_against_first(node, trajectory)
    metrics = node.validate_path_shape(
        trajectory.joint_trajectory, "saved post-teach grasp path"
    )
    enforce_grasp_detour(metrics)
    node.validate_path_collisions(
        trajectory.joint_trajectory, current, "保存抓取轨迹"
    )
    target = target_from_metadata(metadata)
    node.validate_target_still_current(target)
    phrase = input(
        "确认RViz轨迹、现场急停和外部ARM状态后，输入 EXECUTE GRASP："
    ).strip()
    if phrase != "EXECUTE GRASP":
        raise RuntimeError("未收到准确确认短语，拒绝执行")

    # The operator may spend time reviewing.  Revalidate all volatile state
    # after confirmation and immediately before sending the action.
    current = check_current_against_first(node, trajectory)
    node.validate_target_still_current(target)
    node.execute_trajectory(trajectory, "saved current-state -> AnyGrasp A")
    node.check_measured_target(
        trajectory.joint_trajectory.points[-1].positions,
        pipeline.GRASP_ENDPOINT_TOLERANCE_RAD,
        "grasp-end arrival check",
    )
    node.get_logger().info(
        "GRASP ENDPOINT READY; no RH56H1 command and no lift were sent"
    )


def main():
    args = parse_args()
    rclpy.init()
    node = pipeline.TaughtThenAnyGrasp()
    return_code = 2
    try:
        if args.execute_saved:
            run_execute_saved(node)
        else:
            run_plan_only(node)
        return_code = 0
    except Exception as exc:
        node.get_logger().error(str(exc))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(return_code)


if __name__ == "__main__":
    main()
