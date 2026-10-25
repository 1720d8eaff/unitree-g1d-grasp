#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Plan one bounded visual micro-adjustment from the measured arm endpoint.

This file is deliberately plan-only.  It cannot send ExecuteTrajectory,
ARM/DISARM, or RH56H1 commands.  Execution is isolated in the companion
saved-trajectory executor so the exact path can be inspected in RViz first.
"""

import argparse
import copy
import hashlib
import json
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from moveit_msgs.msg import RobotTrajectory
from rclpy.serialization import deserialize_message, serialize_message

import execute_left_taught_then_anygrasp as pipeline
from endpoint_diagnostics import EndpointDiagnostics
from g1d_project_paths import RUN_DIR, ensure_run_dir


HERE = Path(__file__).resolve().parent
ensure_run_dir()
TRAJECTORY_FILE = RUN_DIR / "g1_left_visual_micro_adjustment.cdr"
METADATA_FILE = RUN_DIR / "g1_left_visual_micro_adjustment.json"
FULL_TRAJECTORY_FILE = RUN_DIR / "g1_left_full_taught_to_anygrasp.cdr"
MAX_TRANSLATION_M = 0.005
MAX_ROTATION_RAD = math.radians(3.0)
MAX_JOINT_DETOUR_RAD = 0.12
MAX_CURRENT_TO_FULL_ENDPOINT_ERROR_RAD = 0.15
MAX_CURRENT_TCP_TO_VISUAL_TARGET_M = 0.08


def parse_args():
    parser = argparse.ArgumentParser(
        description="从当前实测TCP向新鲜AnyGrasp目标规划一小步视觉修正；只规划"
    )
    parser.add_argument(
        "--max-translation-mm", type=float, default=3.0,
        help="本轮最大平移，范围1至5毫米，默认3毫米",
    )
    parser.add_argument(
        "--max-rotation-deg", type=float, default=2.0,
        help="本轮最大姿态变化，范围0至3度，默认2度",
    )
    args = parser.parse_args()
    if not 1.0 <= args.max_translation_mm <= 5.0:
        parser.error("--max-translation-mm必须在1至5之间")
    if not 0.0 <= args.max_rotation_deg <= 3.0:
        parser.error("--max-rotation-deg必须在0至3之间")
    return args


def pose_dict(stamped):
    p = stamped.pose.position
    q = stamped.pose.orientation
    return {
        "frame_id": stamped.header.frame_id,
        "position_m": [float(p.x), float(p.y), float(p.z)],
        "quaternion_xyzw": [float(q.x), float(q.y), float(q.z), float(q.w)],
    }


def normalized_quaternion(q):
    norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
    if norm < 1e-12:
        raise RuntimeError("姿态四元数为零")
    return [q.x/norm, q.y/norm, q.z/norm, q.w/norm]


def slerp_quaternion(start, goal, fraction):
    a = normalized_quaternion(start)
    b = normalized_quaternion(goal)
    dot = sum(x*y for x, y in zip(a, b))
    if dot < 0.0:
        b = [-value for value in b]
        dot = -dot
    dot = max(-1.0, min(1.0, dot))
    if dot > 0.9995:
        values = [x + fraction*(y-x) for x, y in zip(a, b)]
        norm = math.sqrt(sum(value*value for value in values))
        values = [value/norm for value in values]
    else:
        theta = math.acos(dot)
        sine = math.sin(theta)
        weight_a = math.sin((1.0-fraction)*theta) / sine
        weight_b = math.sin(fraction*theta) / sine
        values = [weight_a*x + weight_b*y for x, y in zip(a, b)]
    result = Quaternion()
    result.x, result.y, result.z, result.w = values
    return result


def bounded_target(actual, desired, max_translation_m, max_rotation_rad):
    result = PoseStamped()
    result.header = copy.deepcopy(desired.header)
    result.pose = copy.deepcopy(actual.pose)

    delta = [
        desired.pose.position.x - actual.pose.position.x,
        desired.pose.position.y - actual.pose.position.y,
        desired.pose.position.z - actual.pose.position.z,
    ]
    distance = math.sqrt(sum(value*value for value in delta))
    translation_fraction = (
        1.0 if distance <= max_translation_m else max_translation_m / distance
    )
    result.pose.position.x += delta[0] * translation_fraction
    result.pose.position.y += delta[1] * translation_fraction
    result.pose.position.z += delta[2] * translation_fraction

    angle = pipeline.quaternion_angle(
        actual.pose.orientation, desired.pose.orientation
    )
    rotation_fraction = (
        1.0 if angle <= max_rotation_rad or angle < 1e-12
        else max_rotation_rad / angle
    )
    result.pose.orientation = slerp_quaternion(
        actual.pose.orientation, desired.pose.orientation, rotation_fraction
    )
    return result, distance, angle, translation_fraction, rotation_fraction


def enforce_micro_path(metrics):
    violations = {
        name: float(value) for name, value in metrics[2].items()
        if float(value) > MAX_JOINT_DETOUR_RAD
    }
    if violations:
        name = max(violations, key=violations.get)
        raise RuntimeError(
            "视觉微调关节绕行超限："
            f"{name}={violations[name]:.6f}rad > "
            f"{MAX_JOINT_DETOUR_RAD:.6f}rad"
        )


def require_near_last_full_endpoint(measured):
    if not FULL_TRAJECTORY_FILE.exists():
        raise RuntimeError("缺少上一条完整轨迹，不能规划终点微调")
    full = deserialize_message(
        FULL_TRAJECTORY_FILE.read_bytes(), RobotTrajectory
    )
    jt = full.joint_trajectory
    if list(jt.joint_names) != pipeline.base.LEFT_ARM_JOINTS or not jt.points:
        raise RuntimeError("上一条完整轨迹关节顺序错误或轨迹为空")
    current = dict(zip(measured.name, measured.position))
    missing = [
        name for name in pipeline.base.LEFT_ARM_JOINTS if name not in current
    ]
    if missing:
        raise RuntimeError(f"当前状态缺少左臂关节：{missing}")
    errors = {
        name: abs(float(current[name]) - float(expected))
        for name, expected in zip(
            pipeline.base.LEFT_ARM_JOINTS, jt.points[-1].positions
        )
    }
    worst = max(errors, key=errors.get)
    if errors[worst] > MAX_CURRENT_TO_FULL_ENDPOINT_ERROR_RAD:
        raise RuntimeError(
            "当前手臂不在上一条完整轨迹终点附近，禁止生成微调："
            f"{worst}={errors[worst]:.6f}rad > "
            f"{MAX_CURRENT_TO_FULL_ENDPOINT_ERROR_RAD:.6f}rad；"
            "请先重新执行完整arm-only路径，并在到位后、DISARM前运行本程序"
        )
    return errors[worst], worst


def main():
    args = parse_args()
    max_translation_m = args.max_translation_mm / 1000.0
    max_rotation_rad = math.radians(args.max_rotation_deg)

    rclpy.init()
    node = pipeline.TaughtThenAnyGrasp()
    try:
        node.get_logger().warn(
            "VISUAL MICRO-ADJUSTMENT PLAN ONLY: no ExecuteTrajectory, "
            "ARM/DISARM or RH56H1 command can be sent"
        )
        node.validate_scene()
        measured = node.wait_fresh_state(clear=True)
        endpoint_error, endpoint_joint = require_near_last_full_endpoint(
            measured
        )
        fk_helper = EndpointDiagnostics(
            node, pipeline.base.LEFT_ARM_JOINTS,
            RUN_DIR / "unused_visual_micro_diagnostics.json",
        )
        actual_tcp = fk_helper.compute_fk(measured)
        desired, _, position_spread, orientation_spread = (
            node.acquire_stable_target()
        )
        current_visual_distance, _ = pipeline.pose_distance(
            actual_tcp, desired
        )
        if current_visual_distance > MAX_CURRENT_TCP_TO_VISUAL_TARGET_M:
            raise RuntimeError(
                "当前TCP离视觉目标过远，禁止把大范围动作伪装成微调："
                f"{current_visual_distance:.6f}m > "
                f"{MAX_CURRENT_TCP_TO_VISUAL_TARGET_M:.6f}m"
            )
        micro_target, residual_m, residual_rad, tf, rf = bounded_target(
            actual_tcp, desired, max_translation_m, max_rotation_rad
        )

        # Micro correction starts from the measured, already-inspected hand
        # pose.  Its orientation delta is bounded above, so it must not reuse
        # the initial-grasp absolute thumb-component filter: a 2 degree step
        # can be safe while still remaining below that filter's 0.15 absolute
        # component threshold.  Keep the target exact and forbid roll search.
        pipeline.IK_ROLL_RELAXATIONS_DEG = (0.0,)
        pipeline.MIN_THUMB_BODYWARD_COMPONENT = -1.0
        pipeline.MIN_THUMB_DOWNWARD_COMPONENT = -1.0
        pipeline.GRASP_PLANNING_VELOCITY_SCALE = 0.02
        pipeline.GRASP_PLANNING_ACCELERATION_SCALE = 0.02
        trajectory, start_state, planning_time, selected, roll_relaxation = (
            node.plan_grasp(measured, micro_target)
        )
        if abs(roll_relaxation) > 1e-9:
            raise RuntimeError("视觉微调不得改变目标滚转")
        trajectory = node.densify_trajectory(
            trajectory, "visual micro-adjustment"
        )
        metrics = node.validate_path_shape(
            trajectory.joint_trajectory, "visual micro-adjustment"
        )
        enforce_micro_path(metrics)
        node.validate_path_collisions(
            trajectory.joint_trajectory, measured, "视觉微调轨迹"
        )
        node.publish_grasp_preview(start_state, trajectory)

        serialized = serialize_message(trajectory)
        digest = hashlib.sha256(serialized).hexdigest()
        current = dict(zip(measured.name, measured.position))
        metadata = {
            "schema_version": 1,
            "artifact_type": "g1_left_visual_micro_adjustment",
            "plan_only_generator": True,
            "program_contains_execute_client": False,
            "program_contains_arm_disarm_calls": False,
            "program_contains_hand_commands": False,
            "generated_unix_time": time.time(),
            "joint_names": pipeline.base.LEFT_ARM_JOINTS,
            "point_count": len(trajectory.joint_trajectory.points),
            "planning_time_sec": planning_time,
            "start_positions_rad": [
                float(current[name]) for name in pipeline.base.LEFT_ARM_JOINTS
            ],
            "start_sha256": pipeline.fingerprint(
                pipeline.base.LEFT_ARM_JOINTS,
                [float(current[name]) for name in pipeline.base.LEFT_ARM_JOINTS],
            ),
            "current_to_last_full_endpoint_max_error_rad": endpoint_error,
            "current_to_last_full_endpoint_worst_joint": endpoint_joint,
            "maximum_allowed_current_to_full_endpoint_error_rad": (
                MAX_CURRENT_TO_FULL_ENDPOINT_ERROR_RAD
            ),
            "actual_tcp_at_plan": pose_dict(actual_tcp),
            "fresh_visual_target": pose_dict(desired),
            "bounded_micro_target": pose_dict(selected),
            "full_position_residual_m": residual_m,
            "full_orientation_residual_rad": residual_rad,
            "maximum_translation_step_m": max_translation_m,
            "maximum_rotation_step_rad": max_rotation_rad,
            "absolute_thumb_filter_disabled_for_bounded_micro_step": True,
            "ik_roll_relaxation_disabled": True,
            "translation_fraction_used": tf,
            "rotation_fraction_used": rf,
            "target_position_spread_m": position_spread,
            "target_orientation_spread_rad": orientation_spread,
            "maximum_joint_detour_rad": max(metrics[2].values()),
            "endpoint_tolerance_rad": pipeline.GRASP_ENDPOINT_TOLERANCE_RAD,
            "trajectory_sha256": digest,
        }
        pipeline.base.atomic_write(TRAJECTORY_FILE, serialized, binary=True)
        pipeline.base.atomic_write(
            METADATA_FILE,
            json.dumps(metadata, ensure_ascii=False, indent=2) + "\n",
        )
        node.get_logger().info(
            "VISUAL MICRO-ADJUSTMENT PLAN READY | "
            f"full_residual={residual_m:.6f}m/"
            f"{math.degrees(residual_rad):.3f}deg, "
            f"bounded_step={max_translation_m:.6f}m/"
            f"{args.max_rotation_deg:.3f}deg, "
            f"points={metadata['point_count']}; inspect RViz before execution"
        )
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
