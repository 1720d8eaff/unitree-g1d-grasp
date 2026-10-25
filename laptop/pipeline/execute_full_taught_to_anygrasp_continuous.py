#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Plan and optionally execute one continuous taught-to-grasp trajectory.

The exact executed RobotTrajectory contains, in order:
  measured current state -> taught start -> filtered taught path -> AnyGrasp A

The main taught-to-grasp path is sent as one ExecuteTrajectory goal.  With the
explicit ``--execute --visual-adjust`` combination, one additional bounded
visual correction goal may be planned from the measured endpoint and sent.
The program contains no ARM/DISARM service client.  RH56H1 commands are enabled
only by explicit execution flags.  The saved-path ``--grasp-cycle`` mode plans
and previews a 5 cm Cartesian lift, preopens before the reviewed main path,
starts CLOSE one second before its nominal endpoint, and lifts only after the
main endpoint is accepted.  Default is plan-only.
"""

import argparse
import copy
import hashlib
import json
import math
import os
import time
from pathlib import Path

import rclpy
from moveit_msgs.msg import DisplayTrajectory, RobotState, RobotTrajectory
from moveit_msgs.srv import GetCartesianPath
from rclpy.duration import Duration
from rclpy.serialization import deserialize_message, serialize_message

import execute_left_taught_then_anygrasp as pipeline
import anygrasp_segment as approach
import plan_visual_micro_adjustment as visual_micro
from endpoint_diagnostics import EndpointDiagnostics
from anygrasp_segment import (
    plan_anygrasp_segment,
    validate_final_approach_wrist_yaw,
)
from left_taught_segment import plan_left_taught_segment
from rh56h1_gripper_stage import RH56H1GripperStage
from g1d_project_paths import RUN_DIR, ensure_run_dir


HERE = Path(__file__).resolve().parent
ensure_run_dir()
FULL_TRAJECTORY_FILE = RUN_DIR / "g1_left_full_taught_to_anygrasp.cdr"
FULL_METADATA_FILE = RUN_DIR / "g1_left_full_taught_to_anygrasp.json"
ENDPOINT_DIAGNOSTICS_FILE = RUN_DIR / "g1_left_endpoint_diagnostics.json"
VISUAL_ENDPOINT_DIAGNOSTICS_FILE = (
    RUN_DIR / "g1_left_visual_micro_adjustment_diagnostics.json"
)
LIFT_TRAJECTORY_FILE = RUN_DIR / "g1_left_post_grasp_lift_5cm.cdr"
LIFT_METADATA_FILE = RUN_DIR / "g1_left_post_grasp_lift_5cm.json"
SEAM_DURATION_SEC = 0.04
RELAY_TIME_SCALE = 5.40
RELAY_FINAL_SETTLE_SEC = 3.0
ONBOARD_MAX_SPEED_RAD_SEC = 0.35
ONBOARD_MAX_DURATION_SEC = 60.0
MAX_GRASP_SEGMENT_DETOUR_RAD = 0.30
FIRST_REAL_EXECUTION_SLOWDOWN = 1.25
GRASP_CLOSE_BEFORE_ENDPOINT_SEC = 1.0
POST_GRASP_LIFT_M = 0.05
RVIZ_REVIEWED_TARGET_SHIFT_M = 0.015
RVIZ_REVIEWED_TARGET_SHIFT_RAD = math.radians(20.0)

# Shorten MoveIt-generated alignment/grasp portions.  The relay applies 5.40x
# time scaling afterwards; the final full-path speed audit remains mandatory.
MOVEIT_PLAN_VELOCITY_SCALE = 0.10
MOVEIT_PLAN_ACCELERATION_SCALE = 0.05


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "生成当前姿态→示教首点→示教动作→AnyGrasp A的一条连续轨迹；"
            "默认仅规划和RViz预览"
        )
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="执行本次刚生成并预览的整段轨迹；不会自动ARM/DISARM",
    )
    parser.add_argument(
        "--execute-saved",
        action="store_true",
        help="执行G1D_RUN_DIR中已保存并预览的完整轨迹；不会重新规划",
    )
    parser.add_argument(
        "--accept-known-combined-reversal",
        action="store_true",
        help="明确接受保存轨迹超过0.30rad的已知整体关节折返",
    )
    parser.add_argument(
        "--accept-reviewed-target-drift",
        action="store_true",
        help=(
            "仅配合--execute-saved --grasp-cycle：在完整RViz人工审核后，"
            "允许保存目标与当前目标相差最多15毫米/20度"
        ),
    )
    parser.add_argument(
        "--with-hand",
        action="store_true",
        help="执行时调用WIDTH_READY→PREOPEN→手臂→终点验收→CLOSE",
    )
    parser.add_argument(
        "--grasp-cycle",
        action="store_true",
        help=(
            "仅配合--execute-saved：PREOPEN→已审核主轨迹→终点前1秒CLOSE"
            "→终点验收→沿torso_link +Z抬升5厘米；不会自动ARM/DISARM或RELEASE"
        ),
    )
    parser.add_argument(
        "--object-width-m",
        type=float,
        default=0.05,
        help="灵巧手物体宽度，默认0.05米",
    )
    parser.add_argument(
        "--visual-adjust",
        action="store_true",
        help=(
            "完整轨迹终点未进入0.015rad时，自动重新观测并执行一轮"
            "受限视觉微调；必须与--execute一起使用"
        ),
    )
    parser.add_argument(
        "--visual-adjust-mm",
        type=float,
        default=3.0,
        help="自动视觉微调单轮最大平移，范围1至5毫米，默认3毫米",
    )
    parser.add_argument(
        "--visual-adjust-deg",
        type=float,
        default=2.0,
        help="自动视觉微调单轮最大姿态变化，范围0至3度，默认2度",
    )
    args = parser.parse_args()
    if args.execute and args.execute_saved:
        parser.error("--execute与--execute-saved不能同时使用")
    if args.accept_known_combined_reversal and not args.execute_saved:
        parser.error("--accept-known-combined-reversal仅用于--execute-saved")
    if args.accept_reviewed_target_drift and not (
        args.execute_saved and args.grasp_cycle
    ):
        parser.error(
            "--accept-reviewed-target-drift仅用于"
            "--execute-saved --grasp-cycle"
        )
    if args.with_hand and not args.execute:
        parser.error("--with-hand必须与--execute一起使用")
    if args.grasp_cycle and not args.execute_saved:
        parser.error("--grasp-cycle必须与--execute-saved一起使用")
    if args.grasp_cycle and args.with_hand:
        parser.error("--grasp-cycle与--with-hand不能同时使用")
    if args.grasp_cycle and args.visual_adjust:
        parser.error("--grasp-cycle不能与--visual-adjust同时使用")
    if args.visual_adjust and not args.execute:
        parser.error("--visual-adjust必须与--execute一起使用")
    if not 1.0 <= args.visual_adjust_mm <= 5.0:
        parser.error("--visual-adjust-mm必须在1至5之间")
    if not 0.0 <= args.visual_adjust_deg <= 3.0:
        parser.error("--visual-adjust-deg必须在0至3之间")
    return args


def load_saved_full_artifact():
    serialized = FULL_TRAJECTORY_FILE.read_bytes()
    metadata = json.loads(FULL_METADATA_FILE.read_text(encoding="utf-8"))
    if metadata.get("artifact_type") != (
        "g1_left_full_taught_to_anygrasp_continuous"
    ):
        raise RuntimeError("保存的完整轨迹元数据类型错误")
    if metadata.get("target_source_mode") != (
        "live_anygrasp_position_taught_orientation"
    ):
        raise RuntimeError(
            "保存轨迹不是实时AnyGrasp相对施教标定模式；"
            "固定torso_link TCP旧轨迹已失效，必须重新Plan-only"
        )
    if metadata.get("planner_policy_version") != (
        approach.PLANNER_POLICY_VERSION
    ):
        raise RuntimeError(
            "保存轨迹的规划约束版本已失效，必须用当前版本重新Plan-only"
        )
    digest = hashlib.sha256(serialized).hexdigest()
    if digest != metadata.get("trajectory_sha256"):
        raise RuntimeError("保存的完整轨迹CDR与JSON的SHA-256不一致")
    trajectory = deserialize_message(serialized, RobotTrajectory)
    joint_trajectory = trajectory.joint_trajectory
    if list(joint_trajectory.joint_names) != pipeline.base.LEFT_ARM_JOINTS:
        raise RuntimeError("保存的完整轨迹不是严格的左臂7关节轨迹")
    if not joint_trajectory.points:
        raise RuntimeError("保存的完整轨迹为空")
    if len(joint_trajectory.points) != int(metadata.get("point_count", -1)):
        raise RuntimeError("保存的完整轨迹点数与元数据不一致")
    return trajectory, metadata, digest


def target_from_saved_metadata(metadata):
    target = pipeline.PoseStamped()
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


def live_target_from_saved_metadata(metadata):
    position = metadata.get("live_anygrasp_position_before_manual_reference")
    quaternion = metadata.get(
        "live_anygrasp_quaternion_before_manual_reference"
    )
    if position is None or quaternion is None:
        return None
    target = pipeline.PoseStamped()
    target.header.frame_id = str(metadata["target_frame"])
    target.pose.position.x = float(position[0])
    target.pose.position.y = float(position[1])
    target.pose.position.z = float(position[2])
    target.pose.orientation.x = float(quaternion[0])
    target.pose.orientation.y = float(quaternion[1])
    target.pose.orientation.z = float(quaternion[2])
    target.pose.orientation.w = float(quaternion[3])
    return target


def validate_saved_target_current(
    node, frozen, accept_reviewed_drift, label, frozen_live=None
):
    latest, _, position_spread, orientation_spread = (
        node.acquire_stable_target()
    )
    comparison_frozen = frozen if frozen_live is None else frozen_live
    comparison_latest = (
        latest
        if frozen_live is None
        else node.last_raw_anygrasp_reference
    )
    distance, full_angle = pipeline.pose_distance(
        comparison_frozen, comparison_latest
    )
    approach_angle = pipeline.approach_axis_angle(
        comparison_frozen, comparison_latest
    )
    position_limit = (
        RVIZ_REVIEWED_TARGET_SHIFT_M
        if accept_reviewed_drift
        else pipeline.PRE_EXECUTE_TARGET_SHIFT_M
    )
    approach_limit = (
        RVIZ_REVIEWED_TARGET_SHIFT_RAD
        if accept_reviewed_drift
        else pipeline.PRE_EXECUTE_TARGET_SHIFT_RAD
    )
    if distance > position_limit or approach_angle > approach_limit:
        raise RuntimeError(
            f"{label}目标变化超过允许范围："
            f"position={distance:.6f}m/{position_limit:.6f}m, "
            f"approach={approach_angle:.6f}rad/{approach_limit:.6f}rad, "
            f"full_orientation_diagnostic={full_angle:.6f}rad"
        )
    strict_exceeded = (
        distance > pipeline.PRE_EXECUTE_TARGET_SHIFT_M
        or approach_angle > pipeline.PRE_EXECUTE_TARGET_SHIFT_RAD
    )
    log = node.get_logger().warn if strict_exceeded else node.get_logger().info
    log(
        f"{label} TARGET CHECK PASSED | "
        f"mode={'rviz_reviewed_relaxed' if accept_reviewed_drift else 'strict'}, "
        f"position_shift={distance:.6f}m, "
        f"approach_shift={approach_angle:.6f}rad, "
        f"fresh_position_spread={position_spread:.6f}m, "
        f"fresh_orientation_spread={orientation_spread:.6f}rad"
    )


def require_saved_start(node, trajectory):
    measured = node.wait_fresh_state(clear=True)
    current = dict(zip(measured.name, measured.position))
    missing = [
        name for name in pipeline.base.LEFT_ARM_JOINTS
        if name not in current
    ]
    if missing:
        raise RuntimeError(f"当前joint_states缺少左臂关节：{missing}")
    first = trajectory.joint_trajectory.points[0].positions
    errors = {
        name: abs(float(current[name]) - float(expected))
        for name, expected in zip(pipeline.base.LEFT_ARM_JOINTS, first)
    }
    worst = max(errors, key=errors.get)
    if errors[worst] > pipeline.base.PRE_EXECUTE_START_TOLERANCE_RAD:
        raise RuntimeError(
            "当前姿态与保存轨迹起点不匹配："
            f"{worst}={errors[worst]:.6f}rad > "
            f"{pipeline.base.PRE_EXECUTE_START_TOLERANCE_RAD:.6f}rad"
        )
    node.get_logger().info(
        "SAVED FULL START MATCHED | "
        f"max_error={errors[worst]:.6f}rad, joint={worst}"
    )
    return measured


def joint_state_at_arm_positions(measured, positions):
    result = copy.deepcopy(measured)
    index = {name: i for i, name in enumerate(result.name)}
    missing = [
        name for name in pipeline.base.LEFT_ARM_JOINTS if name not in index
    ]
    if missing:
        raise RuntimeError(f"抬升起点缺少左臂关节：{missing}")
    values = list(result.position)
    for name, value in zip(pipeline.base.LEFT_ARM_JOINTS, positions):
        values[index[name]] = float(value)
    result.position = values
    if result.velocity:
        result.velocity = [0.0] * len(result.name)
    if result.effort:
        result.effort = [0.0] * len(result.name)
    return result


def plan_post_grasp_lift(node, measured, main_trajectory):
    if LIFT_TRAJECTORY_FILE.exists() or LIFT_METADATA_FILE.exists():
        raise RuntimeError(
            "本运行目录已存在抓取后抬升规划，拒绝覆盖；"
            "请重新执行./g1dctl plan生成新的运行目录"
        )
    main_points = main_trajectory.joint_trajectory.points
    if not main_points:
        raise RuntimeError("主轨迹为空，不能规划抓取后抬升")
    lift_start = joint_state_at_arm_positions(
        measured, main_points[-1].positions
    )
    fk = EndpointDiagnostics(
        node,
        pipeline.base.LEFT_ARM_JOINTS,
        RUN_DIR / "unused_post_grasp_lift_fk.json",
    )
    start_tcp = fk.compute_fk(lift_start)
    lift_target = copy.deepcopy(start_tcp)
    lift_target.pose.position.z += POST_GRASP_LIFT_M

    client = node.create_client(GetCartesianPath, approach.CARTESIAN_PATH_SERVICE)
    if not client.wait_for_service(timeout_sec=5.0):
        raise RuntimeError(f"{approach.CARTESIAN_PATH_SERVICE}不可用")
    request = GetCartesianPath.Request()
    request.header = copy.deepcopy(lift_target.header)
    request.start_state.joint_state = copy.deepcopy(lift_start)
    request.start_state.is_diff = False
    request.group_name = pipeline.base.GROUP_NAME
    request.link_name = pipeline.EE_LINK
    request.waypoints = [copy.deepcopy(lift_target.pose)]
    request.max_step = approach.CARTESIAN_MAX_STEP_M
    request.jump_threshold = 0.0
    request.prismatic_jump_threshold = 0.0
    request.revolute_jump_threshold = (
        approach.CARTESIAN_REVOLUTE_JUMP_THRESHOLD_RAD
    )
    request.avoid_collisions = True
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    response = future.result()
    if response is None:
        raise RuntimeError("抓取后5厘米Cartesian抬升规划未返回结果")
    if (
        int(response.error_code.val) != 1
        or float(response.fraction) < approach.CARTESIAN_MIN_FRACTION
    ):
        raise RuntimeError(
            "抓取后5厘米Cartesian抬升规划失败："
            f"error_code={response.error_code.val}, "
            f"fraction={response.fraction:.6f}"
        )
    lift = approach.parameterize_cartesian_trajectory(response.solution)
    validate_final_approach_wrist_yaw(lift, "抓取后5厘米抬升")
    metrics = node.validate_path_shape(
        lift.joint_trajectory, "post-grasp 5 cm lift"
    )
    node.validate_path_collisions(
        lift.joint_trajectory, lift_start, "抓取后5厘米抬升"
    )
    timing = audit_relay_timing(lift)
    serialized = serialize_message(lift)
    digest = hashlib.sha256(serialized).hexdigest()
    metadata = {
        "schema_version": 1,
        "artifact_type": "g1_left_post_grasp_cartesian_lift",
        "frame_id": lift_target.header.frame_id,
        "lift_distance_m": POST_GRASP_LIFT_M,
        "cartesian_fraction": float(response.fraction),
        "cartesian_step_m": approach.CARTESIAN_MAX_STEP_M,
        "joint_names": list(lift.joint_trajectory.joint_names),
        "point_count": len(lift.joint_trajectory.points),
        "trajectory_sha256": digest,
        "maximum_point_step_rad": metrics[0],
        "minimum_joint_limit_margin_rad": metrics[1],
        "joint_detour_rad": metrics[2],
        "expected_total_after_relay_sec": timing[2],
        "target_position_m": [
            float(lift_target.pose.position.x),
            float(lift_target.pose.position.y),
            float(lift_target.pose.position.z),
        ],
    }
    pipeline.base.atomic_write(LIFT_TRAJECTORY_FILE, serialized, binary=True)
    pipeline.base.atomic_write(
        LIFT_METADATA_FILE,
        json.dumps(metadata, ensure_ascii=False, indent=2) + "\n",
    )
    node.get_logger().info(
        "POST-GRASP LIFT PLAN READY | "
        f"distance={POST_GRASP_LIFT_M:.3f}m, "
        f"fraction={response.fraction:.6f}, "
        f"points={metadata['point_count']}, "
        f"expected_total={timing[2]:.3f}s"
    )
    return lift, lift_start, metadata


def publish_saved_grasp_cycle_preview(node, start_state, main, lift):
    display = DisplayTrajectory()
    display.model_id = "G1_D"
    display.trajectory_start = start_state
    display.trajectory.append(main)
    display.trajectory.append(lift)
    node.display_pub.publish(display)
    node.get_logger().info(
        "RViz已发布抓取循环："
        f"主轨迹={len(main.joint_trajectory.points)}点，"
        f"抬升5厘米={len(lift.joint_trajectory.points)}点"
    )


def execute_saved_full(args):
    trajectory, metadata, digest = load_saved_full_artifact()
    reversal = float(metadata.get("combined_waypoint_reversal_rad", 1e9))
    if reversal > MAX_GRASP_SEGMENT_DETOUR_RAD:
        if not args.accept_known_combined_reversal:
            raise RuntimeError(
                "保存轨迹整体折返超过0.30rad；如已在RViz确认，必须显式添加"
                "--accept-known-combined-reversal"
            )

    rclpy.init()
    node = pipeline.TaughtThenAnyGrasp()
    try:
        node.get_logger().warn(
            "REAL ARM MOTION: executing exact saved full trajectory; "
            "no ARM/DISARM; "
            + (
                "GRASP CYCLE requested with timed CLOSE and 5 cm lift"
                if args.grasp_cycle
                else "no RH56H1 or lift"
            )
        )
        node.validate_scene()
        measured = require_saved_start(node, trajectory)
        node.validate_path_shape(
            trajectory.joint_trajectory, "saved full continuous path"
        )
        node.validate_path_collisions(
            trajectory.joint_trajectory, measured, "保存完整轨迹"
        )
        timing = audit_relay_timing(trajectory)
        target = target_from_saved_metadata(metadata)
        if metadata.get("manual_teach_live_target_calibration_used", False):
            node.manual_teach_position_offset_xyz = (
                approach.MANUAL_TEACH_WORLD_POSITION_OFFSET_XYZ
            )
            node.manual_teach_orientation_xyzw = (
                approach.MANUAL_TEACH_TCP_QUATERNION_XYZW
            )
        frozen_live = live_target_from_saved_metadata(metadata)
        node.allow_verified_orientation_fallback = (
            metadata.get("orientation_consistency_mode")
            == "verified_reference_fallback"
        )
        validate_saved_target_current(
            node,
            target,
            args.accept_reviewed_target_drift,
            "预览前",
            frozen_live=frozen_live,
        )
        start_state = RobotState()
        start_state.joint_state = copy.deepcopy(measured)
        lift = None
        lift_metadata = None
        if args.grasp_cycle:
            lift, _, lift_metadata = plan_post_grasp_lift(
                node, measured, trajectory
            )
            publish_saved_grasp_cycle_preview(
                node, start_state, trajectory, lift
            )
        else:
            publish_segment_preview(
                node, start_state, trajectory, "已保存完整实机候选轨迹"
            )
        run_name = RUN_DIR.name
        phrase = (
            f"EXECUTE GRASP CYCLE {run_name} {digest[:12]}"
            if args.grasp_cycle
            else f"EXECUTE SAVED {run_name} {digest[:12]}"
        )
        typed = input(
            "在RViz审核全部轨迹、现场急停有人值守且外部ARM完成后，输入："
            f"{phrase}\n> "
        ).strip()
        if typed != phrase:
            raise RuntimeError("确认短语不匹配，拒绝执行")

        measured = require_saved_start(node, trajectory)
        node.validate_scene()
        node.validate_path_collisions(
            trajectory.joint_trajectory, measured, "执行前保存完整轨迹"
        )
        validate_saved_target_current(
            node,
            target,
            args.accept_reviewed_target_drift,
            "执行前",
            frozen_live=frozen_live,
        )
        gripper = None
        if args.grasp_cycle:
            gripper = RH56H1GripperStage(
                node, object_width_m=args.object_width_m
            )
            gripper.preopen()
        node.get_logger().warn(
            "KNOWN REVERSAL OVERRIDE ACCEPTED | "
            f"combined={reversal:.6f}rad, "
            f"expected_total={timing[2]:.3f}s, sha256={digest}"
        )
        node.execute_trajectory(
            trajectory,
            f"saved full run {run_name}",
            before_end_callback=(gripper.close if gripper is not None else None),
            motion_duration_sec=(timing[1] if gripper is not None else None),
            before_end_sec=GRASP_CLOSE_BEFORE_ENDPOINT_SEC,
        )
        node.check_measured_target(
            trajectory.joint_trajectory.points[-1].positions,
            pipeline.GRASP_ENDPOINT_TOLERANCE_RAD,
            "saved full endpoint acceptance",
        )
        if args.grasp_cycle:
            node.check_measured_target(
                lift.joint_trajectory.points[0].positions,
                pipeline.base.PRE_EXECUTE_START_TOLERANCE_RAD,
                "post-grasp lift start acceptance",
            )
            node.execute_trajectory(lift, "post-grasp 5 cm Cartesian lift")
            node.check_measured_target(
                lift.joint_trajectory.points[-1].positions,
                pipeline.GRASP_ENDPOINT_TOLERANCE_RAD,
                "post-grasp lift endpoint acceptance",
            )
            node.get_logger().info(
                "GRASP CYCLE COMPLETE | PREOPEN sent before motion, "
                f"CLOSE started {GRASP_CLOSE_BEFORE_ENDPOINT_SEC:.1f}s before "
                "the nominal main endpoint, 5 cm lift accepted; "
                "hand remains closed and no RELEASE was sent; "
                f"lift_sha256={lift_metadata['trajectory_sha256']}"
            )
        else:
            node.get_logger().info(
                "SAVED FULL TRAJECTORY COMPLETE; stop and inspect physically; "
                "no RH56H1 CLOSE or lift was sent"
            )
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def duration_sec(value):
    return float(value.sec) + float(value.nanosec) * 1e-9


def append_segment(destination, source, label):
    destination_jt = destination.joint_trajectory
    source_jt = source.joint_trajectory
    if list(destination_jt.joint_names) != pipeline.base.LEFT_ARM_JOINTS:
        raise RuntimeError("连续轨迹已有段关节顺序错误")
    if list(source_jt.joint_names) != pipeline.base.LEFT_ARM_JOINTS:
        raise RuntimeError(f"{label}关节顺序错误")
    if not destination_jt.points or not source_jt.points:
        raise RuntimeError(f"{label}或连续轨迹为空")
    offset = duration_sec(destination_jt.points[-1].time_from_start)
    for point in source_jt.points:
        appended = copy.deepcopy(point)
        appended.time_from_start = Duration(
            seconds=(
                offset + SEAM_DURATION_SEC + duration_sec(point.time_from_start)
            )
        ).to_msg()
        destination_jt.points.append(appended)


def slow_trajectory_for_first_real_execution(trajectory, factor):
    if factor < 1.0:
        raise RuntimeError("真机减速因子不能小于1.0")
    for point in trajectory.joint_trajectory.points:
        point.time_from_start = Duration(
            seconds=duration_sec(point.time_from_start) * factor
        ).to_msg()
        if point.velocities:
            point.velocities = [
                float(value) / factor for value in point.velocities
            ]
        if point.accelerations:
            point.accelerations = [
                float(value) / (factor * factor)
                for value in point.accelerations
            ]
    return trajectory


def audit_relay_timing(trajectory):
    points = trajectory.joint_trajectory.points
    maximum_speed = 0.0
    for previous, current in zip(points, points[1:]):
        dt = (
            duration_sec(current.time_from_start)
            - duration_sec(previous.time_from_start)
        ) * RELAY_TIME_SCALE
        if dt <= 0.0:
            raise RuntimeError("连续轨迹含非递增时间戳")
        maximum_speed = max(
            maximum_speed,
            max(
                abs(float(b) - float(a)) / dt
                for a, b in zip(previous.positions, current.positions)
            ),
        )
    input_duration = duration_sec(points[-1].time_from_start)
    after_relay = input_duration * RELAY_TIME_SCALE
    total = after_relay + RELAY_FINAL_SETTLE_SEC
    if maximum_speed > ONBOARD_MAX_SPEED_RAD_SEC:
        raise RuntimeError(
            "连续轨迹relay后速度超限："
            f"{maximum_speed:.6f}>{ONBOARD_MAX_SPEED_RAD_SEC:.6f} rad/s"
        )
    if total > ONBOARD_MAX_DURATION_SEC:
        raise RuntimeError(
            "连续轨迹预计总时长超过板载60秒限制："
            f"{total:.3f}s"
        )
    return input_duration, after_relay, total, maximum_speed


def publish_segment_preview(node, start_state, trajectory, label):
    display = DisplayTrajectory()
    display.model_id = "G1_D"
    display.trajectory_start = start_state
    display.trajectory.append(trajectory)
    node.display_pub.publish(display)
    node.get_logger().info(
        f"RViz已发布{label}：1个MoveIt轨迹段，"
        f"{len(trajectory.joint_trajectory.points)}点"
    )


def publish_two_segment_preview(
    node, start_state, taught_trajectory, grasp_trajectory
):
    """Show the taught and MoveIt grasp portions as two RViz segments."""
    display = DisplayTrajectory()
    display.model_id = "G1_D"
    display.trajectory_start = start_state
    display.trajectory.append(taught_trajectory)
    display.trajectory.append(grasp_trajectory)
    node.display_pub.publish(display)
    node.get_logger().info(
        "RViz已分段发布完整路径："
        f"第1段施教={len(taught_trajectory.joint_trajectory.points)}点，"
        f"第2段MoveIt AnyGrasp="
        f"{len(grasp_trajectory.joint_trajectory.points)}点"
    )


def save_full_artifact(
    trajectory,
    taught_metadata,
    target,
    roll,
    roll_relaxation,
    execution_slowdown,
    counts,
    timing,
    start_state,
    with_hand,
    orientation_consistency_mode,
    approach_reference_deviation_rad,
    approach_metadata,
):
    serialized = serialize_message(trajectory)
    start_map = dict(zip(
        start_state.joint_state.name, start_state.joint_state.position
    ))
    start_positions = [
        float(start_map[name]) for name in pipeline.base.LEFT_ARM_JOINTS
    ]
    p = target.pose.position
    q = target.pose.orientation
    thumb_bodyward, thumb_downward, thumb_right = (
        pipeline.left_thumb_direction_components(target)
    )
    metadata = {
        "schema_version": 1,
        "artifact_type": "g1_left_full_taught_to_anygrasp_continuous",
        "single_execute_trajectory_goal": True,
        "segment_order": [
            "measured_current_to_taught_start",
            "filtered_taught_motion",
            "nominal_taught_end_to_anygrasp_a",
        ],
        "joint_names": pipeline.base.LEFT_ARM_JOINTS,
        "waist_replay_disabled": True,
        "program_contains_arm_disarm_calls": False,
        "no_hand_command_sent": True,
        "hand_sequence_requested_for_execution": bool(with_hand),
        "generated_unix_time": time.time(),
        "point_count": len(trajectory.joint_trajectory.points),
        "segment_point_counts": counts,
        "initial_measured_positions_rad": start_positions,
        "initial_measured_sha256": pipeline.fingerprint(
            pipeline.base.LEFT_ARM_JOINTS, start_positions
        ),
        "taught_trajectory_sha256": taught_metadata["trajectory_sha256"],
        "taught_end_positions_rad": taught_metadata[
            "taught_end_positions_rad"
        ],
        "target_candidate": "A",
        "target_frame": target.header.frame_id,
        "target_position": [p.x, p.y, p.z],
        "target_quaternion_xyzw": [q.x, q.y, q.z, q.w],
        "target_roll_deg": roll,
        "target_roll_relaxation_deg": roll_relaxation,
        "left_thumb_direction": "bodyward_and_down",
        "left_thumb_bodyward_component": thumb_bodyward,
        "left_thumb_downward_component": thumb_downward,
        "left_thumb_robot_right_component_diagnostic": thumb_right,
        "grasp_goal_mode": "collision_free_ik_joint_goal",
        "first_real_execution_slowdown": execution_slowdown,
        "first_real_execution_speed_scale": 1.0 / execution_slowdown,
        "target_lift_m": approach_metadata["effective_target_lift_m"],
        "target_right_offset_m": approach_metadata[
            "effective_target_right_offset_m"
        ],
        "verified_orientation_reference_file": str(
            pipeline.VERIFIED_ORIENTATION_REFERENCE_FILE
        ),
        "orientation_consistency_mode": orientation_consistency_mode,
        "target_source_mode": approach_metadata["target_source_mode"],
        "planner_policy_version": approach_metadata[
            "planner_policy_version"
        ],
        "approach_reference_deviation_rad": (
            approach_reference_deviation_rad
        ),
        "manual_teach_orientation_used": approach_metadata[
            "manual_teach_orientation_used"
        ],
        "manual_teach_calibration_file": approach_metadata[
            "manual_teach_calibration_file"
        ],
        "manual_teach_live_target_calibration_used": approach_metadata[
            "manual_teach_live_target_calibration_used"
        ],
        "manual_teach_position_used": approach_metadata[
            "manual_teach_position_used"
        ],
        "live_anygrasp_position_before_manual_reference": approach_metadata[
            "live_anygrasp_position_before_manual_reference"
        ],
        "live_anygrasp_quaternion_before_manual_reference": approach_metadata[
            "live_anygrasp_quaternion_before_manual_reference"
        ],
        "preferred_reversal_detour_rad": approach_metadata[
            "preferred_reversal_detour_rad"
        ],
        "gross_reversal_limit_rad": approach_metadata[
            "gross_reversal_limit_rad"
        ],
        "final_approach_mode": approach_metadata["approach_mode"],
        "vertical_approach_clearance_m": approach_metadata[
            "vertical_approach_clearance_m"
        ],
        "cartesian_approach_step_m": approach_metadata[
            "cartesian_step_m"
        ],
        "cartesian_approach_fraction": approach_metadata[
            "cartesian_fraction"
        ],
        "overhead_segment_max_detour_rad": approach_metadata[
            "overhead_segment_max_detour_rad"
        ],
        "vertical_descent_max_detour_rad": approach_metadata[
            "vertical_descent_max_detour_rad"
        ],
        "combined_waypoint_reversal_rad": approach_metadata[
            "combined_waypoint_reversal_rad"
        ],
        "final_approach_wrist_yaw_joint": approach_metadata[
            "final_approach_wrist_yaw_joint"
        ],
        "final_approach_wrist_yaw_constraint_mode": approach_metadata[
            "final_approach_wrist_yaw_constraint_mode"
        ],
        "final_approach_wrist_yaw_limit_min_rad": approach_metadata[
            "final_approach_wrist_yaw_limit_min_rad"
        ],
        "final_approach_wrist_yaw_limit_max_rad": approach_metadata[
            "final_approach_wrist_yaw_limit_max_rad"
        ],
        "final_grasp_wrist_yaw_limit_min_rad": approach_metadata[
            "final_grasp_wrist_yaw_limit_min_rad"
        ],
        "final_grasp_wrist_yaw_limit_max_rad": approach_metadata[
            "final_grasp_wrist_yaw_limit_max_rad"
        ],
        "planned_final_grasp_wrist_yaw_rad": approach_metadata[
            "planned_final_grasp_wrist_yaw_rad"
        ],
        "planned_final_descent_wrist_yaw_min_rad": approach_metadata[
            "planned_final_descent_wrist_yaw_min_rad"
        ],
        "planned_final_descent_wrist_yaw_max_rad": approach_metadata[
            "planned_final_descent_wrist_yaw_max_rad"
        ],
        "input_trajectory_duration_sec": timing[0],
        "expected_after_relay_duration_sec": timing[1],
        "relay_final_settle_sec": RELAY_FINAL_SETTLE_SEC,
        "expected_total_after_relay_sec": timing[2],
        "maximum_speed_after_relay_rad_sec": timing[3],
        "pre_execute_start_tolerance_rad": (
            pipeline.base.PRE_EXECUTE_START_TOLERANCE_RAD
        ),
        "final_endpoint_tolerance_rad": (
            pipeline.GRASP_ENDPOINT_TOLERANCE_RAD
        ),
        "trajectory_sha256": hashlib.sha256(serialized).hexdigest(),
    }
    pipeline.base.atomic_write(FULL_TRAJECTORY_FILE, serialized, binary=True)
    pipeline.base.atomic_write(
        FULL_METADATA_FILE,
        json.dumps(metadata, ensure_ascii=False, indent=2) + "\n",
    )
    return metadata


def current_start_error(node, trajectory):
    measured = node.wait_fresh_state(clear=True)
    current = dict(zip(measured.name, measured.position))
    first = trajectory.joint_trajectory.points[0].positions
    errors = {
        name: abs(float(current[name]) - float(expected))
        for name, expected in zip(pipeline.base.LEFT_ARM_JOINTS, first)
    }
    worst_name = max(errors, key=errors.get)
    return errors[worst_name], worst_name


def save_integrated_visual_adjustment(
    trajectory,
    measured,
    actual_tcp,
    desired,
    micro_target,
    residual_m,
    residual_rad,
    maximum_translation_m,
    maximum_rotation_rad,
    metrics,
    timing,
):
    serialized = serialize_message(trajectory)
    digest = hashlib.sha256(serialized).hexdigest()
    current = dict(zip(measured.name, measured.position))
    metadata = {
        "schema_version": 1,
        "artifact_type": "g1_left_integrated_visual_micro_adjustment",
        "single_bounded_adjustment": True,
        "program_contains_arm_disarm_calls": False,
        "program_contains_hand_commands": False,
        "generated_unix_time": time.time(),
        "joint_names": pipeline.base.LEFT_ARM_JOINTS,
        "point_count": len(trajectory.joint_trajectory.points),
        "start_positions_rad": [
            float(current[name]) for name in pipeline.base.LEFT_ARM_JOINTS
        ],
        "actual_tcp_before_adjustment": visual_micro.pose_dict(actual_tcp),
        "fresh_visual_target": visual_micro.pose_dict(desired),
        "bounded_micro_target": visual_micro.pose_dict(micro_target),
        "full_position_residual_m": residual_m,
        "full_orientation_residual_rad": residual_rad,
        "maximum_translation_step_m": maximum_translation_m,
        "maximum_rotation_step_rad": maximum_rotation_rad,
        "maximum_joint_detour_rad": max(metrics[2].values()),
        "input_trajectory_duration_sec": timing[0],
        "expected_total_after_relay_sec": timing[2],
        "maximum_speed_after_relay_rad_sec": timing[3],
        "trajectory_sha256": digest,
    }
    pipeline.base.atomic_write(
        visual_micro.TRAJECTORY_FILE, serialized, binary=True
    )
    pipeline.base.atomic_write(
        visual_micro.METADATA_FILE,
        json.dumps(metadata, ensure_ascii=False, indent=2) + "\n",
    )
    return metadata


def execute_one_integrated_visual_adjustment(node, args):
    measured = node.wait_fresh_state(clear=True)
    visual_micro.require_near_last_full_endpoint(measured)
    fk_helper = EndpointDiagnostics(
        node,
        pipeline.base.LEFT_ARM_JOINTS,
        VISUAL_ENDPOINT_DIAGNOSTICS_FILE,
    )
    actual_tcp = fk_helper.compute_fk(measured)
    desired, _, _, _ = node.acquire_stable_target()
    current_visual_distance, _ = pipeline.pose_distance(actual_tcp, desired)
    if current_visual_distance > visual_micro.MAX_CURRENT_TCP_TO_VISUAL_TARGET_M:
        raise RuntimeError(
            "视觉闭环残差过大，禁止自动微调："
            f"{current_visual_distance:.6f}m > "
            f"{visual_micro.MAX_CURRENT_TCP_TO_VISUAL_TARGET_M:.6f}m"
        )

    maximum_translation_m = args.visual_adjust_mm / 1000.0
    maximum_rotation_rad = math.radians(args.visual_adjust_deg)
    micro_target, residual_m, residual_rad, _, _ = (
        visual_micro.bounded_target(
            actual_tcp,
            desired,
            maximum_translation_m,
            maximum_rotation_rad,
        )
    )
    pipeline.IK_ROLL_RELAXATIONS_DEG = (0.0,)
    pipeline.MIN_THUMB_BODYWARD_COMPONENT = -1.0
    pipeline.MIN_THUMB_DOWNWARD_COMPONENT = -1.0
    pipeline.GRASP_PLANNING_VELOCITY_SCALE = 0.02
    pipeline.GRASP_PLANNING_ACCELERATION_SCALE = 0.02
    trajectory, start_state, _, selected, roll_relaxation = (
        node.plan_grasp(measured, micro_target)
    )
    if abs(roll_relaxation) > 1e-9:
        raise RuntimeError("视觉微调不得改变目标滚转")
    trajectory = node.densify_trajectory(
        trajectory, "integrated visual micro-adjustment"
    )
    validate_final_approach_wrist_yaw(
        trajectory, "一体化视觉微调轨迹"
    )
    metrics = node.validate_path_shape(
        trajectory.joint_trajectory, "integrated visual micro-adjustment"
    )
    visual_micro.enforce_micro_path(metrics)
    node.validate_path_collisions(
        trajectory.joint_trajectory, measured, "一体化视觉微调轨迹"
    )
    timing = audit_relay_timing(trajectory)
    node.publish_grasp_preview(start_state, trajectory)
    adjustment_metadata = save_integrated_visual_adjustment(
        trajectory,
        measured,
        actual_tcp,
        desired,
        selected,
        residual_m,
        residual_rad,
        maximum_translation_m,
        maximum_rotation_rad,
        metrics,
        timing,
    )
    node.get_logger().warn(
        "INTEGRATED VISUAL ADJUSTMENT READY | "
        f"full_residual={residual_m:.6f}m/"
        f"{math.degrees(residual_rad):.3f}deg, "
        f"bounded_step={maximum_translation_m:.6f}m/"
        f"{args.visual_adjust_deg:.3f}deg, "
        f"points={adjustment_metadata['point_count']}"
    )

    error, joint = current_start_error(node, trajectory)
    if error > pipeline.base.PRE_EXECUTE_START_TOLERANCE_RAD:
        raise RuntimeError(
            "视觉微调规划后起点已改变："
            f"{joint}={error:.6f}rad"
        )
    node.validate_target_still_current(desired)
    node.execute_trajectory(
        trajectory, "one bounded integrated visual micro-adjustment"
    )
    report = EndpointDiagnostics(
        node,
        pipeline.base.LEFT_ARM_JOINTS,
        VISUAL_ENDPOINT_DIAGNOSTICS_FILE,
    ).collect(
        target_positions=trajectory.joint_trajectory.points[-1].positions,
        trajectory_file=visual_micro.TRAJECTORY_FILE,
        trajectory_sha256=adjustment_metadata["trajectory_sha256"],
        tolerance_rad=pipeline.GRASP_ENDPOINT_TOLERANCE_RAD,
        origin_label="integrated_visual_micro_adjustment_action_result",
        origin_monotonic=time.monotonic(),
        frozen_target=visual_micro.pose_dict(selected),
    )
    if not report["all_samples_within_tolerance"]:
        raise RuntimeError(
            "一轮视觉微调已执行，但关节终点仍未稳定进入0.015rad；"
            "诊断已保存，不发送CLOSE，不自动执行第二轮"
        )
    return report


def main():
    args = parse_args()
    if args.execute_saved:
        execute_saved_full(args)
        return
    taught, taught_metadata = pipeline.load_local_taught_artifacts()
    taught_start = [
        float(value)
        for value in taught_metadata["taught_start_positions_rad"]
    ]
    taught_end = [
        float(value)
        for value in taught_metadata["taught_end_positions_rad"]
    ]

    # These affect only this process.  The relay still slows every timestamp
    # by 5.40, and the final combined trajectory is audited against hardware
    # speed and duration limits before it can be executed.
    pipeline.base.VELOCITY_SCALE = MOVEIT_PLAN_VELOCITY_SCALE
    pipeline.base.ACCELERATION_SCALE = MOVEIT_PLAN_ACCELERATION_SCALE
    pipeline.GRASP_PLANNING_VELOCITY_SCALE = MOVEIT_PLAN_VELOCITY_SCALE
    pipeline.GRASP_PLANNING_ACCELERATION_SCALE = (
        MOVEIT_PLAN_ACCELERATION_SCALE
    )

    rclpy.init()
    node = pipeline.TaughtThenAnyGrasp()
    return_code = 2
    try:
        node.get_logger().warn(
            "PRIMARY CONTINUOUS PLAN: current -> taught -> AnyGrasp A; "
            "no ARM/DISARM; "
            + (
                "RH56H1 sequence requested only after execution confirmation"
                if args.with_hand
                else "no hand command"
            )
        )
        node.validate_scene()
        initial_measured = node.wait_fresh_state(clear=True)
        taught_segment = plan_left_taught_segment(
            node, taught, taught_metadata, initial_measured
        )
        # Show segment 1 before waiting for the live visual target.
        publish_segment_preview(
            node,
            taught_segment["start_state"],
            taught_segment["trajectory"],
            "第1段（当前位姿→施教轨迹终点）",
        )
        node.get_logger().info(
            "正在采集定位结果，第2段将以施教理论终点为起点"
            "由MoveIt规划到AnyGrasp A"
        )
        grasp_segment = plan_anygrasp_segment(
            node, initial_measured, taught_end
        )
        target = grasp_segment["target"]
        roll = grasp_segment["roll_deg"]
        combined = copy.deepcopy(taught_segment["trajectory"])
        append_segment(
            combined, grasp_segment["trajectory"], "AnyGrasp段"
        )
        execution_slowdown = (
            FIRST_REAL_EXECUTION_SLOWDOWN if args.execute else 1.0
        )
        if execution_slowdown > 1.0:
            slow_trajectory_for_first_real_execution(
                combined, execution_slowdown
            )
            node.get_logger().warn(
                "FIRST REAL EXECUTION SLOWDOWN APPLIED | "
                f"time_scale={execution_slowdown:.2f}, "
                f"speed_scale={1.0 / execution_slowdown:.3f}"
            )
        node.validate_path_shape(
            combined.joint_trajectory, "full continuous path"
        )
        node.validate_path_collisions(
            combined.joint_trajectory, initial_measured, "完整连续轨迹"
        )
        timing = audit_relay_timing(combined)
        publish_two_segment_preview(
            node,
            taught_segment["start_state"],
            taught_segment["trajectory"],
            grasp_segment["trajectory"],
        )
        counts = {
            "alignment": taught_segment["alignment_point_count"],
            "taught": taught_segment["taught_point_count"],
            "anygrasp": grasp_segment["point_count"],
        }
        metadata = save_full_artifact(
            combined,
            taught_metadata,
            target,
            roll,
            grasp_segment["roll_relaxation_deg"],
            execution_slowdown,
            counts,
            timing,
            taught_segment["start_state"],
            args.with_hand,
            grasp_segment["orientation_consistency_mode"],
            grasp_segment["approach_reference_deviation_rad"],
            grasp_segment,
        )
        node.get_logger().info(
            "FULL CONTINUOUS PLAN READY | "
            f"points={metadata['point_count']}, "
            f"expected_total={timing[2]:.3f}s, "
            f"max_speed={timing[3]:.6f}rad/s"
        )

        if not args.execute:
            node.get_logger().info(
                "PLAN ONLY COMPLETE: RViz contains taught + MoveIt AnyGrasp "
                "as two consecutive segments; "
                "no ExecuteTrajectory goal was sent"
            )
            return_code = 0
        else:
            node.get_logger().warn(
                "NON-INTERACTIVE REAL EXECUTION: --execute已授权本次执行；"
                "自动复核起点和AnyGrasp目标后立即发送轨迹"
            )
            error, joint = current_start_error(node, combined)
            if error > pipeline.base.PRE_EXECUTE_START_TOLERANCE_RAD:
                raise RuntimeError(
                    "执行前姿态已改变，必须重新规划："
                    f"{joint}={error:.6f} rad"
                )
            node.validate_target_still_current(target)
            gripper = None
            if args.with_hand:
                gripper = RH56H1GripperStage(
                    node, object_width_m=args.object_width_m
                )
                gripper.preopen()
            node.execute_trajectory(
                combined,
                "single current -> taught -> AnyGrasp trajectory",
            )
            action_result_monotonic = time.monotonic()
            diagnostics = EndpointDiagnostics(
                node,
                pipeline.base.LEFT_ARM_JOINTS,
                ENDPOINT_DIAGNOSTICS_FILE,
            ).collect(
                target_positions=(
                    combined.joint_trajectory.points[-1].positions
                ),
                trajectory_file=FULL_TRAJECTORY_FILE,
                trajectory_sha256=metadata["trajectory_sha256"],
                tolerance_rad=pipeline.GRASP_ENDPOINT_TOLERANCE_RAD,
                origin_label="execute_trajectory_action_result",
                origin_monotonic=action_result_monotonic,
                frozen_target={
                    "frame_id": metadata["target_frame"],
                    "position_m": metadata["target_position"],
                    "quaternion_xyzw": metadata[
                        "target_quaternion_xyzw"
                    ],
                    "orientation_consistency_mode": metadata[
                        "orientation_consistency_mode"
                    ],
                },
            )
            if not diagnostics["all_samples_within_tolerance"]:
                if not args.visual_adjust:
                    raise RuntimeError(
                        "final grasp-end arrival check未稳定进入容差："
                        f"t=0最大误差="
                        f"{diagnostics['maximum_error_initial_rad']:.6f}rad，"
                        f"t=5最大误差="
                        f"{diagnostics['maximum_error_final_rad']:.6f}rad，"
                        f"要求全部样本<="
                        f"{pipeline.GRASP_ENDPOINT_TOLERANCE_RAD:.6f}rad；"
                        "诊断JSON已保存，未发送CLOSE"
                    )
                node.get_logger().warn(
                    "PRIMARY ENDPOINT OUT OF TOLERANCE; "
                    "starting exactly one bounded visual adjustment"
                )
                execute_one_integrated_visual_adjustment(node, args)
            if gripper is not None:
                gripper.close()
            node.get_logger().info(
                "FULL TRAJECTORY SUCCESS; "
                + (
                    "RH56H1 CLOSE acknowledged; no lift sent"
                    if gripper is not None
                    else "arm-only; no RH56H1 command and no lift sent"
                )
            )
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
