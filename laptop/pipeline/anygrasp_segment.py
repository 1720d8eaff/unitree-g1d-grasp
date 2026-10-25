#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Reusable planner for segment 2: taught nominal end -> AnyGrasp A."""

import copy
import json
import math
import os

import execute_left_taught_then_anygrasp as pipeline
import rclpy
from g1d_project_paths import REFERENCE_DATA_DIR
from moveit_msgs.srv import GetCartesianPath


# Manual-teach calibration makes reversal a path-ranking metric.  Collision,
# robot joint limits and Cartesian completeness remain hard checks.  Only a
# grossly abnormal path is rejected here; RViz review remains mandatory.
PREFERRED_GRASP_SEGMENT_DETOUR_RAD = 0.40
MAX_GRASP_SEGMENT_DETOUR_RAD = 1.20
MAX_FRESH_TARGET_PLAN_ATTEMPTS = 1
MAX_OVERHEAD_PATH_ATTEMPTS = 3
VERTICAL_APPROACH_CLEARANCE_M = 0.05
CARTESIAN_PATH_SERVICE = "/compute_cartesian_path"
CARTESIAN_MAX_STEP_M = 0.003
CARTESIAN_MIN_FRACTION = 0.999
CARTESIAN_REVOLUTE_JUMP_THRESHOLD_RAD = 0.10
CARTESIAN_INPUT_MAX_JOINT_SPEED_RAD_SEC = 0.27
CARTESIAN_MIN_POINT_INTERVAL_SEC = 0.04

# The accepted 2026-09-09 manual endpoint is +1.0719 degrees and was stable to
# 0.0042 degrees over its final second.  This replaces the disproved -30..-10
# degree assumption.  The recorded manual motion reached -7.92..+24.97
# degrees, so the IK request, overhead endpoint and Cartesian descent use the
# rounded, operator-requested wider envelope -10..+30 degrees.
FINAL_APPROACH_WRIST_YAW_JOINT = "left_wrist_yaw_joint"
FINAL_APPROACH_WRIST_YAW_MIN_RAD = math.radians(-10.0)
FINAL_APPROACH_WRIST_YAW_MAX_RAD = math.radians(+30.0)
FINAL_GRASP_WRIST_YAW_MIN_RAD = math.radians(-5.0)
FINAL_GRASP_WRIST_YAW_MAX_RAD = math.radians(+5.0)
FINAL_APPROACH_WRIST_YAW_SEEDS_RAD = (
    math.radians(-8.0),
    math.radians(+1.0718536890438732),
    math.radians(+15.0),
    math.radians(+25.0),
)
FINAL_APPROACH_SHOULDER_PITCH_JOINT = "left_shoulder_pitch_joint"
FINAL_APPROACH_SHOULDER_PITCH_SEED_OFFSETS_RAD = (
    math.radians(-20.0),
    math.radians(+20.0),
)

MANUAL_TEACH_CALIBRATION_FILE = (
    REFERENCE_DATA_DIR / "left_grasp_manual_teach_calibration_20260909.json"
)
PLANNER_POLICY_VERSION = "0.3.13-live-position-taught-orientation"


def load_manual_teach_calibration():
    data = json.loads(MANUAL_TEACH_CALIBRATION_FILE.read_text(encoding="utf-8"))
    if data.get("artifact_type") != "g1_left_grasp_manual_teach_calibration":
        raise RuntimeError("第二段人工施教标定文件类型错误")
    if data.get("frame_id") != pipeline.TARGET_FRAME:
        raise RuntimeError("第二段人工施教标定坐标系错误")
    endpoint = [float(value) for value in data["endpoint_positions_rad"]]
    quaternion = [
        float(value) for value in data["endpoint_tcp_quaternion_xyzw"]
    ]
    if len(endpoint) != len(pipeline.base.LEFT_ARM_JOINTS):
        raise RuntimeError("第二段人工施教终点关节数量错误")
    if len(quaternion) != 4 or math.sqrt(sum(v*v for v in quaternion)) < 1e-12:
        raise RuntimeError("第二段人工施教TCP四元数无效")
    position = [
        float(value) for value in data["endpoint_tcp_position_m_diagnostic"]
    ]
    if len(position) != 3:
        raise RuntimeError("第二段人工施教TCP位置无效")
    source_position = [
        float(value) for value in data["source_anygrasp_position_m"]
    ]
    if len(source_position) != 3:
        raise RuntimeError("第二段施教源AnyGrasp位置无效")
    position_offset = tuple(
        target - source for source, target in zip(source_position, position)
    )
    recorded_position_offset = [
        float(value) for value in data["world_tcp_position_offset_m"]
    ]
    if max(
        abs(actual - expected)
        for actual, expected in zip(
            position_offset, recorded_position_offset
        )
    ) > 1e-9:
        raise RuntimeError("第二段人工施教TCP位置偏差校验失败")
    return (
        data,
        endpoint,
        position_offset,
        quaternion,
    )


(
    MANUAL_TEACH_CALIBRATION,
    MANUAL_TEACH_ENDPOINT_RAD,
    MANUAL_TEACH_WORLD_POSITION_OFFSET_XYZ,
    MANUAL_TEACH_TCP_QUATERNION_XYZW,
) = load_manual_teach_calibration()


PREVIOUS_PLANNER_PREVIEW = (
    os.environ.get("G1D_PREVIOUS_PLANNER_PREVIEW", "0") == "1"
)
USE_MANUAL_TEACH_LIVE_TARGET_CALIBRATION = not PREVIOUS_PLANNER_PREVIEW
MAX_OVERHEAD_IK_CANDIDATES = 1 if PREVIOUS_PLANNER_PREVIEW else 4
FINAL_APPROACH_IK_ROLL_RELAXATIONS_DEG = (
    pipeline.IK_ROLL_RELAXATIONS_DEG
    if PREVIOUS_PLANNER_PREVIEW
    else (0.0,)
)


def make_taught_end_joint_state(measured, taught_end):
    result = copy.deepcopy(measured)
    positions = list(result.position)
    index = {name: i for i, name in enumerate(result.name)}
    missing = [
        name for name in pipeline.base.LEFT_ARM_JOINTS if name not in index
    ]
    if missing:
        raise RuntimeError(f"实测状态缺少左臂关节：{missing}")
    for name, value in zip(pipeline.base.LEFT_ARM_JOINTS, taught_end):
        positions[index[name]] = float(value)
    result.position = positions
    return result


def enforce_grasp_detour(metrics):
    violations = {
        name: float(value)
        for name, value in metrics[2].items()
        if float(value) > MAX_GRASP_SEGMENT_DETOUR_RAD
    }
    if violations:
        name = max(violations, key=violations.get)
        raise RuntimeError(
            "AnyGrasp规划段绕行超限："
            f"{name}={violations[name]:.6f} rad > "
            f"{MAX_GRASP_SEGMENT_DETOUR_RAD:.6f} rad"
        )


def make_vertical_overhead_target(target):
    result = copy.deepcopy(target)
    result.pose.position.z += VERTICAL_APPROACH_CLEARANCE_M
    return result


def trajectory_detours(trajectory):
    points = trajectory.joint_trajectory.points
    result = {}
    for joint_index, name in enumerate(pipeline.base.LEFT_ARM_JOINTS):
        values = [float(point.positions[joint_index]) for point in points]
        variation = sum(abs(b-a) for a, b in zip(values, values[1:]))
        direct = abs(values[-1] - values[0])
        result[name] = max(0.0, variation - direct)
    return result


def wrist_yaw_from_joint_state(joint_state):
    values = dict(zip(joint_state.name, joint_state.position))
    if FINAL_APPROACH_WRIST_YAW_JOINT not in values:
        raise RuntimeError(
            f"IK解缺少关节：{FINAL_APPROACH_WRIST_YAW_JOINT}"
        )
    return float(values[FINAL_APPROACH_WRIST_YAW_JOINT])


def wrist_yaw_within_limit(value, minimum, maximum):
    value = float(value)
    return float(minimum) <= value <= float(maximum)


def make_wrist_yaw_ik_constraint(minimum, maximum, name):
    """Constrain MoveIt's IK solve, rather than filtering IK answers."""
    constraint = pipeline.Constraints()
    constraint.name = str(name)
    joint = pipeline.JointConstraint()
    joint.joint_name = FINAL_APPROACH_WRIST_YAW_JOINT
    joint.position = 0.0
    joint.tolerance_below = abs(float(minimum))
    joint.tolerance_above = float(maximum)
    joint.weight = 1.0
    constraint.joint_constraints.append(joint)
    return constraint


def joint_state_minimum_limit_margin(joint_state):
    values = dict(zip(joint_state.name, joint_state.position))
    missing = [
        name for name in pipeline.base.LEFT_ARM_JOINTS if name not in values
    ]
    if missing:
        raise RuntimeError(f"IK解缺少左臂关节：{missing}")
    margins = {}
    for name in pipeline.base.LEFT_ARM_JOINTS:
        lower, upper = pipeline.base.LEFT_ARM_LIMITS[name]
        value = float(values[name])
        margins[name] = min(value - lower, upper - value)
    worst_joint = min(margins, key=margins.get)
    return float(margins[worst_joint]), worst_joint


def validate_trajectory_joint_limit_margin(trajectory, label):
    joint_names = list(trajectory.joint_trajectory.joint_names)
    if joint_names != pipeline.base.LEFT_ARM_JOINTS:
        raise RuntimeError(f"{label}关节顺序错误")
    worst_margin = float("inf")
    worst_joint = None
    worst_point = None
    worst_value = None
    for point_index, point in enumerate(trajectory.joint_trajectory.points):
        for joint_index, name in enumerate(joint_names):
            lower, upper = pipeline.base.LEFT_ARM_LIMITS[name]
            value = float(point.positions[joint_index])
            margin = min(value - lower, upper - value)
            if margin < worst_margin:
                worst_margin = margin
                worst_joint = name
                worst_point = point_index
                worst_value = value
    if worst_margin < pipeline.base.JOINT_LIMIT_MARGIN_RAD:
        raise RuntimeError(
            f"{label}关节限位余量不足：point={worst_point}, "
            f"{worst_joint}={worst_value:+.6f}rad, "
            f"margin={worst_margin:.6f}rad < "
            f"{pipeline.base.JOINT_LIMIT_MARGIN_RAD:.6f}rad"
        )
    return worst_margin, worst_joint


def add_final_approach_wrist_yaw_seeds(seeds, measured):
    """Add bounded wrist-yaw and shoulder-pitch seeds for approach IK."""
    index = {name: i for i, name in enumerate(measured.name)}
    required = (
        FINAL_APPROACH_WRIST_YAW_JOINT,
        FINAL_APPROACH_SHOULDER_PITCH_JOINT,
    )
    missing = [name for name in required if name not in index]
    if missing:
        raise RuntimeError(f"IK种子缺少关节：{missing}")
    result = list(seeds)
    manual_seed = copy.deepcopy(measured)
    manual_positions = list(manual_seed.position)
    for name, value in zip(
        pipeline.base.LEFT_ARM_JOINTS, MANUAL_TEACH_ENDPOINT_RAD
    ):
        manual_positions[index[name]] = float(value)
    manual_seed.position = manual_positions
    result.append(manual_seed)
    wrist_index = index[FINAL_APPROACH_WRIST_YAW_JOINT]
    shoulder_index = index[FINAL_APPROACH_SHOULDER_PITCH_JOINT]
    shoulder_lower, shoulder_upper = pipeline.base.LEFT_ARM_LIMITS[
        FINAL_APPROACH_SHOULDER_PITCH_JOINT
    ]
    for value in FINAL_APPROACH_WRIST_YAW_SEEDS_RAD:
        seed = copy.deepcopy(measured)
        positions = list(seed.position)
        positions[wrist_index] = float(value)
        seed.position = positions
        result.append(seed)
        for offset in FINAL_APPROACH_SHOULDER_PITCH_SEED_OFFSETS_RAD:
            combined_seed = copy.deepcopy(seed)
            combined_positions = list(combined_seed.position)
            combined_positions[shoulder_index] = max(
                shoulder_lower + pipeline.base.JOINT_LIMIT_MARGIN_RAD,
                min(
                    shoulder_upper - pipeline.base.JOINT_LIMIT_MARGIN_RAD,
                    float(positions[shoulder_index]) + float(offset),
                ),
            )
            combined_seed.position = combined_positions
            result.append(combined_seed)
    return result


def validate_wrist_yaw_range(trajectory, label, minimum, maximum):
    joint_names = list(trajectory.joint_trajectory.joint_names)
    if FINAL_APPROACH_WRIST_YAW_JOINT not in joint_names:
        raise RuntimeError(
            f"{label}腕部限位检查失败：轨迹缺少"
            f"{FINAL_APPROACH_WRIST_YAW_JOINT}"
        )
    wrist_index = joint_names.index(FINAL_APPROACH_WRIST_YAW_JOINT)
    values = [
        float(point.positions[wrist_index])
        for point in trajectory.joint_trajectory.points
    ]
    violations = [
        (index, value)
        for index, value in enumerate(values)
        if not wrist_yaw_within_limit(value, minimum, maximum)
    ]
    if violations:
        point_index, value = max(
            violations,
            key=lambda item: max(
                float(minimum) - item[1],
                item[1] - float(maximum),
            ),
        )
        raise RuntimeError(
            f"{label}腕部限位失败："
            f"point={point_index}, "
            f"{FINAL_APPROACH_WRIST_YAW_JOINT}={value:.6f}rad，"
            f"要求[{float(minimum):.6f}, {float(maximum):.6f}]rad"
        )
    return min(values), max(values)


def validate_final_approach_wrist_yaw(trajectory, label):
    """Compatibility wrapper for approach and post-grasp lift checks."""
    return validate_wrist_yaw_range(
        trajectory,
        label,
        FINAL_APPROACH_WRIST_YAW_MIN_RAD,
        FINAL_APPROACH_WRIST_YAW_MAX_RAD,
    )


def validate_endpoint_wrist_yaw(trajectory, label, minimum, maximum):
    joint_names = list(trajectory.joint_trajectory.joint_names)
    if FINAL_APPROACH_WRIST_YAW_JOINT not in joint_names:
        raise RuntimeError(
            f"{label}腕部限位检查失败：轨迹缺少"
            f"{FINAL_APPROACH_WRIST_YAW_JOINT}"
        )
    points = trajectory.joint_trajectory.points
    if not points:
        raise RuntimeError(f"{label}腕部限位检查失败：轨迹为空")
    wrist_index = joint_names.index(FINAL_APPROACH_WRIST_YAW_JOINT)
    value = float(points[-1].positions[wrist_index])
    if not wrist_yaw_within_limit(value, minimum, maximum):
        raise RuntimeError(
            f"{label}终点腕部限位失败："
            f"{FINAL_APPROACH_WRIST_YAW_JOINT}={value:.6f}rad，"
            f"要求[{float(minimum):.6f}, {float(maximum):.6f}]rad"
        )
    return value


def collision_free_ik_candidates(
    node,
    measured,
    target,
    wrist_minimum,
    wrist_maximum,
    constraint_name,
    label,
    extra_seeds=(),
):
    node.grasp_ik_constraints = make_wrist_yaw_ik_constraint(
        wrist_minimum, wrist_maximum, constraint_name
    )
    seeds = add_final_approach_wrist_yaw_seeds(
        list(extra_seeds) + node.make_ik_seed_states(measured), measured
    )
    measured_values = dict(zip(measured.name, measured.position))
    for roll_offset in FINAL_APPROACH_IK_ROLL_RELAXATIONS_DEG:
        adjusted = pipeline.apply_local_x_roll(target, roll_offset)
        thumb_bodyward, thumb_downward, _ = (
            pipeline.left_thumb_direction_components(adjusted)
        )
        if (
            thumb_bodyward < pipeline.MIN_THUMB_BODYWARD_COMPONENT
            or thumb_downward < pipeline.MIN_THUMB_DOWNWARD_COMPONENT
        ):
            continue
        candidates = []
        for seed in seeds:
            code, solution = node.call_position_ik(
                seed, adjusted, avoid_collisions=True
            )
            if code != 1:
                continue
            values = dict(zip(solution.name, solution.position))
            if not all(
                name in values for name in pipeline.base.LEFT_ARM_JOINTS
            ):
                continue
            wrist_yaw = float(values[FINAL_APPROACH_WRIST_YAW_JOINT])
            if not wrist_yaw_within_limit(
                wrist_yaw, wrist_minimum, wrist_maximum
            ):
                raise RuntimeError(
                    "MoveIt返回的IK解违反已提交的腕部规划约束："
                    f"{wrist_yaw:+.6f}rad"
                )
            limit_margin, _ = joint_state_minimum_limit_margin(solution)
            if limit_margin < pipeline.base.JOINT_LIMIT_MARGIN_RAD:
                continue
            deltas = [
                abs(float(values[name]) - float(measured_values[name]))
                for name in pipeline.base.LEFT_ARM_JOINTS
            ]
            score = (max(deltas), sum(value*value for value in deltas))
            candidates.append((score, copy.deepcopy(solution)))
        if candidates:
            candidates.sort(key=lambda item: item[0])
            unique = []
            for score, solution in candidates:
                values = dict(zip(solution.name, solution.position))
                vector = tuple(
                    float(values[name])
                    for name in pipeline.base.LEFT_ARM_JOINTS
                )
                if any(
                    max(abs(a - b) for a, b in zip(vector, previous[2]))
                    < 1e-3
                    for previous in unique
                ):
                    continue
                unique.append((score, solution, vector))
                if len(unique) >= MAX_OVERHEAD_IK_CANDIDATES:
                    break
            node.get_logger().info(
                f"{label} IK CANDIDATES READY | "
                f"roll_relaxation={roll_offset:+.1f}deg, "
                f"raw_count={len(candidates)}, "
                f"unique_count={len(unique)}, "
                f"candidate_limit={MAX_OVERHEAD_IK_CANDIDATES}"
            )
            for index, (score, solution, _) in enumerate(unique, start=1):
                node.get_logger().info(
                    f"{label} IK CANDIDATE | "
                    f"index={index}/{len(unique)}, "
                    f"max_endpoint_delta={score[0]:.6f}rad, "
                    f"{FINAL_APPROACH_WRIST_YAW_JOINT}="
                    f"{wrist_yaw_from_joint_state(solution):+.6f}rad"
                )
            return (
                [(score, solution) for score, solution, _ in unique],
                adjusted,
                float(roll_offset),
            )
    raise RuntimeError(
        f"AnyGrasp抓取规划失败：{label}_IK_NO_SOLUTION "
        f"with {FINAL_APPROACH_WRIST_YAW_JOINT} in "
        f"[{float(wrist_minimum):.6f}, "
        f"{float(wrist_maximum):.6f}]rad"
    )


def parameterize_cartesian_trajectory(trajectory):
    result = copy.deepcopy(trajectory)
    points = result.joint_trajectory.points
    if len(points) < 2:
        raise RuntimeError("垂直接近Cartesian轨迹点数不足")
    elapsed = 0.0
    points[0].time_from_start = pipeline.base.Duration(seconds=0.0).to_msg()
    points[0].velocities = []
    points[0].accelerations = []
    for previous, current in zip(points, points[1:]):
        maximum_delta = max(
            abs(float(b) - float(a))
            for a, b in zip(previous.positions, current.positions)
        )
        interval = max(
            CARTESIAN_MIN_POINT_INTERVAL_SEC,
            maximum_delta / CARTESIAN_INPUT_MAX_JOINT_SPEED_RAD_SEC,
        )
        elapsed += interval
        current.time_from_start = pipeline.base.Duration(
            seconds=elapsed
        ).to_msg()
        current.velocities = []
        current.accelerations = []
    return result


def plan_vertical_descent(node, measured_context, overhead_trajectory, target):
    overhead_positions = overhead_trajectory.joint_trajectory.points[-1].positions
    cartesian_start = make_taught_end_joint_state(
        measured_context, overhead_positions
    )
    client = node.create_client(GetCartesianPath, CARTESIAN_PATH_SERVICE)
    if not client.wait_for_service(timeout_sec=5.0):
        raise RuntimeError(f"{CARTESIAN_PATH_SERVICE}不可用")
    request = GetCartesianPath.Request()
    request.header = copy.deepcopy(target.header)
    request.start_state.joint_state = copy.deepcopy(cartesian_start)
    request.start_state.is_diff = False
    request.group_name = pipeline.base.GROUP_NAME
    request.link_name = pipeline.EE_LINK
    request.waypoints = [copy.deepcopy(target.pose)]
    request.max_step = CARTESIAN_MAX_STEP_M
    request.jump_threshold = 0.0
    request.prismatic_jump_threshold = 0.0
    request.revolute_jump_threshold = CARTESIAN_REVOLUTE_JUMP_THRESHOLD_RAD
    request.avoid_collisions = True
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    response = future.result()
    if response is None:
        raise RuntimeError("垂直接近Cartesian规划未返回结果")
    if int(response.error_code.val) != 1:
        raise RuntimeError(
            "垂直接近Cartesian规划失败："
            f"error_code={response.error_code.val}, "
            f"fraction={response.fraction:.6f}"
        )
    if float(response.fraction) < CARTESIAN_MIN_FRACTION:
        raise RuntimeError(
            "垂直接近Cartesian路径不完整："
            f"fraction={response.fraction:.6f} < "
            f"{CARTESIAN_MIN_FRACTION:.6f}"
        )
    solution = response.solution
    if (
        list(solution.joint_trajectory.joint_names)
        != pipeline.base.LEFT_ARM_JOINTS
        or len(solution.joint_trajectory.points) < 2
    ):
        raise RuntimeError("垂直接近Cartesian轨迹关节顺序错误或点数不足")
    result = parameterize_cartesian_trajectory(solution)
    wrist_yaw_range = validate_wrist_yaw_range(
        result,
        "垂直接近Cartesian",
        FINAL_APPROACH_WRIST_YAW_MIN_RAD,
        FINAL_APPROACH_WRIST_YAW_MAX_RAD,
    )
    final_wrist_yaw = validate_endpoint_wrist_yaw(
        result,
        "最终抓取点",
        FINAL_GRASP_WRIST_YAW_MIN_RAD,
        FINAL_GRASP_WRIST_YAW_MAX_RAD,
    )
    return (
        result,
        float(response.fraction),
        wrist_yaw_range,
        final_wrist_yaw,
    )


def append_vertical_descent(overhead, descent):
    result = copy.deepcopy(overhead)
    destination = result.joint_trajectory
    source = descent.joint_trajectory
    offset = pipeline.base.duration_sec(destination.points[-1].time_from_start)
    for index, point in enumerate(source.points):
        # Cartesian response normally repeats its start state.  Skip it so the
        # seam remains strictly increasing without commanding a dwell.
        if index == 0:
            continue
        appended = copy.deepcopy(point)
        appended.time_from_start = pipeline.base.Duration(
            seconds=(
                offset
                + pipeline.base.SEAM_DURATION_SEC
                + pipeline.base.duration_sec(point.time_from_start)
            )
        ).to_msg()
        destination.points.append(appended)
    return result


def plan_low_detour_top_down(node, nominal_end, measured_context, target):
    # First prove the actual grasp target is reachable with the operator's
    # requested final -5..+5 degree wrist constraint.  These solutions then
    # seed the overhead solve so it stays on the same kinematic branch.
    final_ik_candidates, selected_target, roll_relaxation = (
        collision_free_ik_candidates(
            node,
            nominal_end,
            target,
            FINAL_GRASP_WRIST_YAW_MIN_RAD,
            FINAL_GRASP_WRIST_YAW_MAX_RAD,
            "manual_teach_final_wrist_yaw_minus5_plus5",
            "FINAL_GRASP",
        )
    )
    overhead_target = make_vertical_overhead_target(target)
    ik_candidates, selected_overhead, roll_relaxation = (
        collision_free_ik_candidates(
            node,
            nominal_end,
            overhead_target,
            FINAL_APPROACH_WRIST_YAW_MIN_RAD,
            FINAL_APPROACH_WRIST_YAW_MAX_RAD,
            "manual_teach_approach_wrist_yaw_minus10_plus30",
            "TOP_DOWN_OVERHEAD",
            extra_seeds=[item[1] for item in final_ik_candidates],
        )
    )
    selected_target = copy.deepcopy(selected_target)
    selected_target.pose.orientation = copy.deepcopy(
        selected_overhead.pose.orientation
    )

    best = None
    failures = []
    for candidate_index, (_, ik_solution) in enumerate(
        ik_candidates, start=1
    ):
        for attempt in range(1, MAX_OVERHEAD_PATH_ATTEMPTS + 1):
            try:
                (
                    overhead_trajectory,
                    start_state,
                    planning_time,
                    _,
                    _,
                ) = node.plan_grasp(
                    nominal_end,
                    selected_overhead,
                    precomputed_ik=ik_solution,
                    preselected_target=selected_overhead,
                    preselected_roll_relaxation=roll_relaxation,
                    planning_attempts=1,
                )
                validate_trajectory_joint_limit_margin(
                    overhead_trajectory, "上方接近转移段"
                )
                if not PREVIOUS_PLANNER_PREVIEW:
                    validate_endpoint_wrist_yaw(
                        overhead_trajectory,
                        "上方接近转移段",
                        FINAL_APPROACH_WRIST_YAW_MIN_RAD,
                        FINAL_APPROACH_WRIST_YAW_MAX_RAD,
                    )
                (
                    descent,
                    cartesian_fraction,
                    wrist_yaw_range,
                    final_wrist_yaw,
                ) = (
                    plan_vertical_descent(
                        node,
                        measured_context,
                        overhead_trajectory,
                        selected_target,
                    )
                )
                validate_trajectory_joint_limit_margin(
                    descent, "垂直接近Cartesian"
                )
                combined = append_vertical_descent(
                    overhead_trajectory, descent
                )
                overhead_detours = trajectory_detours(overhead_trajectory)
                descent_detours = trajectory_detours(descent)
                combined_detours = trajectory_detours(combined)
                overhead_joint = max(overhead_detours, key=overhead_detours.get)
                descent_joint = max(descent_detours, key=descent_detours.get)
                combined_joint = max(combined_detours, key=combined_detours.get)
                overhead_score = overhead_detours[overhead_joint]
                descent_score = descent_detours[descent_joint]
                stage_score = max(overhead_score, descent_score)
                combined_score = combined_detours[combined_joint]
                accepted_score = max(stage_score, combined_score)
                if best is None or accepted_score < best[0]:
                    best = (
                        accepted_score,
                        combined_score,
                        combined,
                        start_state,
                        planning_time,
                        selected_target,
                        roll_relaxation,
                        cartesian_fraction,
                        overhead_score,
                        descent_score,
                        wrist_yaw_range,
                        final_wrist_yaw,
                        candidate_index,
                        combined_joint,
                    )
                node.get_logger().info(
                    "TOP-DOWN OVERHEAD PATH CANDIDATE | "
                    f"ik_candidate={candidate_index}/{len(ik_candidates)}, "
                    f"attempt={attempt}/{MAX_OVERHEAD_PATH_ATTEMPTS}, "
                    f"overhead_detour={overhead_score:.6f}rad"
                    f"({overhead_joint}), "
                    f"descent_detour={descent_score:.6f}rad"
                    f"({descent_joint}), "
                    f"combined_waypoint_reversal={combined_score:.6f}rad"
                    f"({combined_joint})"
                )
            except RuntimeError as exc:
                failures.append(
                    f"ik_candidate={candidate_index}, attempt={attempt}: {exc}"
                )

    if best is None:
        raise RuntimeError(
            "AnyGrasp抓取规划失败：TOP_DOWN_PATH_FAILED "
            f"failures={failures}"
        )
    if best[0] > MAX_GRASP_SEGMENT_DETOUR_RAD:
        raise RuntimeError(
            "AnyGrasp抓取规划失败：TOP_DOWN_DETOUR "
            f"best_stage_or_combined={best[0]:.6f}rad, "
            f"combined_waypoint_reversal={best[1]:.6f}rad > "
            f"{MAX_GRASP_SEGMENT_DETOUR_RAD:.6f}rad"
        )
    node.get_logger().info(
        "TOP-DOWN IK PATH SELECTED | "
        f"ik_candidate={best[12]}/{len(ik_candidates)}, "
        f"accepted_score={best[0]:.6f}rad, "
        f"combined_limiting_joint={best[13]}, "
        f"preferred={PREFERRED_GRASP_SEGMENT_DETOUR_RAD:.6f}rad, "
        f"gross_limit={MAX_GRASP_SEGMENT_DETOUR_RAD:.6f}rad"
    )
    return (
        best[2],
        best[3],
        best[4],
        best[5],
        best[6],
        best[7],
        best[8],
        best[9],
        best[1],
        best[10],
        best[11],
    )


def plan_anygrasp_segment(node, measured_context, taught_end):
    # Current planning must follow the complete live candidate-A pose.  Only
    # the explicitly requested historical preview may restore an old absolute
    # orientation after an AnyGrasp branch change.
    node.allow_verified_orientation_fallback = PREVIOUS_PLANNER_PREVIEW
    node.manual_teach_position_offset_xyz = (
        MANUAL_TEACH_WORLD_POSITION_OFFSET_XYZ
        if USE_MANUAL_TEACH_LIVE_TARGET_CALIBRATION
        else None
    )
    node.manual_teach_orientation_xyzw = (
        MANUAL_TEACH_TCP_QUATERNION_XYZW
        if USE_MANUAL_TEACH_LIVE_TARGET_CALIBRATION
        else None
    )
    nominal_end = make_taught_end_joint_state(measured_context, taught_end)
    planning_failures = []
    for attempt in range(1, MAX_FRESH_TARGET_PLAN_ATTEMPTS + 1):
        if attempt > 1:
            node.get_logger().warn(
                f"重新采集AnyGrasp A并规划第2段："
                f"{attempt}/{MAX_FRESH_TARGET_PLAN_ATTEMPTS}"
            )
        target, roll, position_spread, orientation_spread = (
            node.acquire_stable_target()
        )
        try:
            (
                trajectory,
                start_state,
                planning_time,
                target,
                roll_relaxation,
                cartesian_fraction,
                overhead_detour,
                descent_detour,
                combined_waypoint_reversal,
                wrist_yaw_range,
                final_wrist_yaw,
            ) = plan_low_detour_top_down(
                node, nominal_end, measured_context, target
            )
            node.publish_planned_target_frames(target)
            node.get_logger().info(
                "TOP-DOWN APPROACH READY | "
                f"overhead_clearance={VERTICAL_APPROACH_CLEARANCE_M:.3f}m, "
                f"vertical_cartesian_step={CARTESIAN_MAX_STEP_M:.3f}m, "
                f"fraction={cartesian_fraction:.6f}, "
                f"{FINAL_APPROACH_WRIST_YAW_JOINT}_range="
                f"[{wrist_yaw_range[0]:+.6f}, "
                f"{wrist_yaw_range[1]:+.6f}]rad, "
                f"final={final_wrist_yaw:+.6f}rad"
            )
            roll += roll_relaxation
        except RuntimeError as exc:
            message = str(exc)
            if not (
                message.startswith("AnyGrasp抓取规划失败：")
                or message.startswith("垂直接近Cartesian")
                or message.startswith("AnyGrasp规划段绕行超限")
            ):
                raise
            planning_failures.append(message)
            node.get_logger().warn(
                "冻结的稳定AnyGrasp A无可行上方接近路径"
            )
            continue

        trajectory = node.densify_trajectory(
            trajectory, "AnyGrasp segment"
        )
        metrics = node.validate_path_shape(
            trajectory.joint_trajectory, "AnyGrasp segment"
        )
        node.get_logger().info(
            "TOP-DOWN COMPONENT DETOUR CHECK PASSED | "
            f"overhead={overhead_detour:.6f}rad, "
            f"descent={descent_detour:.6f}rad, "
            f"preferred={PREFERRED_GRASP_SEGMENT_DETOUR_RAD:.6f}rad, "
            f"gross_limit={MAX_GRASP_SEGMENT_DETOUR_RAD:.6f}rad, "
            f"combined_waypoint_reversal="
            f"{combined_waypoint_reversal:.6f}rad"
        )
        node.validate_path_collisions(
            trajectory.joint_trajectory, nominal_end, "AnyGrasp规划段"
        )
        if attempt > 1:
            node.get_logger().info(
                f"第{attempt}个稳定AnyGrasp A规划成功"
            )
        return {
            "trajectory": trajectory,
            "start_state": start_state,
            "target": target,
            "roll_deg": roll,
            "roll_relaxation_deg": roll_relaxation,
            "position_spread_m": position_spread,
            "orientation_spread_rad": orientation_spread,
            "orientation_consistency_mode": (
                node.last_orientation_consistency_mode
            ),
            "approach_reference_deviation_rad": (
                node.last_approach_reference_deviation_rad
            ),
            "planning_time_sec": planning_time,
            "metrics": metrics,
            "target_plan_attempt": attempt,
            "point_count": len(trajectory.joint_trajectory.points),
            "approach_mode": "overhead_then_vertical_cartesian_descent",
            "previous_planner_preview": PREVIOUS_PLANNER_PREVIEW,
            "strict_live_orientation": False,
            "target_source_mode": (
                "live_anygrasp_position_taught_orientation"
                if USE_MANUAL_TEACH_LIVE_TARGET_CALIBRATION
                else "legacy_live_anygrasp_preview"
            ),
            "planner_policy_version": PLANNER_POLICY_VERSION,
            "manual_teach_live_target_calibration_used": (
                USE_MANUAL_TEACH_LIVE_TARGET_CALIBRATION
            ),
            "manual_teach_calibration_file": str(
                MANUAL_TEACH_CALIBRATION_FILE
            ),
            "manual_teach_orientation_used": (
                USE_MANUAL_TEACH_LIVE_TARGET_CALIBRATION
            ),
            "manual_teach_position_used": False,
            "effective_target_right_offset_m": (
                0.0
                if USE_MANUAL_TEACH_LIVE_TARGET_CALIBRATION
                else pipeline.FINAL_TARGET_RIGHT_OFFSET_M
            ),
            "effective_target_lift_m": (
                0.0
                if USE_MANUAL_TEACH_LIVE_TARGET_CALIBRATION
                else pipeline.FINAL_TARGET_LIFT_M
            ),
            "live_anygrasp_position_before_manual_reference": [
                node.last_raw_anygrasp_reference.pose.position.x,
                node.last_raw_anygrasp_reference.pose.position.y,
                node.last_raw_anygrasp_reference.pose.position.z,
            ],
            "live_anygrasp_quaternion_before_manual_reference": [
                node.last_raw_anygrasp_reference.pose.orientation.x,
                node.last_raw_anygrasp_reference.pose.orientation.y,
                node.last_raw_anygrasp_reference.pose.orientation.z,
                node.last_raw_anygrasp_reference.pose.orientation.w,
            ],
            "preferred_reversal_detour_rad": (
                PREFERRED_GRASP_SEGMENT_DETOUR_RAD
            ),
            "gross_reversal_limit_rad": MAX_GRASP_SEGMENT_DETOUR_RAD,
            "combined_reversal_limit_enforced": True,
            "verified_orientation_fallback_allowed": (
                node.allow_verified_orientation_fallback
            ),
            "allowed_roll_relaxations_deg": list(
                FINAL_APPROACH_IK_ROLL_RELAXATIONS_DEG
            ),
            "vertical_approach_clearance_m": VERTICAL_APPROACH_CLEARANCE_M,
            "cartesian_step_m": CARTESIAN_MAX_STEP_M,
            "cartesian_fraction": cartesian_fraction,
            "overhead_segment_max_detour_rad": overhead_detour,
            "vertical_descent_max_detour_rad": descent_detour,
            "combined_waypoint_reversal_rad": (
                combined_waypoint_reversal
            ),
            "final_approach_wrist_yaw_joint": (
                FINAL_APPROACH_WRIST_YAW_JOINT
            ),
            "final_approach_wrist_yaw_constraint_mode": (
                "final_moveit_ik_minus5_plus5_approach_minus10_plus30"
            ),
            "final_approach_wrist_yaw_limit_min_rad": (
                FINAL_APPROACH_WRIST_YAW_MIN_RAD
            ),
            "final_approach_wrist_yaw_limit_max_rad": (
                FINAL_APPROACH_WRIST_YAW_MAX_RAD
            ),
            "final_grasp_wrist_yaw_limit_min_rad": (
                FINAL_GRASP_WRIST_YAW_MIN_RAD
            ),
            "final_grasp_wrist_yaw_limit_max_rad": (
                FINAL_GRASP_WRIST_YAW_MAX_RAD
            ),
            "planned_final_grasp_wrist_yaw_rad": final_wrist_yaw,
            "planned_final_descent_wrist_yaw_min_rad": wrist_yaw_range[0],
            "planned_final_descent_wrist_yaw_max_rad": wrist_yaw_range[1],
        }

    raise RuntimeError(
        "冻结的稳定AnyGrasp A目标无可行MoveIt路径："
        f"{planning_failures[-1]}"
    )
