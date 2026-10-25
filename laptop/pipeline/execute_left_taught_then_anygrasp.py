#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Replay the taught left-arm motion, then plan from its measured endpoint.

The existing taught-replay implementation remains the source of truth for the
first stage.  After that action finishes, this process samples fresh joint
states, freezes a fresh AnyGrasp candidate-A target, and plans a new grasp
trajectory from the measured (not nominal) taught endpoint.

Safety defaults:
  * no automatic ARM/DISARM
  * no RH56H1 commands
  * no real motion unless --execute-taught is supplied
  * grasp motion also requires --execute-grasp and an interactive phrase
"""

import argparse
import copy
import hashlib
import importlib.util
import json
import math
import statistics
import time
from pathlib import Path

from g1d_project_paths import (
    REFERENCE_DATA_DIR,
    RUN_DIR,
    TAUGHT_DATA_DIR,
    ensure_run_dir,
)


HERE = Path(__file__).resolve().parent
BASE_PATH = HERE / "execute_left_taught_with_auto_start.py"
TAUGHT_TRAJECTORY_FILE = TAUGHT_DATA_DIR / "g1_left_taught_motion.cdr"
TAUGHT_METADATA_FILE = TAUGHT_DATA_DIR / "g1_left_taught_motion.json"
ensure_run_dir()
GRASP_TRAJECTORY_FILE = RUN_DIR / "g1_left_measured_taught_end_to_grasp.cdr"
GRASP_METADATA_FILE = RUN_DIR / "g1_left_measured_taught_end_to_grasp.json"
VERIFIED_ORIENTATION_REFERENCE_FILE = (
    REFERENCE_DATA_DIR / "left_grasp_verified_orientation_reference.json"
)


def load_base_module():
    spec = importlib.util.spec_from_file_location("g1_taught_replay_base", BASE_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"无法加载现有示教执行器：{BASE_PATH}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


base = load_base_module()

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    DisplayTrajectory,
    JointConstraint,
    OrientationConstraint,
    PlanningOptions,
    PositionConstraint,
    RobotState,
    RobotTrajectory,
)
from moveit_msgs.srv import GetPositionIK, GetStateValidity
from rclpy.action import ActionClient
from rclpy.serialization import serialize_message
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from tf2_ros import StaticTransformBroadcaster


TARGET_TOPIC = "/anygrasp/left_target_candidate_a"
EE_LINK = "left_grasp_tcp"
TARGET_FRAME = "torso_link"
# Empirical TCP-to-object calibration from the first arm-only execution.
# In torso_link, robot right is -Y and up is +Z.  The previous target already
# included +0.01 m lift; the operator requested another +0.01 m.
FINAL_TARGET_RIGHT_OFFSET_M = 0.03
FINAL_TARGET_LIFT_M = 0.02
MIN_DOWNWARD_APPROACH_COMPONENT = 0.35
POSITION_TOLERANCE_M = 0.008
ORIENTATION_TOLERANCE_RAD = 0.10
GRASP_ENDPOINT_TOLERANCE_RAD = 0.015
GRASP_PLANNING_VELOCITY_SCALE = 0.05
GRASP_PLANNING_ACCELERATION_SCALE = 0.05
GRASP_ALLOWED_PLANNING_TIME_SEC = 8.0
IK_SERVICE = "/compute_ik"
IK_TIMEOUT_SEC = 0.35
IK_CALL_TIMEOUT_SEC = 2.0
IK_JOINT_GOAL_TOLERANCE_RAD = 0.003
IK_ROLL_RELAXATIONS_DEG = (
    0.0, +10.0, -10.0, +20.0, -20.0,
    +30.0, -30.0, +40.0, -40.0, +45.0, -45.0,
)
MIN_THUMB_BODYWARD_COMPONENT = 0.15
MIN_THUMB_DOWNWARD_COMPONENT = 0.15
STABILITY_SAMPLE_SEC = 1.5
STABILITY_RANGE_RAD = 0.003
TARGET_SAMPLE_COUNT = 10
# Keep the same 10-frame stability gates, but allow camera/AnyGrasp filtering
# to settle before rejecting the scene.  This changes waiting time only; it
# does not relax position, orientation, or center-drift thresholds.
TARGET_ACQUIRE_TIMEOUT_SEC = 30.0
TARGET_POSITION_SPREAD_M = 0.005
TARGET_ORIENTATION_SPREAD_RAD = 0.08
TARGET_FILTERED_POSITION_SPREAD_M = 0.008
TARGET_FILTERED_ORIENTATION_SPREAD_RAD = 0.12
TARGET_CENTER_DRIFT_M = 0.0035
TARGET_CENTER_DRIFT_RAD = 0.05
PRE_EXECUTE_TARGET_SHIFT_M = 0.008
PRE_EXECUTE_TARGET_SHIFT_RAD = 0.12
RESAMPLED_MAX_POINT_STEP_RAD = 0.08


def load_verified_orientation_reference():
    data = json.loads(
        VERIFIED_ORIENTATION_REFERENCE_FILE.read_text(encoding="utf-8")
    )
    if data.get("artifact_type") != (
        "g1_left_grasp_verified_orientation_reference"
    ):
        raise RuntimeError("抓取姿态参考文件类型错误")
    if data.get("frame_id") != TARGET_FRAME:
        raise RuntimeError("抓取姿态参考坐标系不是torso_link")
    values = [float(value) for value in data["quaternion_xyzw"]]
    if len(values) != 4:
        raise RuntimeError("抓取姿态参考四元数长度错误")
    norm = math.sqrt(sum(value * value for value in values))
    if norm < 1e-12:
        raise RuntimeError("抓取姿态参考四元数为零")
    pose = PoseStamped()
    pose.header.frame_id = TARGET_FRAME
    pose.pose.orientation.x = values[0] / norm
    pose.pose.orientation.y = values[1] / norm
    pose.pose.orientation.z = values[2] / norm
    pose.pose.orientation.w = values[3] / norm
    maximum_deviation = float(data["maximum_live_approach_deviation_rad"])
    if not 0.0 < maximum_deviation < math.pi:
        raise RuntimeError("抓取姿态参考接近轴阈值无效")
    return pose, maximum_deviation


def parse_args():
    parser = argparse.ArgumentParser(
        description="示教连续重放后，从实测终点规划到最新 AnyGrasp 抓取位姿"
    )
    parser.add_argument(
        "--execute-taught",
        action="store_true",
        help="执行示教阶段；否则只预览当前姿态到示教动作",
    )
    parser.add_argument(
        "--execute-grasp",
        action="store_true",
        help="示教后执行抓取轨迹；还需要现场交互确认",
    )
    args = parser.parse_args()
    if args.execute_grasp and not args.execute_taught:
        parser.error("--execute-grasp 必须与 --execute-taught 同时使用")
    return args


def quaternion_angle(a, b):
    dot = abs(float(a.x) * float(b.x) + float(a.y) * float(b.y)
              + float(a.z) * float(b.z) + float(a.w) * float(b.w))
    na = math.sqrt(a.x*a.x + a.y*a.y + a.z*a.z + a.w*a.w)
    nb = math.sqrt(b.x*b.x + b.y*b.y + b.z*b.z + b.w*b.w)
    if na < 1e-12 or nb < 1e-12:
        return float("inf")
    return 2.0 * math.acos(max(-1.0, min(1.0, dot / (na * nb))))


def pose_distance(a, b):
    pa, pb = a.pose.position, b.pose.position
    distance = math.sqrt((pa.x-pb.x)**2 + (pa.y-pb.y)**2 + (pa.z-pb.z)**2)
    return distance, quaternion_angle(a.pose.orientation, b.pose.orientation)


def approach_axis_angle(a, b):
    """Angular change of TCP +X, independent of hand roll about +X."""
    axes = []
    for stamped in (a, b):
        q = stamped.pose.orientation
        norm_sq = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w
        if norm_sq < 1e-12:
            return float("inf")
        axes.append((
            (1.0 - 2.0 * (q.y*q.y + q.z*q.z) / norm_sq),
            (2.0 * (q.x*q.y + q.z*q.w) / norm_sq),
            (2.0 * (q.x*q.z - q.y*q.w) / norm_sq),
        ))
    dot = sum(x * y for x, y in zip(axes[0], axes[1]))
    return math.acos(max(-1.0, min(1.0, dot)))


def average_pose_samples(samples):
    """Return a sign-aligned mean pose for one candidate-A sample window."""
    result = copy.deepcopy(samples[-1])
    count = float(len(samples))
    result.pose.position.x = sum(
        float(item.pose.position.x) for item in samples
    ) / count
    result.pose.position.y = sum(
        float(item.pose.position.y) for item in samples
    ) / count
    result.pose.position.z = sum(
        float(item.pose.position.z) for item in samples
    ) / count

    reference = samples[-1].pose.orientation
    accumulated = [0.0, 0.0, 0.0, 0.0]
    for item in samples:
        q = item.pose.orientation
        values = [float(q.x), float(q.y), float(q.z), float(q.w)]
        dot = (
            values[0] * float(reference.x)
            + values[1] * float(reference.y)
            + values[2] * float(reference.z)
            + values[3] * float(reference.w)
        )
        if dot < 0.0:
            values = [-value for value in values]
        accumulated = [a + b for a, b in zip(accumulated, values)]
    norm = math.sqrt(sum(value * value for value in accumulated))
    if norm < 1e-12:
        raise RuntimeError("AnyGrasp窗口平均四元数无效")
    result.pose.orientation.x = accumulated[0] / norm
    result.pose.orientation.y = accumulated[1] / norm
    result.pose.orientation.z = accumulated[2] / norm
    result.pose.orientation.w = accumulated[3] / norm
    return result


def normalize_angle_deg(angle):
    result = float(angle)
    while result >= 180.0:
        result -= 360.0
    while result < -180.0:
        result += 360.0
    return result


def bodyward_down_thumb_roll_deg(pose):
    """Roll TCP -Y closest to torso [-X, 0, -Z].

    ``left_grasp_tcp`` shares the palm axes and the RH56H1 left thumb points
    along TCP -Y.  Torso +X points away from the body and torso +Z points up,
    so the operator-confirmed thumb direction is proportional to [-1, 0, -1].
    Projecting that vector into the plane normal to TCP +X gives the closest
    direction reachable using only a local TCP-X roll.
    """
    q = pose.pose.orientation
    norm_sq = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w
    if norm_sq < 1e-12:
        raise RuntimeError("AnyGrasp目标四元数为零")

    # Local Y and Z axes in torso coordinates (columns 1 and 2 of R).
    local_y = (
        2.0 * (q.x*q.y - q.z*q.w) / norm_sq,
        1.0 - 2.0 * (q.x*q.x + q.z*q.z) / norm_sq,
        2.0 * (q.y*q.z + q.x*q.w) / norm_sq,
    )
    local_z = (
        2.0 * (q.x*q.z + q.y*q.w) / norm_sq,
        2.0 * (q.y*q.z - q.x*q.w) / norm_sq,
        1.0 - 2.0 * (q.x*q.x + q.y*q.y) / norm_sq,
    )
    desired = (-math.sqrt(0.5), 0.0, -math.sqrt(0.5))
    cosine_score = sum(
        d * (-axis) for d, axis in zip(desired, local_y)
    )
    sine_score = sum(
        d * (-axis) for d, axis in zip(desired, local_z)
    )
    if math.hypot(cosine_score, sine_score) < 1e-9:
        raise RuntimeError("AnyGrasp接近轴与目标拇指方向退化")
    return normalize_angle_deg(
        math.degrees(math.atan2(sine_score, cosine_score))
    )


def left_thumb_direction_components(pose):
    """Return bodyward, downward and robot-right projections of TCP -Y."""
    q = pose.pose.orientation
    norm_sq = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w
    if norm_sq < 1e-12:
        raise RuntimeError("AnyGrasp目标四元数为零")
    local_y_torso_x = 2.0 * (q.x*q.y - q.z*q.w) / norm_sq
    local_y_torso_y = 1.0 - 2.0 * (q.x*q.x + q.z*q.z) / norm_sq
    local_y_torso_z = 2.0 * (q.y*q.z + q.x*q.w) / norm_sq
    # Thumb is TCP -Y. Bodyward/right/down are torso -X/-Y/-Z.
    return (
        local_y_torso_x,
        local_y_torso_z,
        local_y_torso_y,
    )


def apply_local_x_roll(pose, roll_deg):
    result = copy.deepcopy(pose)
    q = result.pose.orientation
    norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
    if norm < 1e-12:
        raise RuntimeError("AnyGrasp目标四元数为零")
    x, y, z, w = q.x/norm, q.y/norm, q.z/norm, q.w/norm
    half = math.radians(float(roll_deg)) * 0.5
    s, c = math.sin(half), math.cos(half)
    q.x = w*s + x*c
    q.y = y*c + z*s
    q.z = z*c - y*s
    q.w = w*c - x*s
    return result


def normalized_quaternion_xyzw(values, label):
    values = [float(value) for value in values]
    if len(values) != 4:
        raise RuntimeError(f"{label}四元数长度错误")
    norm = math.sqrt(sum(value * value for value in values))
    if norm < 1e-12:
        raise RuntimeError(f"{label}四元数为零")
    return tuple(value / norm for value in values)


def apply_final_target_translation(pose):
    """Apply the operator-measured right/up TCP calibration in torso_link."""
    result = copy.deepcopy(pose)
    result.pose.position.y -= FINAL_TARGET_RIGHT_OFFSET_M
    result.pose.position.z += FINAL_TARGET_LIFT_M
    return result


def fingerprint(names, positions):
    payload = {
        "joint_names": list(names),
        "positions_rad": [round(float(value), 8) for value in positions],
    }
    return hashlib.sha256(
        json.dumps(payload, sort_keys=True, separators=(",", ":")).encode()
    ).hexdigest()


def load_local_taught_artifacts():
    trajectory_bytes = TAUGHT_TRAJECTORY_FILE.read_bytes()
    metadata = json.loads(TAUGHT_METADATA_FILE.read_text(encoding="utf-8"))
    if metadata.get("artifact_type") != "g1_left_taught_motion":
        raise RuntimeError("本地示教轨迹元数据类型不匹配")
    if hashlib.sha256(trajectory_bytes).hexdigest() != metadata.get("trajectory_sha256"):
        raise RuntimeError("本地示教轨迹SHA-256校验失败")
    trajectory = base.deserialize_message(trajectory_bytes, base.JointTrajectory)
    if list(trajectory.joint_names) != base.LEFT_ARM_JOINTS:
        raise RuntimeError("本地示教轨迹不是严格的左臂7关节轨迹")
    if len(trajectory.points) != int(metadata.get("point_count", -1)):
        raise RuntimeError("本地示教轨迹点数与元数据不匹配")
    if not metadata.get("waist_replay_disabled", False):
        raise RuntimeError("本地示教元数据没有明确禁止腰部重放")
    return trajectory, metadata


class TaughtThenAnyGrasp(base.TaughtReplay):
    def __init__(self):
        self._collect_joint_samples = False
        self._joint_samples = []
        self._collect_target_samples = False
        self._target_samples = []
        self.latest_anygrasp = None
        self.latest_anygrasp_monotonic = None
        super().__init__()
        (
            self.verified_orientation_reference,
            self.maximum_live_approach_deviation_rad,
        ) = load_verified_orientation_reference()
        self.last_orientation_consistency_mode = None
        self.last_approach_reference_deviation_rad = None
        # The consolidated second-stage planner may set this from a reviewed
        # manual-teach calibration.  Live AnyGrasp supplies the position;
        # teaching supplies a measured position bias and the accepted palm
        # orientation.  An absolute torso_link target must never replace live
        # detection.
        self.manual_teach_position_offset_xyz = None
        self.manual_teach_orientation_xyzw = None
        self.grasp_ik_constraints = None
        self.last_raw_anygrasp_reference = None
        # The legacy standalone planner may still restore the operator-verified
        # orientation after an AnyGrasp branch jump.  The consolidated planner
        # explicitly disables this so its preview always represents the live
        # target that was actually observed.
        self.allow_verified_orientation_fallback = True
        self.target_sub = self.create_subscription(
            PoseStamped, TARGET_TOPIC, self.on_anygrasp, 20
        )
        self.ik_client = self.create_client(GetPositionIK, IK_SERVICE)
        self.planned_target_tf = StaticTransformBroadcaster(self)

    def apply_verified_orientation_continuity(self, live_pose):
        """Keep live position, suppress a different AnyGrasp grasp branch.

        The operator confirmed that the robot and object are unchanged.  A
        live approach axis within 20 degrees remains fully live.  A larger
        jump is treated as an upstream AnyGrasp branch switch: only the
        verified orientation is restored, while the current measured object
        position remains untouched.
        """
        deviation = approach_axis_angle(
            self.verified_orientation_reference, live_pose
        )
        result = copy.deepcopy(live_pose)
        if deviation <= self.maximum_live_approach_deviation_rad:
            mode = "live_consistent"
        elif not self.allow_verified_orientation_fallback:
            mode = "live_strict_no_fallback"
            self.get_logger().warn(
                "ANYGRASP APPROACH BRANCH DEVIATION KEPT LIVE | "
                f"deviation={deviation:.6f}rad > "
                f"{self.maximum_live_approach_deviation_rad:.6f}rad; "
                "strict consolidated planning forbids verified-orientation "
                "fallback"
            )
        else:
            mode = "verified_reference_fallback"
            result.pose.orientation = copy.deepcopy(
                self.verified_orientation_reference.pose.orientation
            )
            self.get_logger().warn(
                "ANYGRASP APPROACH BRANCH JUMP SUPPRESSED | "
                f"deviation={deviation:.6f}rad > "
                f"{self.maximum_live_approach_deviation_rad:.6f}rad; "
                "position remains live, orientation uses verified reference"
            )
        self.last_orientation_consistency_mode = mode
        self.last_approach_reference_deviation_rad = deviation
        return result, mode, deviation

    def on_joint_state(self, message):
        super().on_joint_state(message)
        if self._collect_joint_samples:
            self._joint_samples.append(copy.deepcopy(message))

    def on_anygrasp(self, message):
        self.latest_anygrasp = copy.deepcopy(message)
        self.latest_anygrasp_monotonic = time.monotonic()
        if self._collect_target_samples:
            self._target_samples.append(copy.deepcopy(message))

    def sample_stable_endpoint(self):
        self._joint_samples = []
        self._collect_joint_samples = True
        deadline = time.monotonic() + STABILITY_SAMPLE_SEC
        try:
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.02)
        finally:
            self._collect_joint_samples = False
        if len(self._joint_samples) < 10:
            raise RuntimeError("示教结束后完整关节样本不足")

        rows = []
        for message in self._joint_samples:
            current = dict(zip(message.name, message.position))
            if all(name in current for name in base.LEFT_ARM_JOINTS):
                rows.append([float(current[name]) for name in base.LEFT_ARM_JOINTS])
        if len(rows) < 10:
            raise RuntimeError("示教结束后的样本缺少左臂7关节")

        ranges = [max(row[i] for row in rows) - min(row[i] for row in rows)
                  for i in range(len(base.LEFT_ARM_JOINTS))]
        worst_index = max(range(len(ranges)), key=ranges.__getitem__)
        if ranges[worst_index] > STABILITY_RANGE_RAD:
            raise RuntimeError(
                "示教终点仍在漂移："
                f"{base.LEFT_ARM_JOINTS[worst_index]} range="
                f"{ranges[worst_index]:.6f} rad"
            )

        result = copy.deepcopy(self._joint_samples[-1])
        position_map = dict(zip(result.name, result.position))
        medians = [statistics.median(row[i] for row in rows)
                   for i in range(len(base.LEFT_ARM_JOINTS))]
        result.position = [
            medians[base.LEFT_ARM_JOINTS.index(name)]
            if name in base.LEFT_ARM_JOINTS else float(position_map[name])
            for name in result.name
        ]
        self.get_logger().info(
            "MEASURED TAUGHT END STABLE | samples="
            f"{len(rows)}, worst_range={ranges[worst_index]:.6f} rad, "
            f"joint={base.LEFT_ARM_JOINTS[worst_index]}"
        )
        return result, ranges

    def densify_trajectory(self, trajectory, label):
        """Insert time-interpolated points without changing the planned path."""
        result = copy.deepcopy(trajectory)
        points = result.joint_trajectory.points
        if len(points) < 2:
            raise RuntimeError(f"{label}轨迹点数不足")
        dense = [copy.deepcopy(points[0])]
        inserted = 0
        for previous, current in zip(points, points[1:]):
            maximum_delta = max(
                abs(float(b) - float(a))
                for a, b in zip(previous.positions, current.positions)
            )
            pieces = max(1, int(math.ceil(
                maximum_delta / RESAMPLED_MAX_POINT_STEP_RAD
            )))
            previous_time = base.duration_sec(previous.time_from_start)
            current_time = base.duration_sec(current.time_from_start)
            if current_time <= previous_time:
                raise RuntimeError(f"{label}时间戳未严格递增")
            for piece in range(1, pieces + 1):
                ratio = float(piece) / float(pieces)
                point = copy.deepcopy(current)
                point.positions = [
                    float(a) + ratio * (float(b) - float(a))
                    for a, b in zip(previous.positions, current.positions)
                ]
                for field in ("velocities", "accelerations", "effort"):
                    before = list(getattr(previous, field))
                    after = list(getattr(current, field))
                    if before and len(before) == len(after):
                        setattr(point, field, [
                            float(a) + ratio * (float(b) - float(a))
                            for a, b in zip(before, after)
                        ])
                point.time_from_start = base.Duration(
                    seconds=previous_time + ratio * (current_time - previous_time)
                ).to_msg()
                dense.append(point)
                if piece < pieces:
                    inserted += 1
        result.joint_trajectory.points = dense
        self.get_logger().info(
            f"{label}重采样完成：{len(points)}->{len(dense)}点，"
            f"新增{inserted}点，目标点间变化<={RESAMPLED_MAX_POINT_STEP_RAD:.3f}rad"
        )
        return result

    def acquire_stable_target(self):
        self._target_samples = []
        self._collect_target_samples = True
        deadline = time.monotonic() + TARGET_ACQUIRE_TIMEOUT_SEC
        accepted_samples = None
        accepted_mode = None
        best_position_spread = float("inf")
        best_orientation_spread = float("inf")
        best_center_position_drift = float("inf")
        best_center_orientation_drift = float("inf")
        try:
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
                valid = [m for m in self._target_samples if m.header.frame_id == TARGET_FRAME]
                if len(valid) < TARGET_SAMPLE_COUNT:
                    continue
                candidate = valid[-TARGET_SAMPLE_COUNT:]
                reference = average_pose_samples(candidate)
                position_spread = max(
                    pose_distance(reference, item)[0] for item in candidate
                )
                orientation_spread = max(
                    pose_distance(reference, item)[1] for item in candidate
                )
                half = TARGET_SAMPLE_COUNT // 2
                early_center = average_pose_samples(candidate[:half])
                late_center = average_pose_samples(candidate[half:])
                center_position_drift, center_orientation_drift = (
                    pose_distance(early_center, late_center)
                )
                score = max(
                    position_spread / TARGET_POSITION_SPREAD_M,
                    orientation_spread / TARGET_ORIENTATION_SPREAD_RAD,
                )
                best_score = max(
                    best_position_spread / TARGET_POSITION_SPREAD_M,
                    best_orientation_spread / TARGET_ORIENTATION_SPREAD_RAD,
                )
                if score < best_score:
                    best_position_spread = position_spread
                    best_orientation_spread = orientation_spread
                    best_center_position_drift = center_position_drift
                    best_center_orientation_drift = center_orientation_drift
                if (
                    position_spread <= TARGET_POSITION_SPREAD_M
                    and orientation_spread <= TARGET_ORIENTATION_SPREAD_RAD
                ):
                    accepted_samples = candidate
                    accepted_mode = "strict"
                    break
                if (
                    position_spread <= TARGET_FILTERED_POSITION_SPREAD_M
                    and orientation_spread
                    <= TARGET_FILTERED_ORIENTATION_SPREAD_RAD
                    and center_position_drift <= TARGET_CENTER_DRIFT_M
                    and center_orientation_drift <= TARGET_CENTER_DRIFT_RAD
                ):
                    accepted_samples = candidate
                    accepted_mode = "filtered_stationary"
                    break
        finally:
            self._collect_target_samples = False

        if accepted_samples is None:
            valid_count = len([
                m for m in self._target_samples if m.header.frame_id == TARGET_FRAME
            ])
            if valid_count < TARGET_SAMPLE_COUNT:
                raise RuntimeError("没有收到足够的新鲜torso_link AnyGrasp candidate A")
            raise RuntimeError(
                f"{TARGET_ACQUIRE_TIMEOUT_SEC:.0f}秒内没有找到稳定的"
                f"AnyGrasp连续{TARGET_SAMPLE_COUNT}帧："
                f"best_position_spread={best_position_spread:.6f}m, "
                f"best_orientation_spread={best_orientation_spread:.6f}rad, "
                f"best_center_position_drift="
                f"{best_center_position_drift:.6f}m, "
                f"best_center_orientation_drift="
                f"{best_center_orientation_drift:.6f}rad"
            )
        samples = accepted_samples
        live_reference = average_pose_samples(samples)
        self.last_raw_anygrasp_reference = copy.deepcopy(live_reference)
        position_spread = max(
            pose_distance(live_reference, item)[0] for item in samples
        )
        orientation_spread = max(
            pose_distance(live_reference, item)[1] for item in samples
        )
        reference, orientation_mode, approach_reference_deviation = (
            self.apply_verified_orientation_continuity(live_reference)
        )

        q = reference.pose.orientation
        norm_sq = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w
        if norm_sq < 1e-12:
            raise RuntimeError("AnyGrasp目标四元数为零")
        approach_z = 2.0 * (q.x*q.z - q.y*q.w) / norm_sq
        if approach_z > -MIN_DOWNWARD_APPROACH_COMPONENT:
            raise RuntimeError(
                f"AnyGrasp接近方向不满足向下要求：TCP +X z={approach_z:+.3f}"
            )
        position_offset = self.manual_teach_position_offset_xyz
        manual_orientation = self.manual_teach_orientation_xyzw
        if (position_offset is None) != (manual_orientation is None):
            raise RuntimeError("人工施教实时目标标定不完整")
        if position_offset is None:
            roll = bodyward_down_thumb_roll_deg(reference)
            target = apply_local_x_roll(reference, roll)
            target = apply_final_target_translation(target)
            position_mode = "live_with_legacy_translation"
            applied_right_offset = FINAL_TARGET_RIGHT_OFFSET_M
            applied_lift = FINAL_TARGET_LIFT_M
        else:
            if len(position_offset) != 3:
                raise RuntimeError("人工施教TCP位置偏差长度错误")
            target = copy.deepcopy(reference)
            target.pose.position.x += float(position_offset[0])
            target.pose.position.y += float(position_offset[1])
            target.pose.position.z += float(position_offset[2])
            target_q = normalized_quaternion_xyzw(
                manual_orientation, "人工施教TCP"
            )
            (
                target.pose.orientation.x,
                target.pose.orientation.y,
                target.pose.orientation.z,
                target.pose.orientation.w,
            ) = target_q
            roll = 0.0
            orientation_mode = (
                f"{orientation_mode}+manual_teach_orientation"
            )
            self.last_orientation_consistency_mode = orientation_mode
            position_mode = (
                "live_anygrasp_position_with_taught_world_offset"
            )
            applied_right_offset = 0.0
            applied_lift = 0.0
        target_q = target.pose.orientation
        target_norm_sq = (
            target_q.x*target_q.x + target_q.y*target_q.y
            + target_q.z*target_q.z + target_q.w*target_q.w
        )
        target_approach_z = 2.0 * (
            target_q.x*target_q.z - target_q.y*target_q.w
        ) / target_norm_sq
        if target_approach_z > -MIN_DOWNWARD_APPROACH_COMPONENT:
            raise RuntimeError(
                "标定后的实时AnyGrasp接近方向不满足向下要求："
                f"TCP +X z={target_approach_z:+.3f}"
            )
        thumb_bodyward, thumb_downward, thumb_right = (
            left_thumb_direction_components(target)
        )
        if thumb_bodyward <= 0.0 or thumb_downward <= 0.0:
            raise RuntimeError(
                "拇指方向分支选择失败："
                f"bodyward={thumb_bodyward:+.6f}, "
                f"downward={thumb_downward:+.6f}"
            )
        self.get_logger().info(
            "FROZEN ANYGRASP A | "
            f"mode={accepted_mode}, samples={len(samples)}, "
            f"orientation_mode={orientation_mode}, "
            f"position_mode={position_mode}, "
            f"approach_reference_deviation="
            f"{approach_reference_deviation:.6f}rad, "
            f"position_spread={position_spread:.6f}m, "
            f"orientation_spread={orientation_spread:.6f}rad, "
            f"center_position_drift={center_position_drift:.6f}m, "
            f"center_orientation_drift={center_orientation_drift:.6f}rad, "
            f"roll={roll:+.3f}deg, "
            f"thumb_bodyward={thumb_bodyward:+.6f}, "
            f"thumb_downward={thumb_downward:+.6f}, "
            f"thumb_right_diagnostic={thumb_right:+.6f}, "
            f"right_offset={applied_right_offset:.3f}m, "
            f"total_lift={applied_lift:.3f}m, "
            f"xyz=[{target.pose.position.x:+.4f}, "
            f"{target.pose.position.y:+.4f}, "
            f"{target.pose.position.z:+.4f}]"
        )
        return target, roll, position_spread, orientation_spread

    def make_pose_constraints(self, target):
        constraints = Constraints()
        constraints.name = "measured_taught_end_to_anygrasp_a"
        position = PositionConstraint()
        position.header = copy.deepcopy(target.header)
        position.link_name = EE_LINK
        position.weight = 1.0
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [POSITION_TOLERANCE_M]
        volume = BoundingVolume()
        volume.primitives.append(sphere)
        region_pose = copy.deepcopy(target.pose)
        region_pose.orientation.x = 0.0
        region_pose.orientation.y = 0.0
        region_pose.orientation.z = 0.0
        region_pose.orientation.w = 1.0
        volume.primitive_poses.append(region_pose)
        position.constraint_region = volume

        orientation = OrientationConstraint()
        orientation.header = copy.deepcopy(target.header)
        orientation.link_name = EE_LINK
        orientation.orientation = copy.deepcopy(target.pose.orientation)
        orientation.absolute_x_axis_tolerance = ORIENTATION_TOLERANCE_RAD
        orientation.absolute_y_axis_tolerance = ORIENTATION_TOLERANCE_RAD
        orientation.absolute_z_axis_tolerance = ORIENTATION_TOLERANCE_RAD
        orientation.weight = 1.0
        constraints.position_constraints.append(position)
        constraints.orientation_constraints.append(orientation)
        return constraints

    def make_ik_seed_states(self, measured):
        """Build bounded deterministic seeds around the taught endpoint."""
        offsets = [
            {},
            {"left_wrist_roll_joint": +1.20},
            {"left_wrist_roll_joint": -1.20},
            {"left_shoulder_yaw_joint": +0.50},
            {"left_shoulder_yaw_joint": -0.50},
            {"left_elbow_joint": +0.40},
            {"left_elbow_joint": -0.40},
            {
                "left_shoulder_yaw_joint": +0.45,
                "left_wrist_roll_joint": -1.00,
            },
            {
                "left_shoulder_yaw_joint": -0.45,
                "left_wrist_roll_joint": +1.00,
            },
        ]
        seeds = []
        index = {name: i for i, name in enumerate(measured.name)}
        missing = [name for name in base.LEFT_ARM_JOINTS if name not in index]
        if missing:
            raise RuntimeError(f"IK种子缺少左臂关节：{missing}")
        for changes in offsets:
            seed = copy.deepcopy(measured)
            positions = list(seed.position)
            for name, delta in changes.items():
                lower, upper = base.LEFT_ARM_LIMITS[name]
                positions[index[name]] = max(
                    lower + base.JOINT_LIMIT_MARGIN_RAD,
                    min(
                        upper - base.JOINT_LIMIT_MARGIN_RAD,
                        float(positions[index[name]]) + float(delta),
                    ),
                )
            seed.position = positions
            seeds.append(seed)
        return seeds

    def call_position_ik(self, seed, target, avoid_collisions):
        if not self.ik_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f"{IK_SERVICE}不可用")
        request = GetPositionIK.Request()
        request.ik_request.group_name = base.GROUP_NAME
        request.ik_request.robot_state.joint_state = copy.deepcopy(seed)
        request.ik_request.robot_state.is_diff = False
        request.ik_request.avoid_collisions = bool(avoid_collisions)
        request.ik_request.ik_link_name = EE_LINK
        request.ik_request.pose_stamped = copy.deepcopy(target)
        if self.grasp_ik_constraints is not None:
            request.ik_request.constraints = copy.deepcopy(
                self.grasp_ik_constraints
            )
        request.ik_request.timeout = base.Duration(
            seconds=IK_TIMEOUT_SEC
        ).to_msg()
        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(
            self, future, timeout_sec=IK_CALL_TIMEOUT_SEC
        )
        if not future.done() or future.result() is None:
            raise RuntimeError(f"{IK_SERVICE}调用超时")
        response = future.result()
        return int(response.error_code.val), response.solution.joint_state

    def solve_collision_free_grasp_ik(self, measured, target):
        seeds = self.make_ik_seed_states(measured)
        tested_offsets = []
        any_kinematic_solution = False
        collision_codes = []
        for roll_offset in IK_ROLL_RELAXATIONS_DEG:
            adjusted = apply_local_x_roll(target, roll_offset)
            thumb_bodyward, thumb_downward, thumb_right = (
                left_thumb_direction_components(adjusted)
            )
            if (
                thumb_bodyward < MIN_THUMB_BODYWARD_COMPONENT
                or thumb_downward < MIN_THUMB_DOWNWARD_COMPONENT
            ):
                continue
            tested_offsets.append(float(roll_offset))

            unconstrained_solutions = []
            for seed in seeds:
                code, solution = self.call_position_ik(
                    seed, adjusted, avoid_collisions=False
                )
                if code == 1:
                    unconstrained_solutions.append(solution)
            if not unconstrained_solutions:
                continue

            any_kinematic_solution = True
            collision_seeds = unconstrained_solutions + seeds
            for seed in collision_seeds:
                code, solution = self.call_position_ik(
                    seed, adjusted, avoid_collisions=True
                )
                collision_codes.append(code)
                if code == 1:
                    self.get_logger().info(
                        "ANYGRASP IK PASSED | "
                        f"roll_relaxation={roll_offset:+.1f}deg, "
                        f"thumb_bodyward={thumb_bodyward:+.6f}, "
                        f"thumb_downward={thumb_downward:+.6f}, "
                        f"thumb_right_diagnostic={thumb_right:+.6f}, "
                        f"kinematic_solutions="
                        f"{len(unconstrained_solutions)}"
                    )
                    return solution, adjusted, float(roll_offset)

        if not any_kinematic_solution:
            raise RuntimeError(
                "AnyGrasp抓取规划失败：IK_NO_SOLUTION "
                f"seeds={len(seeds)}, "
                f"tested_roll_offsets_deg={tested_offsets}"
            )
        raise RuntimeError(
            "AnyGrasp抓取规划失败：IK_IN_COLLISION "
            f"tested_roll_offsets_deg={tested_offsets}, "
            f"collision_codes={collision_codes}"
        )

    def make_joint_goal_constraints(self, solution):
        values = dict(zip(solution.name, solution.position))
        missing = [name for name in base.LEFT_ARM_JOINTS if name not in values]
        if missing:
            raise RuntimeError(f"IK解缺少左臂关节：{missing}")
        constraints = Constraints()
        constraints.name = "collision_free_anygrasp_ik_joint_goal"
        for name in base.LEFT_ARM_JOINTS:
            constraint = JointConstraint()
            constraint.joint_name = name
            constraint.position = float(values[name])
            constraint.tolerance_above = IK_JOINT_GOAL_TOLERANCE_RAD
            constraint.tolerance_below = IK_JOINT_GOAL_TOLERANCE_RAD
            constraint.weight = 1.0
            constraints.joint_constraints.append(constraint)
        return constraints

    def publish_planned_target_frames(self, target):
        stamp = self.get_clock().now().to_msg()
        target_tf = TransformStamped()
        target_tf.header.stamp = stamp
        target_tf.header.frame_id = target.header.frame_id
        target_tf.child_frame_id = "planned_left_grasp_target"
        target_tf.transform.translation.x = target.pose.position.x
        target_tf.transform.translation.y = target.pose.position.y
        target_tf.transform.translation.z = target.pose.position.z
        target_tf.transform.rotation = copy.deepcopy(target.pose.orientation)

        # Rotate the diagnostic frame -90 degrees around target local Z so
        # its red +X axis equals target TCP -Y, the RH56H1 thumb direction.
        thumb_tf = copy.deepcopy(target_tf)
        thumb_tf.child_frame_id = "planned_left_grasp_thumb_direction"
        q = target.pose.orientation
        half = math.radians(-90.0) * 0.5
        s, c = math.sin(half), math.cos(half)
        thumb_tf.transform.rotation.x = q.x * c + q.y * s
        thumb_tf.transform.rotation.y = q.y * c - q.x * s
        thumb_tf.transform.rotation.z = q.w * s + q.z * c
        thumb_tf.transform.rotation.w = q.w * c - q.z * s
        self.planned_target_tf.sendTransform([target_tf, thumb_tf])
        self.get_logger().info(
            "RViz目标坐标已发布："
            "planned_left_grasp_thumb_direction的红色+X="
            "本次规划的实际拇指方向"
        )

    def plan_grasp(
        self,
        measured,
        target,
        precomputed_ik=None,
        preselected_target=None,
        preselected_roll_relaxation=0.0,
        planning_attempts=10,
    ):
        if not self.move_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("/move_action 不可用")
        if precomputed_ik is None:
            ik_solution, selected_target, roll_relaxation = (
                self.solve_collision_free_grasp_ik(measured, target)
            )
        else:
            ik_solution = copy.deepcopy(precomputed_ik)
            selected_target = copy.deepcopy(
                target if preselected_target is None
                else preselected_target
            )
            roll_relaxation = float(preselected_roll_relaxation)
        start_state = RobotState()
        start_state.joint_state = copy.deepcopy(measured)
        start_state.is_diff = False
        goal = MoveGroup.Goal()
        goal.request.group_name = base.GROUP_NAME
        goal.request.pipeline_id = "ompl"
        goal.request.planner_id = "left_arm"
        goal.request.num_planning_attempts = int(planning_attempts)
        goal.request.allowed_planning_time = GRASP_ALLOWED_PLANNING_TIME_SEC
        goal.request.max_velocity_scaling_factor = GRASP_PLANNING_VELOCITY_SCALE
        goal.request.max_acceleration_scaling_factor = (
            GRASP_PLANNING_ACCELERATION_SCALE
        )
        goal.request.start_state = start_state
        goal.request.goal_constraints = [
            self.make_joint_goal_constraints(ik_solution)
        ]
        options = PlanningOptions()
        options.plan_only = True
        options.look_around = False
        options.replan = False
        goal.planning_options = options

        future = self.move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError("MoveIt拒绝示教实测终点到AnyGrasp的规划")
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wrapped = result_future.result()
        if wrapped is None or int(wrapped.result.error_code.val) != 1:
            code = None if wrapped is None else int(wrapped.result.error_code.val)
            raise RuntimeError(f"AnyGrasp抓取规划失败：error_code={code}")
        trajectory = wrapped.result.planned_trajectory
        jt = trajectory.joint_trajectory
        if list(jt.joint_names) != base.LEFT_ARM_JOINTS or not jt.points:
            raise RuntimeError("AnyGrasp抓取轨迹关节或点数无效")
        current = dict(zip(measured.name, measured.position))
        first_error = max(abs(float(current[name]) - float(value))
                          for name, value in zip(base.LEFT_ARM_JOINTS, jt.points[0].positions))
        if first_error > base.PLAN_START_BIND_TOLERANCE_RAD:
            raise RuntimeError(f"抓取规划未绑定实测终点：{first_error:.6f} rad")
        self.publish_planned_target_frames(selected_target)
        return (
            trajectory,
            start_state,
            float(wrapped.result.planning_time),
            selected_target,
            roll_relaxation,
        )

    def validate_path_collisions(self, trajectory, measured, label):
        if not self.validity_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("/check_state_validity 不可用")
        names = list(measured.name)
        base_positions = [float(value) for value in measured.position]
        index = {name: i for i, name in enumerate(names)}
        for point_index, point in enumerate(trajectory.points):
            state = RobotState()
            state.joint_state = copy.deepcopy(measured)
            state.joint_state.position = list(base_positions)
            for name, value in zip(base.LEFT_ARM_JOINTS, point.positions):
                state.joint_state.position[index[name]] = float(value)
            request = GetStateValidity.Request()
            request.robot_state = state
            request.group_name = base.GROUP_NAME
            future = self.validity_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            response = future.result()
            if response is None or not response.valid:
                raise RuntimeError(f"{label}第{point_index}点碰撞或检查超时")
        self.get_logger().info(f"{label}碰撞检查通过：{len(trajectory.points)}点")

    def publish_grasp_preview(self, start_state, trajectory):
        display = DisplayTrajectory()
        display.model_id = "G1_D"
        display.trajectory_start = start_state
        display.trajectory.append(trajectory)
        self.display_pub.publish(display)

    def save_grasp(self, trajectory, measured, target, roll, planning_time, metrics, ranges):
        serialized = serialize_message(trajectory)
        current = dict(zip(measured.name, measured.position))
        start_positions = [float(current[name]) for name in base.LEFT_ARM_JOINTS]
        p, q = target.pose.position, target.pose.orientation
        thumb_bodyward, thumb_downward, thumb_right = (
            left_thumb_direction_components(target)
        )
        metadata = {
            "schema_version": 1,
            "artifact_type": "g1_measured_taught_end_to_anygrasp_a",
            "replay_safe": False,
            "candidate": "A",
            "planning_time_sec": planning_time,
            "joint_names": base.LEFT_ARM_JOINTS,
            "point_count": len(trajectory.joint_trajectory.points),
            "trajectory_duration_sec": base.duration_sec(
                trajectory.joint_trajectory.points[-1].time_from_start
            ),
            "generated_unix_time": time.time(),
            "measured_start_positions_rad": start_positions,
            "measured_start_sha256": fingerprint(base.LEFT_ARM_JOINTS, start_positions),
            "measured_stability_ranges_rad": ranges,
            "target_frame": target.header.frame_id,
            "target_position": [p.x, p.y, p.z],
            "target_quaternion_xyzw": [q.x, q.y, q.z, q.w],
            "target_roll_deg": roll,
            "left_thumb_direction": "bodyward_and_down",
            "left_thumb_bodyward_component": thumb_bodyward,
            "left_thumb_downward_component": thumb_downward,
            "left_thumb_robot_right_component_diagnostic": thumb_right,
            "grasp_goal_mode": "collision_free_ik_joint_goal",
            "target_lift_m": FINAL_TARGET_LIFT_M,
            "target_right_offset_m": FINAL_TARGET_RIGHT_OFFSET_M,
            "verified_orientation_reference_file": str(
                VERIFIED_ORIENTATION_REFERENCE_FILE
            ),
            "orientation_consistency_mode": (
                self.last_orientation_consistency_mode
            ),
            "approach_reference_deviation_rad": (
                self.last_approach_reference_deviation_rad
            ),
            "maximum_point_step_rad": metrics[0],
            "minimum_joint_limit_margin_rad": metrics[1],
            "joint_detour_rad": metrics[2],
            "trajectory_sha256": hashlib.sha256(serialized).hexdigest(),
            "fresh_plan_required_for_every_measured_taught_endpoint": True,
            "pre_execute_start_tolerance_rad": (
                base.PRE_EXECUTE_START_TOLERANCE_RAD
            ),
            "grasp_endpoint_tolerance_rad": GRASP_ENDPOINT_TOLERANCE_RAD,
            "program_contains_arm_disarm_calls": False,
            "no_hand_command_sent": True,
        }
        base.atomic_write(GRASP_TRAJECTORY_FILE, serialized, binary=True)
        base.atomic_write(
            GRASP_METADATA_FILE,
            json.dumps(metadata, ensure_ascii=False, indent=2) + "\n",
        )

    def validate_target_still_current(self, frozen):
        # Compare two filtered windows.  Comparing the planned 10-frame mean
        # with one raw frame caused false 8 mm trips even in a static scene.
        latest, _, position_spread, orientation_spread = (
            self.acquire_stable_target()
        )
        distance, angle = pose_distance(frozen, latest)
        approach_angle = approach_axis_angle(frozen, latest)
        if (
            distance > PRE_EXECUTE_TARGET_SHIFT_M
            or approach_angle > PRE_EXECUTE_TARGET_SHIFT_RAD
        ):
            raise RuntimeError(
                f"执行前目标已移动：position={distance:.6f}m, "
                f"approach={approach_angle:.6f}rad, "
                f"full_orientation_diagnostic={angle:.6f}rad"
            )
        self.get_logger().info(
            "PRE-EXECUTE FILTERED TARGET CHECK PASSED | "
            f"position_shift={distance:.6f}m, "
            f"approach_shift={approach_angle:.6f}rad, "
            f"fresh_position_spread={position_spread:.6f}m, "
            f"fresh_orientation_spread={orientation_spread:.6f}rad"
        )


def main():
    args = parse_args()
    taught, metadata = load_local_taught_artifacts()
    taught_start = [float(value) for value in metadata["taught_start_positions_rad"]]
    taught_end = [float(value) for value in metadata["taught_end_positions_rad"]]
    rclpy.init()
    node = TaughtThenAnyGrasp()
    return_code = 2
    try:
        node.validate_scene()
        node.validate_path_shape(taught, "taught path")
        measured = node.wait_fresh_state(clear=True)
        node.validate_taught_collisions(taught, measured)
        alignment, start_state, planning_time = node.plan_alignment(measured, taught_start)
        alignment = node.densify_trajectory(alignment, "alignment path")
        align_metrics = node.validate_path_shape(alignment.joint_trajectory, "alignment path")
        node.validate_path_collisions(
            alignment.joint_trajectory, measured, "对齐轨迹"
        )
        continuous = node.combine_alignment_and_taught(alignment, taught)
        node.save_alignment(alignment, planning_time, align_metrics)
        node.publish_preview(start_state, alignment, taught)
        node.get_logger().info("TAUGHT REPLAY PLAN READY")
        if not args.execute_taught:
            node.get_logger().info("PLAN ONLY COMPLETE: no real motion was sent")
            return_code = 0
        else:
            current = node.wait_fresh_state(clear=True)
            current_map = dict(zip(current.name, current.position))
            start_error = max(abs(float(current_map[name]) - float(value))
                              for name, value in zip(base.LEFT_ARM_JOINTS,
                                                     alignment.joint_trajectory.points[0].positions))
            if start_error > base.PRE_EXECUTE_START_TOLERANCE_RAD:
                raise RuntimeError(f"示教执行前姿态改变：{start_error:.6f} rad")
            node.execute_trajectory(continuous, "current -> taught start -> taught motion")
            measured_end, ranges = node.sample_stable_endpoint()
            nominal_error, nominal_joint = node.measured_target_error(
                measured_end, taught_end, "taught nominal-end diagnostic"
            )
            node.get_logger().warn(
                "示教理论终点误差仅作诊断；下一段使用实测稳定状态："
                f"{nominal_joint}={nominal_error:.6f} rad"
            )
            target, roll, _, _ = node.acquire_stable_target()
            (
                grasp,
                grasp_start,
                grasp_planning_time,
                target,
                roll_relaxation,
            ) = node.plan_grasp(measured_end, target)
            roll += roll_relaxation
            metrics = node.validate_path_shape(grasp.joint_trajectory, "grasp path")
            node.validate_path_collisions(grasp.joint_trajectory, measured_end, "抓取轨迹")
            node.save_grasp(grasp, measured_end, target, roll,
                            grasp_planning_time, metrics, ranges)
            node.publish_grasp_preview(grasp_start, grasp)
            node.get_logger().info(
                "GRASP PLAN READY FROM MEASURED TAUGHT END | "
                f"points={len(grasp.joint_trajectory.points)}"
            )
            if not args.execute_grasp:
                node.get_logger().info(
                    "GRASP PLAN-ONLY COMPLETE: inspect RViz; no grasp motion was sent"
                )
                return_code = 0
            else:
                phrase = input("确认RViz、急停和现场安全后，输入 EXECUTE GRASP：").strip()
                if phrase != "EXECUTE GRASP":
                    raise RuntimeError("未收到准确确认短语，拒绝抓取执行")
                current = node.wait_fresh_state(clear=True)
                current_map = dict(zip(current.name, current.position))
                first = grasp.joint_trajectory.points[0].positions
                start_error = max(abs(float(current_map[name]) - float(value))
                                  for name, value in zip(base.LEFT_ARM_JOINTS, first))
                if start_error > base.PRE_EXECUTE_START_TOLERANCE_RAD:
                    raise RuntimeError(f"抓取执行前姿态改变：{start_error:.6f} rad")
                node.validate_target_still_current(target)
                node.execute_trajectory(grasp, "measured taught end -> AnyGrasp A")
                node.check_measured_target(
                    grasp.joint_trajectory.points[-1].positions,
                    GRASP_ENDPOINT_TOLERANCE_RAD,
                    "grasp-end arrival check",
                )
                node.get_logger().info(
                    "GRASP ENDPOINT READY; no RH56H1 CLOSE command was sent"
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
