#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Continuously align to the taught start and replay the taught arm path.

The alignment is freshly planned by MoveIt from measured /joint_states, then
joined to the fixed taught path as one continuous execution.  This avoids a
stop-and-correct handoff at the taught first pose.  The script never
ARM/DISARMs and requires --execute for real motion.
"""

import argparse
import copy
import hashlib
import json
import math
import os
import time
from pathlib import Path

from g1d_project_paths import RUN_DIR, TAUGHT_DATA_DIR, ensure_run_dir


os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
os.environ["CYCLONEDDS_URI"] = "/home/yii/cyclonedds_ws/cyclonedds.xml"
os.environ["ROS_DOMAIN_ID"] = "0"
os.environ["ROS_LOCALHOST_ONLY"] = "0"

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.serialization import deserialize_message, serialize_message

from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    Constraints,
    DisplayTrajectory,
    JointConstraint,
    PlanningOptions,
    PlanningSceneComponents,
    RobotState,
    RobotTrajectory,
)
from moveit_msgs.srv import GetPlanningScene, GetStateValidity
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory


HERE = Path(__file__).resolve().parent
TAUGHT_TRAJECTORY_FILE = TAUGHT_DATA_DIR / "g1_left_taught_motion.cdr"
TAUGHT_METADATA_FILE = TAUGHT_DATA_DIR / "g1_left_taught_motion.json"
ensure_run_dir()
ALIGN_TRAJECTORY_FILE = RUN_DIR / "g1_left_current_to_taught_start.cdr"
ALIGN_METADATA_FILE = RUN_DIR / "g1_left_current_to_taught_start.json"

MOVE_ACTION = "/move_action"
EXECUTE_ACTION = "/execute_trajectory"
SCENE_SERVICE = "/get_planning_scene"
STATE_VALIDITY_SERVICE = "/check_state_validity"
DISPLAY_TOPIC = "/display_planned_path"
JOINT_STATES_TOPIC = "/joint_states"
TRAJECTORY_STATUS_TOPIC = "/g1_d/trajectory_status"
GROUP_NAME = "left_arm"
REQUIRED_TABLE_ID = "work_table"

LEFT_ARM_JOINTS = [
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
]

LEFT_ARM_LIMITS = {
    "left_shoulder_pitch_joint": (-3.0892, 2.6704),
    "left_shoulder_roll_joint": (-1.5882, 2.2515),
    "left_shoulder_yaw_joint": (-2.6180, 2.6180),
    "left_elbow_joint": (-1.0472, 2.0944),
    "left_wrist_roll_joint": (-1.972222054, 1.972222054),
    "left_wrist_pitch_joint": (-1.614429558, 1.614429558),
    "left_wrist_yaw_joint": (-1.614429558, 1.614429558),
}

JOINT_STATE_TIMEOUT_SEC = 5.0
JOINT_STATE_MAX_AGE_SEC = 0.5
GOAL_TOLERANCE_RAD = 0.005
PLAN_START_BIND_TOLERANCE_RAD = 0.005
PRE_EXECUTE_START_TOLERANCE_RAD = 0.008
STAGE_ENDPOINT_TOLERANCE_RAD = 0.030
SEAM_DURATION_SEC = 0.04
JOINT_LIMIT_MARGIN_RAD = 0.10
MAX_POINT_STEP_RAD = 0.12
DETOUR_WARNING_RAD = 0.30
PLANNING_TIME_SEC = 8.0
VELOCITY_SCALE = 0.05
ACCELERATION_SCALE = 0.05


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "先在线规划到示教首点，再重放固定示教轨迹；默认仅规划和审查。"
        )
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="执行两段动作；脚本不会自动 ARM/DISARM",
    )
    return parser.parse_args()


def sha256(data):
    return hashlib.sha256(data).hexdigest()


def duration_sec(duration):
    return float(duration.sec) + float(duration.nanosec) * 1e-9


def atomic_write(path, data, binary=False):
    temporary = path.with_suffix(path.suffix + ".tmp")
    if binary:
        temporary.write_bytes(data)
    else:
        temporary.write_text(data, encoding="utf-8")
    os.replace(temporary, path)


def load_taught_artifacts():
    trajectory_bytes = TAUGHT_TRAJECTORY_FILE.read_bytes()
    metadata = json.loads(TAUGHT_METADATA_FILE.read_bytes())
    if metadata.get("artifact_type") != "g1_left_taught_motion":
        raise RuntimeError("示教轨迹元数据类型不匹配")
    if sha256(trajectory_bytes) != metadata.get("trajectory_sha256"):
        raise RuntimeError("示教轨迹 SHA-256 校验失败")
    trajectory = deserialize_message(trajectory_bytes, JointTrajectory)
    if list(trajectory.joint_names) != LEFT_ARM_JOINTS:
        raise RuntimeError("示教轨迹关节顺序不匹配")
    if len(trajectory.points) != int(metadata.get("point_count", -1)):
        raise RuntimeError("示教轨迹点数与元数据不匹配")
    return trajectory, metadata


class TaughtReplay(Node):
    def __init__(self):
        super().__init__("left_taught_replay_with_auto_start")
        self.latest_joint_state = None
        self.latest_joint_state_monotonic = None
        self.awaiting_execution_start = False
        self.execution_start_monotonic = None
        self.execution_start_goal_id = None
        self.create_subscription(
            JointState, JOINT_STATES_TOPIC, self.on_joint_state, 50
        )
        self.create_subscription(
            String,
            TRAJECTORY_STATUS_TOPIC,
            self.on_trajectory_status,
            20,
        )
        self.move_client = ActionClient(self, MoveGroup, MOVE_ACTION)
        self.execute_client = ActionClient(
            self, ExecuteTrajectory, EXECUTE_ACTION
        )
        self.scene_client = self.create_client(GetPlanningScene, SCENE_SERVICE)
        self.validity_client = self.create_client(
            GetStateValidity, STATE_VALIDITY_SERVICE
        )
        self.display_pub = self.create_publisher(
            DisplayTrajectory, DISPLAY_TOPIC, 10
        )

    def on_joint_state(self, message):
        self.latest_joint_state = copy.deepcopy(message)
        self.latest_joint_state_monotonic = time.monotonic()

    def on_trajectory_status(self, message):
        if not self.awaiting_execution_start:
            return
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            return
        if (
            str(payload.get("state", "")) == "executing"
            and self.execution_start_monotonic is None
        ):
            self.execution_start_monotonic = time.monotonic()
            self.execution_start_goal_id = str(payload.get("goal_id", ""))
            self.get_logger().info(
                "BOARD EXECUTION START OBSERVED | "
                f"goal={self.execution_start_goal_id or 'unknown'}"
            )

    def wait_fresh_state(self, clear=False):
        if clear:
            self.latest_joint_state = None
            self.latest_joint_state_monotonic = None
        deadline = time.monotonic() + JOINT_STATE_TIMEOUT_SEC
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.latest_joint_state is None:
                continue
            age = time.monotonic() - self.latest_joint_state_monotonic
            if age <= JOINT_STATE_MAX_AGE_SEC:
                return copy.deepcopy(self.latest_joint_state)
        raise RuntimeError("5 秒内没有收到新鲜 /joint_states")

    def validate_scene(self):
        if not self.scene_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("/get_planning_scene 不可用")
        request = GetPlanningScene.Request()
        request.components.components = (
            PlanningSceneComponents.WORLD_OBJECT_NAMES
            | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        )
        future = self.scene_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        response = future.result()
        if response is None:
            raise RuntimeError("无法回读 PlanningScene")
        objects = {
            item.id: item for item in response.scene.world.collision_objects
        }
        table = objects.get(REQUIRED_TABLE_ID)
        if table is None or not table.primitives or not table.primitive_poses:
            raise RuntimeError("PlanningScene 缺少有效 work_table")
        dimensions = [float(v) for v in table.primitives[0].dimensions]
        expected = [0.40, 1.00, 0.12]
        if any(abs(a - b) > 1e-6 for a, b in zip(dimensions, expected)):
            raise RuntimeError("work_table 尺寸不匹配")
        self.get_logger().info("work_table check passed")

    def validate_path_shape(self, trajectory, label):
        if not trajectory.points:
            raise RuntimeError(f"{label} 轨迹为空")
        previous_time = -1.0
        previous = None
        maximum_step = 0.0
        minimum_margin = float("inf")
        detours = {}
        for point_index, point in enumerate(trajectory.points):
            positions = [float(value) for value in point.positions]
            if len(positions) != len(LEFT_ARM_JOINTS):
                raise RuntimeError(f"{label} 第 {point_index} 点长度错误")
            if not all(math.isfinite(value) for value in positions):
                raise RuntimeError(f"{label} 第 {point_index} 点含非有限值")
            timestamp = duration_sec(point.time_from_start)
            if timestamp < previous_time:
                raise RuntimeError(f"{label} 时间戳不单调")
            previous_time = timestamp
            if previous is not None:
                maximum_step = max(
                    maximum_step,
                    max(abs(a - b) for a, b in zip(positions, previous)),
                )
            previous = positions
            for name, value in zip(LEFT_ARM_JOINTS, positions):
                lower, upper = LEFT_ARM_LIMITS[name]
                minimum_margin = min(
                    minimum_margin, value - lower, upper - value
                )
        if maximum_step > MAX_POINT_STEP_RAD:
            raise RuntimeError(
                f"{label} 最大点间变化 {maximum_step:.6f} rad 超限"
            )
        if minimum_margin < JOINT_LIMIT_MARGIN_RAD:
            raise RuntimeError(
                f"{label} 最小关节限位余量 {minimum_margin:.6f} rad 超限"
            )
        for joint_index, name in enumerate(LEFT_ARM_JOINTS):
            values = [
                float(point.positions[joint_index])
                for point in trajectory.points
            ]
            variation = sum(abs(b - a) for a, b in zip(values, values[1:]))
            direct = abs(values[-1] - values[0])
            detour = max(0.0, variation - direct)
            detours[name] = detour
            if detour > DETOUR_WARNING_RAD:
                self.get_logger().warn(
                    f"{label} detour warning: {name}={detour:.6f} rad"
                )
        return maximum_step, minimum_margin, detours

    def make_goal_constraints(self, target):
        constraints = Constraints()
        constraints.name = "left_taught_start_joint_goal"
        for name, value in zip(LEFT_ARM_JOINTS, target):
            item = JointConstraint()
            item.joint_name = name
            item.position = float(value)
            item.tolerance_above = GOAL_TOLERANCE_RAD
            item.tolerance_below = GOAL_TOLERANCE_RAD
            item.weight = 1.0
            constraints.joint_constraints.append(item)
        return constraints

    def plan_alignment(self, measured, taught_start):
        if not self.move_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("/move_action 不可用")
        goal = MoveGroup.Goal()
        request = goal.request
        request.group_name = GROUP_NAME
        request.pipeline_id = "ompl"
        request.planner_id = "left_arm"
        request.num_planning_attempts = 1
        request.allowed_planning_time = PLANNING_TIME_SEC
        request.max_velocity_scaling_factor = VELOCITY_SCALE
        request.max_acceleration_scaling_factor = ACCELERATION_SCALE
        start_state = RobotState()
        start_state.joint_state = copy.deepcopy(measured)
        start_state.is_diff = False
        request.start_state = start_state
        request.goal_constraints = [self.make_goal_constraints(taught_start)]
        options = PlanningOptions()
        options.plan_only = True
        options.look_around = False
        options.replan = False
        goal.planning_options = options
        future = self.move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError("MoveIt 拒绝当前姿态到示教首点的规划请求")
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wrapped = result_future.result()
        if wrapped is None:
            raise RuntimeError("MoveIt 规划没有返回结果")
        result = wrapped.result
        if int(result.error_code.val) != 1:
            raise RuntimeError(
                f"当前姿态到示教首点规划失败：error_code={result.error_code.val}"
            )
        trajectory = result.planned_trajectory.joint_trajectory
        if list(trajectory.joint_names) != LEFT_ARM_JOINTS:
            raise RuntimeError("衔接轨迹关节顺序不匹配")
        current = dict(zip(measured.name, measured.position))
        first_error = max(
            abs(float(current[name]) - float(expected))
            for name, expected in zip(
                LEFT_ARM_JOINTS, trajectory.points[0].positions
            )
        )
        if first_error > PLAN_START_BIND_TOLERANCE_RAD:
            raise RuntimeError(
                f"衔接轨迹没有绑定实测起点：{first_error:.6f} rad"
            )
        goal_error = max(
            abs(float(actual) - float(expected))
            for actual, expected in zip(
                trajectory.points[-1].positions, taught_start
            )
        )
        if goal_error > 0.010:
            raise RuntimeError(
                f"衔接轨迹终点没有到达示教首点：{goal_error:.6f} rad"
            )
        return result.planned_trajectory, start_state, float(result.planning_time)

    def validate_taught_collisions(self, taught, measured):
        if not self.validity_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("/check_state_validity 不可用")
        names = list(measured.name)
        base_positions = [float(value) for value in measured.position]
        name_to_index = {name: i for i, name in enumerate(names)}
        missing = [name for name in LEFT_ARM_JOINTS if name not in name_to_index]
        if missing:
            raise RuntimeError(f"实测状态缺少左臂关节：{missing}")
        for point_index, point in enumerate(taught.points):
            state = RobotState()
            state.joint_state = copy.deepcopy(measured)
            state.joint_state.position = list(base_positions)
            for name, value in zip(LEFT_ARM_JOINTS, point.positions):
                state.joint_state.position[name_to_index[name]] = float(value)
            state.is_diff = False
            request = GetStateValidity.Request()
            request.robot_state = state
            request.group_name = GROUP_NAME
            future = self.validity_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            response = future.result()
            if response is None:
                raise RuntimeError(
                    f"示教轨迹第 {point_index} 点碰撞检查超时"
                )
            if not response.valid:
                contacts = [
                    f"{c.contact_body_1}<->{c.contact_body_2}"
                    for c in response.contacts[:5]
                ]
                raise RuntimeError(
                    f"示教轨迹第 {point_index} 点碰撞：{contacts}"
                )
        self.get_logger().info(
            f"Taught-path collision check passed: {len(taught.points)} points"
        )

    def publish_preview(self, start_state, alignment, taught):
        taught_wrapped = RobotTrajectory()
        taught_wrapped.joint_trajectory = taught
        display = DisplayTrajectory()
        display.model_id = "g1_d"
        display.trajectory_start = start_state
        display.trajectory.append(alignment)
        display.trajectory.append(taught_wrapped)
        self.display_pub.publish(display)

    def combine_alignment_and_taught(self, alignment, taught):
        """Join both paths into one goal so there is no handoff correction."""
        combined = copy.deepcopy(alignment)
        joint_trajectory = combined.joint_trajectory
        if list(joint_trajectory.joint_names) != LEFT_ARM_JOINTS:
            raise RuntimeError("衔接轨迹关节顺序不匹配")
        if list(taught.joint_names) != LEFT_ARM_JOINTS:
            raise RuntimeError("示教轨迹关节顺序不匹配")
        if not joint_trajectory.points or not taught.points:
            raise RuntimeError("连续轨迹含空段")

        offset = duration_sec(joint_trajectory.points[-1].time_from_start)
        for point in taught.points:
            appended = copy.deepcopy(point)
            appended.time_from_start = Duration(
                seconds=(
                    offset
                    + SEAM_DURATION_SEC
                    + duration_sec(point.time_from_start)
                )
            ).to_msg()
            joint_trajectory.points.append(appended)
        self.validate_path_shape(joint_trajectory, "continuous replay path")
        return combined

    def save_alignment(self, alignment, planning_time, metrics):
        serialized = serialize_message(alignment.joint_trajectory)
        metadata = {
            "schema_version": 1,
            "artifact_type": "g1_left_current_to_taught_start_plan_only",
            "replay_safe": False,
            "planning_time_sec": planning_time,
            "joint_names": LEFT_ARM_JOINTS,
            "point_count": len(alignment.joint_trajectory.points),
            "duration_sec": duration_sec(
                alignment.joint_trajectory.points[-1].time_from_start
            ),
            "maximum_point_step_rad": metrics[0],
            "minimum_joint_limit_margin_rad": metrics[1],
            "joint_detour_rad": metrics[2],
            "trajectory_sha256": sha256(serialized),
            "fresh_plan_required_for_every_new_measured_start": True,
        }
        atomic_write(ALIGN_TRAJECTORY_FILE, serialized, binary=True)
        atomic_write(
            ALIGN_METADATA_FILE,
            json.dumps(metadata, ensure_ascii=False, indent=2) + "\n",
        )

    def check_measured_target(self, target, tolerance, label, clear=True):
        measured = self.wait_fresh_state(clear=clear)
        current = dict(zip(measured.name, measured.position))
        missing = [name for name in LEFT_ARM_JOINTS if name not in current]
        if missing:
            raise RuntimeError(f"{label} 缺少左臂关节：{missing}")
        errors = {
            name: abs(float(current[name]) - float(expected))
            for name, expected in zip(LEFT_ARM_JOINTS, target)
        }
        worst_name = max(errors, key=errors.get)
        worst_error = errors[worst_name]
        self.get_logger().info(
            f"{label}: max_error={worst_error:.6f} rad, joint={worst_name}"
        )
        if worst_error > tolerance:
            raise RuntimeError(
                f"{label} 超差：{worst_error:.6f} > {tolerance:.6f} rad"
            )
        return measured

    def measured_target_error(self, measured, target, label):
        current = dict(zip(measured.name, measured.position))
        missing = [name for name in LEFT_ARM_JOINTS if name not in current]
        if missing:
            raise RuntimeError(f"{label} 缺少左臂关节：{missing}")
        errors = {
            name: abs(float(current[name]) - float(expected))
            for name, expected in zip(LEFT_ARM_JOINTS, target)
        }
        worst_name = max(errors, key=errors.get)
        worst_error = errors[worst_name]
        self.get_logger().info(
            f"{label}: max_error={worst_error:.6f} rad, joint={worst_name}"
        )
        return worst_error, worst_name

    def execute_trajectory(
        self,
        trajectory,
        label,
        before_end_callback=None,
        motion_duration_sec=None,
        before_end_sec=1.0,
    ):
        if before_end_callback is not None:
            if motion_duration_sec is None or motion_duration_sec <= 0.0:
                raise ValueError("定时回调需要有效的轨迹运动时长")
            if not 0.0 < before_end_sec < motion_duration_sec:
                raise ValueError("定时回调提前量必须小于轨迹运动时长")
        if not self.execute_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("/execute_trajectory 不可用")
        self.awaiting_execution_start = before_end_callback is not None
        self.execution_start_monotonic = None
        self.execution_start_goal_id = None
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        self.get_logger().warn(f"REAL MOTION: executing {label}")
        future = self.execute_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if handle is None or not handle.accepted:
            self.awaiting_execution_start = False
            raise RuntimeError(f"MoveIt 拒绝执行 {label}")
        result_future = handle.get_result_async()
        callback_sent = False
        callback_error = None
        callback_deadline = None
        while rclpy.ok() and not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            if (
                before_end_callback is not None
                and callback_deadline is None
                and self.execution_start_monotonic is not None
            ):
                callback_deadline = (
                    self.execution_start_monotonic
                    + float(motion_duration_sec)
                    - float(before_end_sec)
                )
                self.get_logger().warn(
                    f"TIMED CALLBACK ARMED FROM BOARD START: "
                    f"{before_end_sec:.3f}s before nominal motion endpoint "
                    f"of {motion_duration_sec:.3f}s"
                )
            if (
                callback_deadline is not None
                and not callback_sent
                and time.monotonic() >= callback_deadline
            ):
                callback_sent = True
                try:
                    before_end_callback()
                except Exception as exc:
                    callback_error = exc
                    self.get_logger().error(
                        "定时回调失败；主轨迹将继续完成："
                        f"{exc}"
                    )
        self.awaiting_execution_start = False
        wrapped = result_future.result()
        if wrapped is None:
            raise RuntimeError(f"{label} 没有返回执行结果")
        code = int(wrapped.result.error_code.val)
        if code != 1:
            raise RuntimeError(f"{label} 执行失败：error_code={code}")
        if before_end_callback is not None and not callback_sent:
            raise RuntimeError(
                f"{label}在定时回调触发前结束；未发送回调命令"
            )
        if callback_error is not None:
            raise RuntimeError(
                f"{label}已完成，但定时回调失败：{callback_error}"
            )


def main():
    args = parse_args()
    taught, metadata = load_taught_artifacts()
    taught_start = [float(v) for v in metadata["taught_start_positions_rad"]]
    taught_end = [float(v) for v in metadata["taught_end_positions_rad"]]
    rclpy.init()
    node = TaughtReplay()
    try:
        node.get_logger().warn(
            "Continuous taught replay: fresh current->taught-start alignment "
            "joined directly to the fixed taught path. No handoff corrections; "
            "no automatic ARM/DISARM."
        )
        node.validate_scene()
        taught_metrics = node.validate_path_shape(taught, "taught path")
        measured = node.wait_fresh_state(clear=True)
        node.validate_taught_collisions(taught, measured)
        alignment, start_state, planning_time = node.plan_alignment(
            measured, taught_start
        )
        align_metrics = node.validate_path_shape(
            alignment.joint_trajectory, "alignment path"
        )
        continuous = node.combine_alignment_and_taught(alignment, taught)
        node.save_alignment(alignment, planning_time, align_metrics)
        node.publish_preview(start_state, alignment, taught)
        node.get_logger().info(
            "CONTINUOUS PLAN PASSED | "
            f"align_points={len(alignment.joint_trajectory.points)}, "
            f"taught_points={len(taught.points)}, "
            f"combined_points={len(continuous.joint_trajectory.points)}, "
            f"taught_source_duration={metadata['source_motion_duration_sec']:.3f}s"
        )
        if not args.execute:
            node.get_logger().info(
                "PLAN ONLY COMPLETE: no ExecuteTrajectory goal was sent"
            )
            return_code = 0
        else:
            current = node.wait_fresh_state(clear=True)
            current_map = dict(zip(current.name, current.position))
            first = alignment.joint_trajectory.points[0].positions
            start_error = max(
                abs(float(current_map[name]) - float(expected))
                for name, expected in zip(LEFT_ARM_JOINTS, first)
            )
            if start_error > PRE_EXECUTE_START_TOLERANCE_RAD:
                raise RuntimeError(
                    "执行前姿态已改变，拒绝旧衔接规划："
                    f"{start_error:.6f} rad"
                )
            node.execute_trajectory(
                continuous,
                "continuous current -> taught start -> taught motion",
            )
            node.check_measured_target(
                taught_end,
                STAGE_ENDPOINT_TOLERANCE_RAD,
                "taught-end arrival check",
            )
            node.get_logger().info("CONTINUOUS TAUGHT REPLAY SUCCESS")
            return_code = 0
    except Exception as exc:
        node.get_logger().error(str(exc))
        return_code = 2
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(return_code)


if __name__ == "__main__":
    main()
