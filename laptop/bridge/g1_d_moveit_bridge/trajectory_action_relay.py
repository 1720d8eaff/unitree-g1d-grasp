#!/usr/bin/env python3

"""Laptop-side Humble FollowJointTrajectory -> JointTrajectory relay.

This node deliberately keeps the Foxy/Humble boundary on trajectory_msgs and
waits for a real status message from the onboard executor.  It never reports
success merely because the planned trajectory duration has elapsed.
"""

import copy
import json
import math
import threading
import time
from typing import Dict, List, Optional, Sequence

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory


LEFT_ARM_JOINTS = [
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
]

RIGHT_ARM_JOINTS = [
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
]

DUAL_ARM_JOINTS = LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS
FINAL_STATES = {"succeeded", "aborted", "canceled", "rejected"}

# Laptop-side trajectory preprocessing.
TIME_SCALE = 5.4
RESAMPLE_MAX_INCREMENT = 0.08
ONBOARD_MAX_TRAJECTORY_INCREMENT = 0.12
ONBOARD_MAX_SEGMENT_SPEED = 0.35
FINAL_SETTLE_TIME_SEC = 3.0


class TrajectoryActionRelay(Node):
    """Expose Humble actions and relay trajectories/status over plain topics."""

    def __init__(self) -> None:
        super().__init__("g1_d_trajectory_action_relay")

        self._callback_group = ReentrantCallbackGroup()
        self._trajectory_publisher = self.create_publisher(
            JointTrajectory, "/g1_d/trajectory_command", 10
        )
        self._cancel_publisher = self.create_publisher(
            String, "/g1_d/trajectory_cancel", 10
        )
        self._status_subscription = self.create_subscription(
            String,
            "/g1_d/trajectory_status",
            self._status_callback,
            20,
            callback_group=self._callback_group,
        )

        self._busy_lock = threading.Lock()
        self._busy = False

        self._status_condition = threading.Condition()
        self._status_by_goal: Dict[str, Dict[str, object]] = {}

        self._action_servers: List[ActionServer] = []
        self._create_action_server(
            "/left_arm_controller/follow_joint_trajectory",
            "left_arm",
            LEFT_ARM_JOINTS,
        )
        self._create_action_server(
            "/right_arm_controller/follow_joint_trajectory",
            "right_arm",
            RIGHT_ARM_JOINTS,
        )
        self._create_action_server(
            "/dual_arm_controller/follow_joint_trajectory",
            "dual_arm",
            DUAL_ARM_JOINTS,
        )

        self.get_logger().info("真实轨迹 Action relay 已启动")
        self.get_logger().warning(
            "该节点会把 MoveIt 轨迹转发给板载机；只有板载执行器 ARMED 时机器人会运动"
        )

    def _create_action_server(
        self,
        action_name: str,
        label: str,
        expected_joints: Sequence[str],
    ) -> None:
        server = ActionServer(
            self,
            FollowJointTrajectory,
            action_name,
            execute_callback=self._make_execute_callback(label),
            goal_callback=self._make_goal_callback(label, expected_joints),
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )
        self._action_servers.append(server)
        self.get_logger().info(f"已创建 Action Server：{action_name}")

    def _make_goal_callback(self, label: str, expected_joints: Sequence[str]):
        def goal_callback(goal_request):
            error = self._validate_goal(goal_request, expected_joints)
            if error is not None:
                self.get_logger().error(f"[{label}] 拒绝轨迹：{error}")
                return GoalResponse.REJECT

            with self._busy_lock:
                if self._busy:
                    self.get_logger().error(f"[{label}] 拒绝轨迹：已有轨迹正在处理")
                    return GoalResponse.REJECT
                self._busy = True

            self.get_logger().info(
                f"[{label}] 接受 Action goal：{len(goal_request.trajectory.points)} 个点"
            )
            return GoalResponse.ACCEPT

        return goal_callback

    def _make_execute_callback(self, label: str):
        def execute_callback(goal_handle):
            result = FollowJointTrajectory.Result()
            goal_id = bytes(goal_handle.goal_id.uuid).hex()
            cancel_sent = False

            try:
                trajectory = copy.deepcopy(goal_handle.request.trajectory)

                original_point_count = len(trajectory.points)

                for point in trajectory.points:
                    total_ns = int(
                        (point.time_from_start.sec * 1e9)
                        + point.time_from_start.nanosec
                    )
                    total_ns = int(total_ns * TIME_SCALE)
                    point.time_from_start.sec = total_ns // 1000000000
                    point.time_from_start.nanosec = total_ns % 1000000000

                    if point.velocities:
                        point.velocities = [
                            value / TIME_SCALE
                            for value in point.velocities
                        ]

                    if point.accelerations:
                        point.accelerations = [
                            value / (TIME_SCALE * TIME_SCALE)
                            for value in point.accelerations
                        ]

                inserted_points = self._resample_trajectory(
                    trajectory,
                    max_increment=RESAMPLE_MAX_INCREMENT,
                )

                # Keep commanding the exact final target for a short settling
                # window. The onboard executor otherwise reports completion at
                # the nominal final timestamp and immediately holds the measured
                # (still lagging) pose. A duplicate zero-velocity endpoint gives
                # the low-level controller time to remove that residual error.
                final_point = copy.deepcopy(trajectory.points[-1])
                final_ns = (
                    int(final_point.time_from_start.sec) * 1_000_000_000
                    + int(final_point.time_from_start.nanosec)
                    + int(FINAL_SETTLE_TIME_SEC * 1_000_000_000)
                )
                final_point.time_from_start.sec = final_ns // 1_000_000_000
                final_point.time_from_start.nanosec = final_ns % 1_000_000_000
                final_point.velocities = [0.0] * len(final_point.positions)
                final_point.accelerations = [0.0] * len(final_point.positions)
                trajectory.points.append(final_point)

                max_increment, max_segment_speed = (
                    self._trajectory_metrics(trajectory)
                )

                self.get_logger().info(
                    f"[{label}] 轨迹预处理："
                    f"{original_point_count} -> {len(trajectory.points)} 个点，"
                    f"插入 {inserted_points} 点，"
                    f"TIME_SCALE={TIME_SCALE:.2f}，"
                    f"FINAL_SETTLE={FINAL_SETTLE_TIME_SEC:.1f}s，"
                    f"max_delta={max_increment:.6f} rad，"
                    f"max_speed={max_segment_speed:.6f} rad/s"
                )

                if max_increment > ONBOARD_MAX_TRAJECTORY_INCREMENT + 1e-9:
                    goal_handle.abort()
                    result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                    result.error_string = (
                        "relay 重采样后仍超过板载单点增量限制："
                        f"{max_increment:.6f} rad > "
                        f"{ONBOARD_MAX_TRAJECTORY_INCREMENT:.6f} rad"
                    )
                    return result

                if max_segment_speed > ONBOARD_MAX_SEGMENT_SPEED + 1e-9:
                    goal_handle.abort()
                    result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                    result.error_string = (
                        "relay 时间缩放后仍超过板载分段速度限制："
                        f"{max_segment_speed:.6f} rad/s > "
                        f"{ONBOARD_MAX_SEGMENT_SPEED:.6f} rad/s"
                    )
                    return result

                trajectory.header.frame_id = f"g1d:{goal_id}"
                final_time = self._duration_seconds(
                    trajectory.points[-1].time_from_start
                )

                with self._status_condition:
                    self._status_by_goal.pop(goal_id, None)

                if not self._wait_for_onboard_subscriber(goal_handle, 5.0):
                    goal_handle.abort()
                    result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                    result.error_string = (
                        "未发现板载机 /g1_d/trajectory_command 订阅者"
                    )
                    return result

                self._trajectory_publisher.publish(trajectory)
                self.get_logger().info(
                    f"[{label}] 已转发 goal={goal_id}，"
                    f"{len(trajectory.joint_names)} 个关节，"
                    f"{len(trajectory.points)} 个点，总时长 {final_time:.3f}s"
                )

                deadline = time.monotonic() + max(45.0, final_time + 30.0)

                while rclpy.ok():
                    if goal_handle.is_cancel_requested and not cancel_sent:
                        self._publish_cancel(goal_id)
                        cancel_sent = True
                        self.get_logger().warning(
                            f"[{label}] 已向板载机转发取消请求：{goal_id}"
                        )

                    status = self._wait_for_status(goal_id, timeout=0.10)
                    if status is not None:
                        state = str(status.get("state", ""))
                        message = str(status.get("message", ""))

                        if state == "accepted":
                            self.get_logger().info(f"[{label}] 板载机已接受轨迹")
                        elif state == "executing":
                            pass
                        elif state == "succeeded":
                            goal_handle.succeed()
                            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                            result.error_string = message or "板载机真实轨迹执行成功"
                            return result
                        elif state == "canceled":
                            goal_handle.canceled()
                            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                            result.error_string = message or "轨迹已取消"
                            return result
                        elif state == "rejected":
                            goal_handle.abort()
                            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                            result.error_string = message or "板载机拒绝轨迹"
                            return result
                        elif state == "aborted":
                            goal_handle.abort()
                            result.error_code = (
                                FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                            )
                            result.error_string = message or "板载机中止轨迹"
                            return result

                    if time.monotonic() >= deadline:
                        self._publish_cancel(goal_id)
                        goal_handle.abort()
                        result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                        result.error_string = "等待板载机最终状态超时，已发送取消请求"
                        return result

                goal_handle.abort()
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = "ROS 2 正在关闭"
                return result

            except Exception as exc:
                self.get_logger().error(f"[{label}] relay 异常：{exc}")
                if not cancel_sent:
                    self._publish_cancel(goal_id)
                goal_handle.abort()
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = str(exc)
                return result

            finally:
                with self._status_condition:
                    self._status_by_goal.pop(goal_id, None)
                with self._busy_lock:
                    self._busy = False

        return execute_callback

    @classmethod
    def _resample_trajectory(cls, trajectory, max_increment: float) -> int:
        if max_increment <= 0.0:
            raise ValueError("max_increment 必须大于 0")

        original_points = list(trajectory.points)
        if len(original_points) < 2:
            return 0

        resampled = [copy.deepcopy(original_points[0])]
        inserted_points = 0

        for previous, current in zip(original_points[:-1], original_points[1:]):
            if len(previous.positions) != len(current.positions):
                raise ValueError("相邻 trajectory point 的 positions 数量不一致")

            max_delta = max(
                (abs(current.positions[i] - previous.positions[i])
                 for i in range(len(current.positions))),
                default=0.0,
            )
            segment_count = max(1, int(math.ceil(max_delta / max_increment)))

            previous_ns = int(previous.time_from_start.sec) * 1_000_000_000 + int(previous.time_from_start.nanosec)
            current_ns = int(current.time_from_start.sec) * 1_000_000_000 + int(current.time_from_start.nanosec)
            segment_ns = current_ns - previous_ns

            if segment_ns <= 0:
                raise ValueError("重采样时发现 time_from_start 未严格递增")
            if segment_ns < segment_count:
                raise ValueError("轨迹时间分辨率不足以完成安全重采样")

            for segment_index in range(1, segment_count + 1):
                if segment_index == segment_count:
                    new_point = copy.deepcopy(current)
                else:
                    ratio = segment_index / float(segment_count)
                    new_point = copy.deepcopy(current)
                    new_point.positions = [
                        previous.positions[i] + ratio * (current.positions[i] - previous.positions[i])
                        for i in range(len(current.positions))
                    ]

                    if previous.velocities and current.velocities:
                        new_point.velocities = [
                            previous.velocities[i] + ratio * (current.velocities[i] - previous.velocities[i])
                            for i in range(len(current.velocities))
                        ]
                    else:
                        new_point.velocities = []

                    if previous.accelerations and current.accelerations:
                        new_point.accelerations = [
                            previous.accelerations[i] + ratio * (current.accelerations[i] - previous.accelerations[i])
                            for i in range(len(current.accelerations))
                        ]
                    else:
                        new_point.accelerations = []

                    if previous.effort and current.effort:
                        new_point.effort = [
                            previous.effort[i] + ratio * (current.effort[i] - previous.effort[i])
                            for i in range(len(current.effort))
                        ]
                    else:
                        new_point.effort = []

                    interpolated_ns = previous_ns + (segment_ns * segment_index) // segment_count
                    new_point.time_from_start.sec = interpolated_ns // 1_000_000_000
                    new_point.time_from_start.nanosec = interpolated_ns % 1_000_000_000

                resampled.append(new_point)

            inserted_points += segment_count - 1

        trajectory.points = resampled
        return inserted_points

    @classmethod
    def _trajectory_metrics(cls, trajectory):
        if len(trajectory.points) < 2:
            return 0.0, 0.0

        max_increment = 0.0
        max_segment_speed = 0.0
        previous = trajectory.points[0]
        previous_time = cls._duration_seconds(previous.time_from_start)

        for current in trajectory.points[1:]:
            current_time = cls._duration_seconds(current.time_from_start)
            dt = current_time - previous_time
            if dt <= 0.0:
                raise ValueError("轨迹指标检查发现 time_from_start 未严格递增")

            for i in range(len(current.positions)):
                delta = abs(current.positions[i] - previous.positions[i])
                max_increment = max(max_increment, delta)
                max_segment_speed = max(max_segment_speed, delta / dt)

            previous = current
            previous_time = current_time

        return max_increment, max_segment_speed

    def _wait_for_onboard_subscriber(self, goal_handle, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                return False
            if self._trajectory_publisher.get_subscription_count() > 0:
                return True
            time.sleep(0.10)
        return self._trajectory_publisher.get_subscription_count() > 0

    def _status_callback(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            goal_id = str(payload.get("goal_id", ""))
            state = str(payload.get("state", ""))
            if not goal_id or state not in {
                "accepted", "executing", "succeeded", "aborted", "canceled", "rejected"
            }:
                self.get_logger().warning(
                    f"忽略格式不正确的 trajectory_status：{message.data}"
                )
                return

            with self._status_condition:
                self._status_by_goal[goal_id] = payload
                self._status_condition.notify_all()
        except Exception as exc:
            self.get_logger().warning(f"无法解析 trajectory_status：{exc}")

    def _wait_for_status(
        self, goal_id: str, timeout: float
    ) -> Optional[Dict[str, object]]:
        with self._status_condition:
            status = self._status_by_goal.pop(goal_id, None)
            if status is not None:
                return status
            self._status_condition.wait(timeout=timeout)
            return self._status_by_goal.pop(goal_id, None)

    def _publish_cancel(self, goal_id: str) -> None:
        message = String()
        message.data = goal_id
        self._cancel_publisher.publish(message)

    def _cancel_callback(self, _goal_handle):
        return CancelResponse.ACCEPT

    @classmethod
    def _validate_goal(
        cls, goal_request, expected_joints: Sequence[str]
    ) -> Optional[str]:
        trajectory = goal_request.trajectory
        joint_names = list(trajectory.joint_names)

        if (
            goal_request.multi_dof_trajectory.joint_names
            or goal_request.multi_dof_trajectory.points
        ):
            return "当前桥接器不支持 multi_dof_trajectory"

        if len(joint_names) != len(set(joint_names)):
            return "joint_names 中存在重复名称"

        if set(joint_names) != set(expected_joints):
            missing = sorted(set(expected_joints) - set(joint_names))
            extra = sorted(set(joint_names) - set(expected_joints))
            return f"关节集合不匹配；missing={missing}, extra={extra}"

        if not trajectory.points:
            return "轨迹中没有 point"

        previous_time = -1.0
        for point_index, point in enumerate(trajectory.points, start=1):
            if len(point.positions) != len(joint_names):
                return f"point {point_index} positions 数量错误"
            if point.velocities and len(point.velocities) != len(joint_names):
                return f"point {point_index} velocities 数量错误"
            if point.accelerations and len(point.accelerations) != len(joint_names):
                return f"point {point_index} accelerations 数量错误"
            if not all(math.isfinite(value) for value in point.positions):
                return f"point {point_index} 包含非有限位置值"

            point_time = cls._duration_seconds(point.time_from_start)
            if point_time < 0.0:
                return f"point {point_index} 时间不能为负数"
            if point_time <= previous_time:
                return (
                    "time_from_start 必须严格递增；"
                    f"point {point_index}={point_time:.6f}s"
                )
            previous_time = point_time

        if previous_time <= 0.0:
            return "轨迹总时长必须大于 0"

        return None

    @staticmethod
    def _duration_seconds(duration) -> float:
        return float(duration.sec) + float(duration.nanosec) * 1e-9


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrajectoryActionRelay()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
