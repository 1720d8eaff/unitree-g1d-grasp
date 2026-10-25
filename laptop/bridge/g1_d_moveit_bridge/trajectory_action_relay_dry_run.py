#!/usr/bin/env python3

import math
import threading
import time
from typing import List, Optional, Sequence

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer
from rclpy.action import CancelResponse
from rclpy.action import GoalResponse
from rclpy.node import Node
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


class TrajectoryActionRelayDryRun(Node):
    """
    笔记本 Humble 端：
    FollowJointTrajectory Action -> JointTrajectory topic。

    板载机当前仍是 dry-run 接收节点，所以不会控制机器人。
    """

    def __init__(self) -> None:
        super().__init__(
            "g1_d_trajectory_action_relay_dry_run"
        )

        self._trajectory_publisher = self.create_publisher(
            JointTrajectory,
            "/g1_d/trajectory_command",
            10,
        )

        self._busy_lock = threading.Lock()
        self._busy = False

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

        self.get_logger().info(
            "Humble FollowJointTrajectory 转发节点已启动"
        )
        self.get_logger().warning(
            "当前为 DRY-RUN：只转发 JointTrajectory，"
            "板载机不会发布 LowCmd"
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
            execute_callback=self._make_execute_callback(
                label,
            ),
            goal_callback=self._make_goal_callback(
                label,
                expected_joints,
            ),
            cancel_callback=self._cancel_callback,
        )

        self._action_servers.append(server)

        self.get_logger().info(
            f"已创建 Action Server：{action_name}"
        )

    def _make_goal_callback(
        self,
        label: str,
        expected_joints: Sequence[str],
    ):
        def goal_callback(goal_request):
            error = self._validate_goal(
                goal_request,
                expected_joints,
            )

            if error is not None:
                self.get_logger().error(
                    f"[{label}] 拒绝轨迹：{error}"
                )
                return GoalResponse.REJECT

            with self._busy_lock:
                if self._busy:
                    self.get_logger().error(
                        f"[{label}] 拒绝轨迹："
                        "已有轨迹正在执行"
                    )
                    return GoalResponse.REJECT

                self._busy = True

            self.get_logger().info(
                f"[{label}] 接受轨迹："
                f"{len(goal_request.trajectory.points)} 个点"
            )

            return GoalResponse.ACCEPT

        return goal_callback

    def _make_execute_callback(self, label: str):
        def execute_callback(goal_handle):
            result = FollowJointTrajectory.Result()

            try:
                trajectory = (
                    goal_handle.request.trajectory
                )

                # 防止 topic 订阅者尚未完成 DDS 发现时丢失消息。
                discovery_deadline = (
                    time.monotonic() + 5.0
                )

                while (
                    self._trajectory_publisher
                    .get_subscription_count() == 0
                    and time.monotonic()
                    < discovery_deadline
                ):
                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()

                        result.error_code = (
                            FollowJointTrajectory.Result
                            .INVALID_GOAL
                        )
                        result.error_string = (
                            "等待板载机订阅者时被取消"
                        )
                        return result

                    time.sleep(0.10)

                if (
                    self._trajectory_publisher
                    .get_subscription_count() == 0
                ):
                    goal_handle.abort()

                    result.error_code = (
                        FollowJointTrajectory.Result
                        .INVALID_GOAL
                    )
                    result.error_string = (
                        "未发现板载机 "
                        "/g1_d/trajectory_command 订阅者"
                    )
                    return result

                self._trajectory_publisher.publish(
                    trajectory
                )

                final_time = self._duration_seconds(
                    trajectory.points[-1]
                    .time_from_start
                )

                self.get_logger().info(
                    f"[{label}] 已转发到板载机："
                    f"{len(trajectory.joint_names)} 个关节，"
                    f"{len(trajectory.points)} 个点，"
                    f"总时长 {final_time:.3f}s"
                )

                # dry-run 中按轨迹总时长等待，
                # 模拟 FollowJointTrajectory 执行过程。
                start_time = time.monotonic()

                while (
                    time.monotonic() - start_time
                    < final_time
                ):
                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()

                        result.error_code = (
                            FollowJointTrajectory.Result
                            .INVALID_GOAL
                        )
                        result.error_string = (
                            "DRY-RUN 轨迹已取消"
                        )

                        self.get_logger().warning(
                            f"[{label}] 轨迹已取消"
                        )
                        return result

                    time.sleep(0.05)

                goal_handle.succeed()

                result.error_code = (
                    FollowJointTrajectory.Result.SUCCESSFUL
                )
                result.error_string = (
                    "DRY-RUN 转发成功；"
                    "板载机仅校验轨迹，机器人未运动"
                )

                self.get_logger().info(
                    f"[{label}] DRY-RUN Action 完成"
                )

                return result

            except Exception as exc:
                self.get_logger().error(
                    f"[{label}] 执行异常：{exc}"
                )

                goal_handle.abort()

                result.error_code = (
                    FollowJointTrajectory.Result
                    .INVALID_GOAL
                )
                result.error_string = str(exc)

                return result

            finally:
                with self._busy_lock:
                    self._busy = False

        return execute_callback

    def _cancel_callback(self, goal_handle):
        self.get_logger().info(
            "收到轨迹取消请求"
        )
        return CancelResponse.ACCEPT

    @classmethod
    def _validate_goal(
        cls,
        goal_request,
        expected_joints: Sequence[str],
    ) -> Optional[str]:
        trajectory = goal_request.trajectory
        joint_names = list(
            trajectory.joint_names
        )

        if (
            goal_request.multi_dof_trajectory.joint_names
            or goal_request.multi_dof_trajectory.points
        ):
            return (
                "当前桥接器不支持 "
                "multi_dof_trajectory"
            )

        if len(joint_names) != len(set(joint_names)):
            return "joint_names 中存在重复名称"

        if set(joint_names) != set(expected_joints):
            missing = sorted(
                set(expected_joints)
                - set(joint_names)
            )

            extra = sorted(
                set(joint_names)
                - set(expected_joints)
            )

            return (
                f"关节集合不匹配；"
                f"missing={missing}, extra={extra}"
            )

        if not trajectory.points:
            return "轨迹中没有 point"

        previous_time = -1.0

        for point_index, point in enumerate(
            trajectory.points,
            start=1,
        ):
            if (
                len(point.positions)
                != len(joint_names)
            ):
                return (
                    f"point {point_index} "
                    "positions 数量错误"
                )

            if (
                point.velocities
                and len(point.velocities)
                != len(joint_names)
            ):
                return (
                    f"point {point_index} "
                    "velocities 数量错误"
                )

            if (
                point.accelerations
                and len(point.accelerations)
                != len(joint_names)
            ):
                return (
                    f"point {point_index} "
                    "accelerations 数量错误"
                )

            if not all(
                math.isfinite(value)
                for value in point.positions
            ):
                return (
                    f"point {point_index} "
                    "包含非有限位置值"
                )

            point_time = cls._duration_seconds(
                point.time_from_start
            )

            if point_time < 0.0:
                return (
                    f"point {point_index} "
                    "时间不能为负数"
                )

            if point_time <= previous_time:
                return (
                    "time_from_start 必须严格递增；"
                    f"point {point_index}="
                    f"{point_time:.6f}s"
                )

            previous_time = point_time

        return None

    @staticmethod
    def _duration_seconds(duration) -> float:
        return (
            float(duration.sec)
            + float(duration.nanosec) * 1e-9
        )


def main(args=None) -> None:
    rclpy.init(args=args)

    node = TrajectoryActionRelayDryRun()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
