#!/usr/bin/env python3

import math
from typing import Dict, List, Optional, Sequence

import rclpy
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

EXPECTED_GROUPS: Dict[str, Sequence[str]] = {
    "left_arm": LEFT_ARM_JOINTS,
    "right_arm": RIGHT_ARM_JOINTS,
    "dual_arm": DUAL_ARM_JOINTS,
}


class TrajectoryTopicDryRun(Node):

    def __init__(self) -> None:
        super().__init__("g1_d_trajectory_topic_dry_run")

        self._subscription = self.create_subscription(
            JointTrajectory,
            "/g1_d/trajectory_command",
            self._trajectory_callback,
            10,
        )

        self.get_logger().info(
            "正在监听 /g1_d/trajectory_command"
        )
        self.get_logger().warning(
            "当前为 TOPIC DRY-RUN：不会发布 LowCmd，机器人不会运动"
        )

    def _trajectory_callback(
        self,
        message: JointTrajectory,
    ) -> None:
        group_name = self._identify_group(
            message.joint_names,
        )

        if group_name is None:
            self.get_logger().error(
                "拒绝轨迹：关节集合不属于左臂、右臂或双臂；"
                f"joint_names={list(message.joint_names)}"
            )
            return

        error = self._validate_trajectory(message)

        if error is not None:
            self.get_logger().error(
                f"[{group_name}] 拒绝轨迹：{error}"
            )
            return

        self.get_logger().info(
            f"[{group_name}] 接收到合法轨迹："
            f"{len(message.joint_names)} 个关节，"
            f"{len(message.points)} 个点"
        )

        self.get_logger().info(
            f"[{group_name}] joint_names="
            f"{list(message.joint_names)}"
        )

        for point_index, point in enumerate(
            message.points,
            start=1,
        ):
            point_time = (
                float(point.time_from_start.sec)
                + float(point.time_from_start.nanosec) * 1e-9
            )

            positions_text = ", ".join(
                f"{value:+.4f}"
                for value in point.positions
            )

            self.get_logger().info(
                f"[{group_name}] point {point_index}: "
                f"t={point_time:.3f}s, "
                f"positions=[{positions_text}]"
            )

        self.get_logger().info(
            f"[{group_name}] TOPIC DRY-RUN 校验通过"
        )

    @staticmethod
    def _identify_group(
        joint_names: Sequence[str],
    ) -> Optional[str]:
        received_set = set(joint_names)

        if len(received_set) != len(joint_names):
            return None

        for group_name, expected_joints in EXPECTED_GROUPS.items():
            if received_set == set(expected_joints):
                return group_name

        return None

    @staticmethod
    def _validate_trajectory(
        message: JointTrajectory,
    ) -> Optional[str]:
        joint_count = len(message.joint_names)

        if joint_count == 0:
            return "joint_names 为空"

        if not message.points:
            return "轨迹没有任何 point"

        previous_time = -1.0

        for point_index, point in enumerate(
            message.points,
            start=1,
        ):
            if len(point.positions) != joint_count:
                return (
                    f"point {point_index} positions 数量为 "
                    f"{len(point.positions)}，应为 {joint_count}"
                )

            if point.velocities and (
                len(point.velocities) != joint_count
            ):
                return (
                    f"point {point_index} velocities 数量错误"
                )

            if point.accelerations and (
                len(point.accelerations) != joint_count
            ):
                return (
                    f"point {point_index} accelerations 数量错误"
                )

            if not all(
                math.isfinite(value)
                for value in point.positions
            ):
                return (
                    f"point {point_index} 包含非有限位置值"
                )

            point_time = (
                float(point.time_from_start.sec)
                + float(point.time_from_start.nanosec) * 1e-9
            )

            if point_time < 0.0:
                return (
                    f"point {point_index} 的时间不能为负数"
                )

            if point_time <= previous_time:
                return (
                    "time_from_start 必须严格递增；"
                    f"point {point_index}={point_time:.6f}s"
                )

            previous_time = point_time

        return None


def main(args=None) -> None:
    rclpy.init(args=args)

    node = TrajectoryTopicDryRun()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
