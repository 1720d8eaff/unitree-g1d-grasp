#!/usr/bin/env python3

import json
import math
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


JOINT_NAMES = [
    "Left_Wheel_Joint",
    "Right_Wheel_Joint",
    "LZ_mt_Joint",
    "LZ_it_Joint",
    "Yaw_Joint",
    "torso_Joint",

    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",

    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",

    "left_hand_index_0_joint",
    "left_hand_index_1_joint",
    "left_hand_middle_0_joint",
    "left_hand_middle_1_joint",
    "left_hand_thumb_0_joint",
    "left_hand_thumb_1_joint",
    "left_hand_thumb_2_joint",

    "right_hand_index_0_joint",
    "right_hand_index_1_joint",
    "right_hand_middle_0_joint",
    "right_hand_middle_1_joint",
    "right_hand_thumb_0_joint",
    "right_hand_thumb_1_joint",
    "right_hand_thumb_2_joint",
]

STATE_FILE = "/tmp/g1_lowstate.json"


class G1DStateBridge(Node):

    def __init__(self):
        super().__init__("g1_d_state_bridge")

        self.publisher = self.create_publisher(
            JointState,
            "/joint_states",
            10,
        )

        self.timer = self.create_timer(
            0.02,
            self.publish_joint_state,
        )

        self.get_logger().info(
            "G1-D JSON to JointState bridge started."
        )

    def publish_joint_state(self):
        if not os.path.exists(STATE_FILE):
            return

        try:
            with open(STATE_FILE, "r") as file:
                data = json.load(file)

            position = data["position"]
            waist_yaw = float(data["waist_yaw_position"])
            waist_pitch = float(data["waist_pitch_position"])
            velocity = data.get("velocity", [])
            effort = data.get("effort", [])

            if len(position) != 14:
                self.get_logger().warning(
                    f"Expected 14 positions, got {len(position)}"
                )
                return

            if not all(math.isfinite(value) for value in position):
                self.get_logger().warning(
                    "Joint positions contain NaN or Inf."
                )
                return

            if not math.isfinite(waist_yaw) or not math.isfinite(waist_pitch):
                self.get_logger().warning(
                    "Waist joint positions contain NaN or Inf."
                )
                return

            message = JointState()
            message.header.stamp = self.get_clock().now().to_msg()
            message.name = JOINT_NAMES
            # URDF Yaw_Joint rotates around Y, so it represents physical
            # waist pitch (motor 14). torso_Joint rotates around Z and maps
            # to physical waist yaw (motor 12).
            message.position = [
                0.0,
                0.0,
                0.0,
                0.0,
                waist_pitch,
                waist_yaw,
            ] + position + [0.0] * 14

            # if len(velocity) == 14:
            #    message.velocity = velocity

            # if len(effort) == 14:
            #    message.effort = effort

            self.publisher.publish(message)

        except (OSError, json.JSONDecodeError, KeyError) as error:
            self.get_logger().warning(
                f"Failed to read state file: {error}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = G1DStateBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
