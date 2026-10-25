#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
AnyGrasp -> RH56H1 left-hand automatic orientation selector.

Input:
  UDP 5011 pose_matrix in torso_link (raw AnyGrasp pose after hand-eye transform)

Publishes TF / PoseStamped:
  anygrasp_target_raw
  left_grasp_target_A
  left_grasp_target_B
  left_grasp_target          <- automatically selected A/B
  left_pregrasp_target       <- 5 cm behind + 5 cm vertical lift from selected target

Selection:
  - A and B differ by 180 deg roll about the AnyGrasp/TCP approach X axis.
  - Candidate A is selected deterministically; it is never switched from the
    current wrist pose or TF availability.
  - Physical RViz verification showed candidate B's hand-model thumb opposite
    the desired direction.  Candidate A supplies the required 180-degree roll.
  - The diagnostic TF left_grasp_thumb_direction uses its red +X axis to show
    the actual A-pose thumb direction (TCP -Y).

IMPORTANT:
  Visualization/planning target only. This script never commands the robot.
"""

import json
import math
import socket

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import Buffer, TransformBroadcaster, TransformListener


UDP_HOST = "0.0.0.0"
UDP_PORT = 5011

PARENT_FRAME = "torso_link"
CURRENT_TCP_FRAME = "left_grasp_tcp"

PREGRASP_DISTANCE_M = 0.05
PREGRASP_LIFT_M = 0.05

# Switch only when the alternative is clearly better.
SWITCH_HYSTERESIS_DEG = 20.0


def rx(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = np.array([
        [1.0, 0.0, 0.0],
        [0.0, c, -s],
        [0.0, s,  c],
    ], dtype=np.float64)
    return T


def rz(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = np.array([
        [c, -s, 0.0],
        [s,  c, 0.0],
        [0.0, 0.0, 1.0],
    ], dtype=np.float64)
    return T


# Mapping already validated visually:
# AnyGrasp +X -> TCP +X (approach)
#
# A: TCP +Z = AnyGrasp +Y
# B: TCP +Z = AnyGrasp -Y
MAP_A = rx(-90.0)
MAP_B = rx(+90.0)


def quat_xyzw_to_R(qx, qy, qz, qw):
    q = np.array([qx, qy, qz, qw], dtype=np.float64)
    n = np.linalg.norm(q)
    if n < 1e-12:
        return np.eye(3, dtype=np.float64)
    q /= n
    x, y, z, w = q

    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x*x + y*y)],
    ], dtype=np.float64)


def R_to_quat_xyzw(R):
    R = np.asarray(R, dtype=np.float64)
    tr = float(np.trace(R))

    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * s
        qx = (R[2, 1] - R[1, 2]) / s
        qy = (R[0, 2] - R[2, 0]) / s
        qz = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s

    q = np.array([qx, qy, qz, qw], dtype=np.float64)
    q /= np.linalg.norm(q)
    return q


def rotation_angle_deg(R1, R2):
    dR = R1.T @ R2
    c = (np.trace(dR) - 1.0) * 0.5
    c = float(np.clip(c, -1.0, 1.0))
    return math.degrees(math.acos(c))


def fill_pose(T, frame_id, stamp):
    p = T[:3, 3]
    q = R_to_quat_xyzw(T[:3, :3])

    msg = PoseStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.pose.position.x = float(p[0])
    msg.pose.position.y = float(p[1])
    msg.pose.position.z = float(p[2])
    msg.pose.orientation.x = float(q[0])
    msg.pose.orientation.y = float(q[1])
    msg.pose.orientation.z = float(q[2])
    msg.pose.orientation.w = float(q[3])
    return msg


def fill_tf(T, parent, child, stamp):
    p = T[:3, 3]
    q = R_to_quat_xyzw(T[:3, :3])

    tf = TransformStamped()
    tf.header.stamp = stamp
    tf.header.frame_id = parent
    tf.child_frame_id = child

    tf.transform.translation.x = float(p[0])
    tf.transform.translation.y = float(p[1])
    tf.transform.translation.z = float(p[2])

    tf.transform.rotation.x = float(q[0])
    tf.transform.rotation.y = float(q[1])
    tf.transform.rotation.z = float(q[2])
    tf.transform.rotation.w = float(q[3])
    return tf


class AutoSelector(Node):
    def __init__(self):
        super().__init__("anygrasp_left_auto_selector")

        self.pub_raw = self.create_publisher(
            PoseStamped, "/anygrasp/target_pose_raw", 10
        )
        self.pub_a = self.create_publisher(
            PoseStamped, "/anygrasp/left_target_candidate_a", 10
        )
        self.pub_b = self.create_publisher(
            PoseStamped, "/anygrasp/left_target_candidate_b", 10
        )
        self.pub_final = self.create_publisher(
            PoseStamped, "/anygrasp/left_grasp_target", 10
        )
        self.pub_pre = self.create_publisher(
            PoseStamped, "/anygrasp/left_pregrasp_target", 10
        )

        self.br = TransformBroadcaster(self)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer, self, spin_thread=False
        )

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((UDP_HOST, UDP_PORT))
        self.sock.setblocking(False)

        self.previous_choice = None
        self.last_log_time = self.get_clock().now()

        self.timer = self.create_timer(0.01, self.poll)

        self.get_logger().info(f"Listening UDP {UDP_HOST}:{UDP_PORT}")
        self.get_logger().info(
            f"Pregrasp offset: {PREGRASP_DISTANCE_M*100:.0f} cm along -TCP X + {PREGRASP_LIFT_M*100:.0f} cm Z lift"
        )
        self.get_logger().info(
            f"A/B switch hysteresis: {SWITCH_HYSTERESIS_DEG:.0f} deg"
        )
        self.get_logger().warn(
            "Visualization/planning only. Do NOT Execute yet."
        )

    def get_current_tcp_R(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                PARENT_FRAME,
                CURRENT_TCP_FRAME,
                Time(),
                timeout=Duration(seconds=0.005),
            )
            q = tf.transform.rotation
            return quat_xyzw_to_R(q.x, q.y, q.z, q.w)
        except Exception:
            return None

    def select_candidate(self, T_a, T_b):
        R_current = self.get_current_tcp_R()

        # Preserve the physically verified A mapping even when the current TCP
        # TF is unavailable. Candidate B's hand-model thumb was observed to be
        # exactly opposite the desired direction.
        if R_current is None:
            return "A", None, None, "fixed A: verified hand-model thumb"

        angle_a = rotation_angle_deg(R_current, T_a[:3, :3])
        angle_b = rotation_angle_deg(R_current, T_b[:3, :3])

        # Fixed physical-hand roll selection. A/B share the same approach axis
        # and differ by 180 degrees around TCP +X.
        choice = "A"
        reason = "fixed A: verified hand-model thumb"

        return choice, angle_a, angle_b, reason
    def poll(self):
        try:
            data, _ = self.sock.recvfrom(65535)
        except BlockingIOError:
            return

        try:
            payload = json.loads(data.decode("utf-8"))
            T_raw = np.asarray(payload["pose_matrix"], dtype=np.float64)
            if T_raw.shape != (4, 4):
                raise ValueError("pose_matrix is not 4x4")
        except Exception as exc:
            self.get_logger().warning(f"Bad packet: {exc}")
            return

        frame = payload.get("frame_id", PARENT_FRAME)

        T_a = T_raw @ MAP_A
        T_b = T_raw @ MAP_B

        choice, angle_a, angle_b, reason = self.select_candidate(T_a, T_b)
        self.previous_choice = choice

        # Keep AnyGrasp's approach direction and apply the verified A roll.
        T_final = T_a.copy()

        # The hand-model thumb points along TCP -Y.  With candidate A this is
        # the same world direction as the previously confirmed desired red
        # axis (+Y of B), because A/B differ by 180 degrees about TCP +X.
        T_thumb_direction = T_final @ rz(-90.0)

        # Calibration previews only.  Post-multiplying by Rx rotates around
        # the TCP +X approach axis, so position and approach direction remain
        # exactly unchanged.  The final published target stays at 0 degrees
        # until the operator visually selects a correction in RViz.
        T_thumb_roll_m10 = T_final @ rx(-10.0)
        T_thumb_roll_p10 = T_final @ rx(+10.0)

        # Pregrasp = move opposite the TCP +X approach direction.
        approach = T_final[:3, 0].copy()
        approach /= np.linalg.norm(approach)

        up = np.array([0.0, 0.0, 1.0])
        
        # Pregrasp strategy:
        # Keep grasp orientation unchanged.
        # Move backward 5 cm along TCP approach direction.
        # Lift 5 cm along vertical Z direction for table clearance.

        T_pre = T_final.copy()
        T_pre[:3, 3] = (
            T_final[:3, 3]
            - PREGRASP_DISTANCE_M * approach
            + PREGRASP_LIFT_M * up
        )

        stamp = self.get_clock().now().to_msg()

        self.pub_raw.publish(fill_pose(T_raw, frame, stamp))
        self.pub_a.publish(fill_pose(T_a, frame, stamp))
        self.pub_b.publish(fill_pose(T_b, frame, stamp))
        self.pub_final.publish(fill_pose(T_final, frame, stamp))
        self.pub_pre.publish(fill_pose(T_pre, frame, stamp))

        self.br.sendTransform([
            fill_tf(T_raw, frame, "anygrasp_target_raw", stamp),
            fill_tf(T_a, frame, "left_grasp_target_A", stamp),
            fill_tf(T_b, frame, "left_grasp_target_B", stamp),
            fill_tf(T_final, frame, "left_grasp_target", stamp),
            fill_tf(
                T_thumb_direction,
                frame,
                "left_grasp_thumb_direction",
                stamp,
            ),
            fill_tf(
                T_thumb_roll_m10,
                frame,
                "left_grasp_thumb_roll_m10",
                stamp,
            ),
            fill_tf(
                T_thumb_roll_p10,
                frame,
                "left_grasp_thumb_roll_p10",
                stamp,
            ),
            fill_tf(T_pre, frame, "left_pregrasp_target", stamp),
        ])

        # Throttled status log ~1 Hz.
        now = self.get_clock().now()
        if (now - self.last_log_time).nanoseconds > 1_000_000_000:
            p = T_final[:3, 3]
            if angle_a is not None:
                self.get_logger().info(
                    f"SELECT {choice} | "
                    f"A={angle_a:.1f}deg B={angle_b:.1f}deg | "
                    f"{reason} | "
                    f"grasp xyz=[{p[0]:+.3f},{p[1]:+.3f},{p[2]:+.3f}]"
                )
            else:
                self.get_logger().info(
                    f"SELECT {choice} | {reason} | "
                    f"grasp xyz=[{p[0]:+.3f},{p[1]:+.3f},{p[2]:+.3f}]"
                )
            self.last_log_time = now

    def destroy_node(self):
        try:
            self.sock.close()
        finally:
            super().destroy_node()


def main():
    rclpy.init()
    node = AutoSelector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
