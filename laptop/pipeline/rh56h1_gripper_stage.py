#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""High-level RH56H1 PREOPEN/CLOSE stage using standard ROS messages only."""

import time

import rclpy
from std_msgs.msg import Float32, String


class RH56H1GripperStage:
    def __init__(self, node, object_width_m=0.05):
        self.node = node
        self.object_width_m = float(object_width_m)
        if not 0.025 <= self.object_width_m <= 0.078:
            raise ValueError("object_width_m必须在[0.025, 0.078]米")
        self.latest_state = None
        self.width_ready = False
        self.width_pub = node.create_publisher(
            Float32, "/virtual_gripper/object_width", 10
        )
        self.command_pub = node.create_publisher(
            String, "/virtual_gripper/command", 10
        )
        self.state_sub = node.create_subscription(
            String, "/virtual_gripper/state", self._state_callback, 10
        )

    def _state_callback(self, message):
        self.latest_state = message.data.strip().upper()

    def _wait_connections(self):
        deadline = time.monotonic() + 3.0
        while rclpy.ok() and time.monotonic() < deadline:
            if (
                self.width_pub.get_subscription_count() > 0
                and self.command_pub.get_subscription_count() > 0
            ):
                return
            rclpy.spin_once(self.node, timeout_sec=0.1)
        raise RuntimeError(
            "找不到RH56H1高层控制器；请检查controller_v4、mapper_v5和UDP driver"
        )

    def _publish_width_and_wait(self):
        self._wait_connections()
        self.width_ready = False
        message = Float32()
        message.data = self.object_width_m
        self.latest_state = None
        for _ in range(3):
            self.width_pub.publish(message)
            rclpy.spin_once(self.node, timeout_sec=0.1)
        deadline = time.monotonic() + 2.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.latest_state in ("WIDTH_READY", "ERROR"):
                break
        if self.latest_state != "WIDTH_READY":
            raise RuntimeError(
                "RH56H1未确认物体宽度："
                f"state={self.latest_state!r}"
            )
        self.width_ready = True
        self.node.get_logger().info(
            f"RH56H1 WIDTH_READY: object_width={self.object_width_m:.3f}m"
        )

    def _command_and_wait(self, command, expected):
        message = String()
        message.data = command
        self.latest_state = None
        self.command_pub.publish(message)
        deadline = time.monotonic() + 2.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.latest_state in (expected, "ERROR"):
                break
        if self.latest_state != expected:
            raise RuntimeError(
                f"RH56H1 {command}未确认：state={self.latest_state!r}"
            )
        self.node.get_logger().info(f"RH56H1 {expected}")

    def preopen(self):
        self._publish_width_and_wait()
        self._command_and_wait("PREOPEN", "PREOPEN_COMMAND_SENT")

    def close(self):
        if not self.width_ready:
            self._publish_width_and_wait()
        self._command_and_wait("CLOSE", "CLOSE_COMMAND_SENT")
