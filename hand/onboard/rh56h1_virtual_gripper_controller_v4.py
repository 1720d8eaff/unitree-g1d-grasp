#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String


class RH56H1VirtualGripperController(Node):
    """
    High-level virtual gripper controller for RH56H1.

    Inputs
    ------
    /virtual_gripper/object_width : std_msgs/msg/Float32
        Object/grasp width in meters.

    /virtual_gripper/command : std_msgs/msg/String
        PREOPEN
        CLOSE
        RELEASE

    Outputs
    -------
    /virtual_gripper/target_width : std_msgs/msg/Float32
        Target virtual gripper width sent to the existing mapper.

    /virtual_gripper/state : std_msgs/msg/String
        IDLE / WIDTH_READY / PREOPEN_COMMAND_SENT / CLOSE_COMMAND_SENT / RELEASE_COMMAND_SENT / ERROR

    Notes
    -----
    This node does NOT talk to the RH56H1 hardware directly.
    It reuses the already validated:
        target_width -> rh56h1_virtual_gripper_mapper_v2.py
                     -> /set_angle_data
                     -> rh56h1_udp_driver
                     -> UDP gateway
                     -> RH56H1
    """

    WIDTH_MIN = 0.025
    WIDTH_MAX = 0.078

    PREOPEN_WIDTH = 0.070    # fixed 70 mm pre-open width
    CLOSE_MARGIN = 0.005     # object width - 5 mm
    RELEASE_WIDTH = 0.078

    def __init__(self):
        super().__init__('rh56h1_virtual_gripper_controller')

        self.object_width = None

        self.target_pub = self.create_publisher(
            Float32,
            '/virtual_gripper/target_width',
            10
        )
        self.state_pub = self.create_publisher(
            String,
            '/virtual_gripper/state',
            10
        )

        self.width_sub = self.create_subscription(
            Float32,
            '/virtual_gripper/object_width',
            self.width_callback,
            10
        )
        self.command_sub = self.create_subscription(
            String,
            '/virtual_gripper/command',
            self.command_callback,
            10
        )

        self.publish_state('IDLE')

        self.get_logger().info('RH56H1 virtual gripper controller started')
        self.get_logger().info(
            'object_width -> PREOPEN(70 mm fixed), CLOSE(-5 mm), RELEASE(78 mm)'
        )
        self.get_logger().info(
            'Calibrated virtual width range: 25~78 mm'
        )

    def publish_state(self, state):
        msg = String()
        msg.data = state
        self.state_pub.publish(msg)

    def publish_target_width(self, width_m):
        msg = Float32()
        msg.data = float(width_m)
        self.target_pub.publish(msg)

    @staticmethod
    def mm(width_m):
        return width_m * 1000.0

    def width_callback(self, msg):
        width = float(msg.data)

        if not math.isfinite(width):
            self.get_logger().error('Rejected non-finite object_width')
            self.publish_state('ERROR')
            return

        if width < self.WIDTH_MIN or width > self.WIDTH_MAX:
            self.get_logger().error(
                f'Rejected object_width={self.mm(width):.1f} mm: '
                f'valid range is {self.mm(self.WIDTH_MIN):.0f}~'
                f'{self.mm(self.WIDTH_MAX):.0f} mm'
            )
            self.publish_state('ERROR')
            return

        self.object_width = width
        self.publish_state('WIDTH_READY')

        preopen = self.PREOPEN_WIDTH
        close = max(width - self.CLOSE_MARGIN, self.WIDTH_MIN)

        self.get_logger().info(
            f'object_width={self.mm(width):.1f} mm -> '
            f'PREOPEN={self.mm(preopen):.1f} mm, '
            f'CLOSE={self.mm(close):.1f} mm'
        )

    def command_callback(self, msg):
        command = msg.data.strip().upper()

        if command == 'RELEASE':
            self.publish_target_width(self.RELEASE_WIDTH)
            self.publish_state('RELEASE_COMMAND_SENT')
            self.get_logger().info(
                f'RELEASE -> target_width={self.mm(self.RELEASE_WIDTH):.1f} mm'
            )
            return

        if self.object_width is None:
            self.get_logger().error(
                f'{command} rejected: no valid object_width received yet'
            )
            self.publish_state('ERROR')
            return

        if command == 'PREOPEN':
            target = self.PREOPEN_WIDTH
            self.publish_target_width(target)
            self.publish_state('PREOPEN_COMMAND_SENT')
            self.get_logger().info(
                f'PREOPEN -> target_width={self.mm(target):.1f} mm'
            )

        elif command == 'CLOSE':
            target = max(
                self.object_width - self.CLOSE_MARGIN,
                self.WIDTH_MIN
            )
            self.publish_target_width(target)
            self.publish_state('CLOSE_COMMAND_SENT')
            self.get_logger().info(
                f'CLOSE -> target_width={self.mm(target):.1f} mm'
            )

        else:
            self.get_logger().error(
                f'Unknown command "{msg.data}". '
                f'Use PREOPEN / CLOSE / RELEASE.'
            )
            self.publish_state('ERROR')


def main(args=None):
    rclpy.init(args=args)
    node = RH56H1VirtualGripperController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
