#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from service_interfaces.msg import SetAngle1


class RH56H1VirtualGripperMapper(Node):
    # Real-hand calibration:
    # width 25 mm -> index 1194, middle(raw) 1216
    # width 60 mm -> index 1392, middle(raw) 1414
    # width 80 mm -> index 1590, middle(raw) 1612
    WIDTHS_M = [0.025, 0.060, 0.080]
    INDEX_CMD = [1194, 1392, 1590]
    MIDDLE_RAW_CMD = [1216, 1414, 1612]

    # Real-object correction validated with the 50 mm cube:
    # at CLOSE=45 mm, middle 1329 did not contact well;
    # middle 1307 was suitable -> fixed correction -22.
    MIDDLE_CONTACT_OFFSET = -22

    # Validated thumb opposition pose.
    THUMB_FLEX_CMD = 1350
    THUMB_ABDUCTION_CMD = 880

    # Safe virtual opening. 80 mm would make the original middle command
    # exceed the driver's 1600 limit.
    MIN_WIDTH_M = 0.025
    MAX_SAFE_WIDTH_M = 0.078

    def __init__(self):
        super().__init__('rh56h1_virtual_gripper_mapper')

        self.publisher = self.create_publisher(
            SetAngle1,
            '/set_angle_data',
            10
        )

        self.subscription = self.create_subscription(
            Float32,
            '/virtual_gripper/target_width',
            self.width_callback,
            10
        )

        self.get_logger().info('RH56H1 virtual gripper mapper v5 started')
        self.get_logger().info(
            'Input: /virtual_gripper/target_width [m], safe range 0.025~0.078'
        )
        self.get_logger().info(
            'Controls Middle(3), Index(4), Thumb Flexion(5), Thumb Abduction(6).'
        )
        self.get_logger().info(
            'Middle correction=-22; thumb_flex=1350; thumb_abduction=880.'
        )

    @staticmethod
    def interp_piecewise(x, xs, ys):
        if x <= xs[0]:
            return ys[0]
        if x >= xs[-1]:
            return ys[-1]

        for i in range(len(xs) - 1):
            x0, x1 = xs[i], xs[i + 1]
            if x0 <= x <= x1:
                y0, y1 = ys[i], ys[i + 1]
                ratio = (x - x0) / (x1 - x0)
                return y0 + ratio * (y1 - y0)

        raise RuntimeError('Interpolation failed')

    def width_callback(self, msg):
        width = float(msg.data)

        if not math.isfinite(width):
            self.get_logger().error('Rejected non-finite target width')
            return

        if width < self.MIN_WIDTH_M or width > self.MAX_SAFE_WIDTH_M:
            self.get_logger().error(
                f'Rejected width={width:.4f} m: safe range is '
                f'{self.MIN_WIDTH_M:.3f}~{self.MAX_SAFE_WIDTH_M:.3f} m'
            )
            return

        index = int(round(self.interp_piecewise(
            width,
            self.WIDTHS_M,
            self.INDEX_CMD
        )))

        middle_raw = int(round(self.interp_piecewise(
            width,
            self.WIDTHS_M,
            self.MIDDLE_RAW_CMD
        )))

        middle = middle_raw + self.MIDDLE_CONTACT_OFFSET

        # Extra protection against invalid commands.
        index = max(850, min(1600, index))
        middle = max(850, min(1600, middle))

        cmd = SetAngle1()
        cmd.finger_ids = [3, 4, 5, 6]
        cmd.angles = [
            middle,
            index,
            self.THUMB_FLEX_CMD,
            self.THUMB_ABDUCTION_CMD,
        ]

        self.publisher.publish(cmd)

        self.get_logger().info(
            f'width={width * 1000.0:.1f} mm -> '
            f'middle={middle} (raw={middle_raw}, offset={self.MIDDLE_CONTACT_OFFSET}), '
            f'index={index}, '
            f'thumb_flex={self.THUMB_FLEX_CMD}, '
            f'thumb_abduction={self.THUMB_ABDUCTION_CMD}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = RH56H1VirtualGripperMapper()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
