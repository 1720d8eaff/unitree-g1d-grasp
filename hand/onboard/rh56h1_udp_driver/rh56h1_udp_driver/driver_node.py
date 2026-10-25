#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socket
import time

import rclpy
from rclpy.node import Node

from service_interfaces.msg import (
    GetAngleAct1,
    GetForceAct1,
    SetAngle1,
    SetForce1,
    SetSpeed1,
)


FINGER_NAMES = [
    "Pinky",
    "Ring Finger",
    "Middle Finger",
    "Index Finger",
    "Thumb Flexion",
    "Thumb Abduction",
]

ANGLE_SET_ADDR = [1040, 1041, 1042, 1043, 1044, 1045]
FORCE_SET_ADDR = [1046, 1047, 1048, 1049, 1050, 1051]
SPEED_SET_ADDR = [1052, 1053, 1054, 1055, 1056, 1057]

ANGLE_ACT_BASE = 1064
FORCE_ACT_BASE = 1070

ANGLE_LIMITS = [
    (850, 1600),
    (850, 1600),
    (850, 1600),
    (850, 1600),
    (950, 1350),
    (670, 1700),
]

FORCE_LIMITS = [(0, 900)] * 6
SPEED_LIMITS = [(0, 4000)] * 6


class RH56H1UdpDriver(Node):
    def __init__(self):
        super().__init__("rh56h1_udp_driver")

        self.declare_parameter("gateway_ip", "192.168.123.222")
        self.declare_parameter("gateway_port", 5005)
        self.declare_parameter("hand_id", 1)
        self.declare_parameter("poll_hz", 10.0)
        self.declare_parameter("udp_timeout", 0.5)

        self.gateway_ip = str(self.get_parameter("gateway_ip").value)
        self.gateway_port = int(self.get_parameter("gateway_port").value)
        self.hand_id = int(self.get_parameter("hand_id").value)
        self.poll_hz = float(self.get_parameter("poll_hz").value)
        self.udp_timeout = float(self.get_parameter("udp_timeout").value)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(self.udp_timeout)

        self.angle_pub = self.create_publisher(GetAngleAct1, "angle_data", 10)
        self.force_pub = self.create_publisher(GetForceAct1, "force_data", 10)

        self.create_subscription(
            SetAngle1, "set_angle_data", self.angle_callback, 10
        )
        self.create_subscription(
            SetForce1, "set_force_data", self.force_callback, 10
        )
        self.create_subscription(
            SetSpeed1, "set_speed_data", self.speed_callback, 10
        )

        period = 1.0 / max(self.poll_hz, 0.5)
        self.timer = self.create_timer(period, self.publish_state)

        self.last_comm_error_log = 0.0

        self.get_logger().info("RH56H1 UDP driver started")
        self.get_logger().info(
            "gateway={}:{} hand_id={} poll_hz={}".format(
                self.gateway_ip,
                self.gateway_port,
                self.hand_id,
                self.poll_hz,
            )
        )
        self.get_logger().info(
            "Single-finger commands use single-register writes; "
            "no -1 placeholder group writes."
        )

    @staticmethod
    def _checksum(frame_without_checksum):
        return sum(frame_without_checksum[2:]) & 0xFF

    def _build_read(self, address, nbytes):
        frame = [
            0xEB,
            0x90,
            self.hand_id,
            0x04,
            0x11,
            address & 0xFF,
            (address >> 8) & 0xFF,
            nbytes & 0xFF,
        ]
        frame.append(self._checksum(frame))
        return bytes(frame)

    def _build_write_short(self, address, value):
        value16 = int(value) & 0xFFFF
        frame = [
            0xEB,
            0x90,
            self.hand_id,
            0x05,  # 2 data bytes + 3
            0x12,
            address & 0xFF,
            (address >> 8) & 0xFF,
            value16 & 0xFF,
            (value16 >> 8) & 0xFF,
        ]
        frame.append(self._checksum(frame))
        return bytes(frame)

    @staticmethod
    def _validate_checksum(frame):
        if len(frame) < 9:
            raise RuntimeError("response frame too short")
        expected = sum(frame[2:-1]) & 0xFF
        if frame[-1] != expected:
            raise RuntimeError(
                "bad checksum: got 0x{:02X}, expected 0x{:02X}".format(
                    frame[-1], expected
                )
            )

    def _transact(self, request):
        self.sock.sendto(
            request,
            (self.gateway_ip, self.gateway_port),
        )
        response, addr = self.sock.recvfrom(4096)

        if response.startswith(b"ERR:"):
            raise RuntimeError(response.decode(errors="replace"))

        if addr[0] != self.gateway_ip:
            raise RuntimeError(
                "unexpected UDP response source: {}:{}".format(
                    addr[0], addr[1]
                )
            )

        if response[:2] != b"\x90\xEB":
            raise RuntimeError(
                "unexpected response header: {}".format(
                    response[:2].hex(" ").upper()
                )
            )

        self._validate_checksum(response)
        return response

    def _read6(self, address):
        response = self._transact(self._build_read(address, 12))

        if len(response) < 20:
            raise RuntimeError(
                "read response too short: {} bytes".format(len(response))
            )
        if response[4] != 0x11:
            raise RuntimeError(
                "unexpected read response cmd: 0x{:02X}".format(response[4])
            )
        if response[5] != (address & 0xFF) or \
                response[6] != ((address >> 8) & 0xFF):
            raise RuntimeError("read response address mismatch")

        raw = response[7:19]
        values = []

        for i in range(0, 12, 2):
            value = raw[i] | (raw[i + 1] << 8)
            if value >= 0x8000:
                value -= 0x10000
            values.append(int(value))

        return values

    def _write_short(self, address, value):
        request = self._build_write_short(address, value)
        response = self._transact(request)

        if len(response) < 9:
            raise RuntimeError(
                "write response too short: {} bytes".format(len(response))
            )
        if response[4] != 0x12:
            raise RuntimeError(
                "unexpected write response cmd: 0x{:02X}".format(
                    response[4]
                )
            )
        if response[5] != (address & 0xFF) or \
                response[6] != ((address >> 8) & 0xFF):
            raise RuntimeError("write response address mismatch")
        if response[7] != 0x01:
            raise RuntimeError(
                "RH56H1 did not confirm write: status=0x{:02X}".format(
                    response[7]
                )
            )

    @staticmethod
    def _validate_pairs(finger_ids, values):
        if len(finger_ids) != len(values):
            raise ValueError(
                "finger_ids length {} != values length {}".format(
                    len(finger_ids), len(values)
                )
            )
        if len(finger_ids) == 0:
            raise ValueError("empty command")

    def _write_command(self, kind, finger_ids, values, addresses, limits):
        self._validate_pairs(finger_ids, values)

        commands = []

        for finger_id, value in zip(finger_ids, values):
            finger_id = int(finger_id)
            value = int(value)

            if finger_id < 1 or finger_id > 6:
                raise ValueError(
                    "invalid finger_id {}, expected 1..6".format(finger_id)
                )

            lo, hi = limits[finger_id - 1]
            if value < lo or value > hi:
                raise ValueError(
                    "{} finger {} value {} outside [{}, {}]".format(
                        kind, finger_id, value, lo, hi
                    )
                )

            commands.append(
                (finger_id, value, addresses[finger_id - 1])
            )

        # Deliberately write each requested finger separately.
        for finger_id, value, address in commands:
            self._write_short(address, value)
            self.get_logger().info(
                "{} finger_id={} value={} addr={}".format(
                    kind, finger_id, value, address
                )
            )

    def angle_callback(self, msg):
        try:
            self._write_command(
                "ANGLE",
                msg.finger_ids,
                msg.angles,
                ANGLE_SET_ADDR,
                ANGLE_LIMITS,
            )
        except Exception as exc:
            self.get_logger().error("angle command rejected/failed: {}".format(exc))

    def force_callback(self, msg):
        try:
            self._write_command(
                "FORCE",
                msg.finger_ids,
                msg.forces,
                FORCE_SET_ADDR,
                FORCE_LIMITS,
            )
        except Exception as exc:
            self.get_logger().error("force command rejected/failed: {}".format(exc))

    def speed_callback(self, msg):
        try:
            self._write_command(
                "SPEED",
                msg.finger_ids,
                msg.speeds,
                SPEED_SET_ADDR,
                SPEED_LIMITS,
            )
        except Exception as exc:
            self.get_logger().error("speed command rejected/failed: {}".format(exc))

    def _log_comm_error(self, exc):
        now = time.monotonic()
        if now - self.last_comm_error_log >= 1.0:
            self.last_comm_error_log = now
            self.get_logger().error("RH56H1 communication error: {}".format(exc))

    def publish_state(self):
        try:
            angles = self._read6(ANGLE_ACT_BASE)
            forces = self._read6(FORCE_ACT_BASE)

            angle_msg = GetAngleAct1()
            angle_msg.finger_ids = [1, 2, 3, 4, 5, 6]
            angle_msg.angle_values = angles
            angle_msg.finger_names = list(FINGER_NAMES)
            self.angle_pub.publish(angle_msg)

            force_msg = GetForceAct1()
            force_msg.finger_ids = [1, 2, 3, 4, 5, 6]
            force_msg.force_values = forces
            force_msg.finger_names = list(FINGER_NAMES)
            self.force_pub.publish(force_msg)

        except Exception as exc:
            self._log_comm_error(exc)

    def destroy_node(self):
        try:
            self.sock.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RH56H1UdpDriver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
