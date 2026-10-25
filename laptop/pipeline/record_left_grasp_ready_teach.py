#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Read-only recorder for the startup-to-left_grasp_ready teach motion.

This node only subscribes to /joint_states. It never publishes, calls a
service, sends an action, arms the robot, or changes controller state.
"""

import argparse
import json
import math
import os
import time
from datetime import datetime, timezone
from pathlib import Path

from g1d_project_paths import RAW_DATA_DIR

# Match the laptop MoveIt environment before importing rclpy.
os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
os.environ.setdefault(
    "CYCLONEDDS_URI", "/home/yii/cyclonedds_ws/cyclonedds.xml"
)
os.environ.setdefault("ROS_DOMAIN_ID", "0")
os.environ.setdefault("ROS_LOCALHOST_ONLY", "0")

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


LEFT_ARM_JOINTS = [
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
]

# These are context only. They are recorded when present but are not part of
# the replayed left-arm trajectory.
CONTEXT_JOINTS = ["Yaw_Joint", "torso_Joint"]


def utc_now_iso():
    return datetime.now(timezone.utc).isoformat()


def atomic_write_json(path, payload):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    os.replace(temporary, path)


class TeachRecorder(Node):
    def __init__(self, sample_hz, recording_not_before_monotonic):
        super().__init__("left_grasp_ready_teach_recorder")
        self.sample_period_sec = 1.0 / sample_hz
        self.recording_not_before_monotonic = recording_not_before_monotonic
        self.started_monotonic = None
        self.last_sample_monotonic = None
        self.first_received_iso = None
        self.last_received_iso = None
        self.samples = []
        self.missing_message_count = 0
        self.invalid_message_count = 0
        self.context_joint_names = []
        self.subscription = self.create_subscription(
            JointState,
            "/joint_states",
            self._joint_state_callback,
            50,
        )
        self.get_logger().warn(
            "READ-ONLY RECORDING: no command, service, action, ARM, or motor "
            "control will be sent. Stop with Ctrl+C after Ready is stable."
        )

    def _joint_state_callback(self, msg):
        now = time.monotonic()
        if now < self.recording_not_before_monotonic:
            return
        if (
            self.last_sample_monotonic is not None
            and now - self.last_sample_monotonic < self.sample_period_sec
        ):
            return

        if len(msg.name) != len(msg.position):
            self.invalid_message_count += 1
            return
        current = dict(zip(msg.name, msg.position))
        missing = [name for name in LEFT_ARM_JOINTS if name not in current]
        if missing:
            self.missing_message_count += 1
            if self.missing_message_count == 1:
                self.get_logger().error(
                    f"/joint_states missing required left-arm joints: {missing}"
                )
            return

        positions = [float(current[name]) for name in LEFT_ARM_JOINTS]
        if not all(math.isfinite(value) for value in positions):
            self.invalid_message_count += 1
            return

        if self.started_monotonic is None:
            self.started_monotonic = now
            self.first_received_iso = utc_now_iso()
            self.context_joint_names = [
                name for name in CONTEXT_JOINTS if name in current
            ]
            self.get_logger().info(
                "First complete left-arm state received; recording started."
            )

        context = {
            name: float(current[name])
            for name in self.context_joint_names
            if math.isfinite(float(current[name]))
        }
        self.samples.append(
            {
                "time_from_start_sec": now - self.started_monotonic,
                "positions_rad": positions,
                "context_positions_rad": context,
                "ros_stamp": {
                    "sec": int(msg.header.stamp.sec),
                    "nanosec": int(msg.header.stamp.nanosec),
                },
            }
        )
        self.last_sample_monotonic = now
        self.last_received_iso = utc_now_iso()

        if len(self.samples) % 150 == 0:
            self.get_logger().info(
                f"Recorded {len(self.samples)} samples | "
                f"duration={self.samples[-1]['time_from_start_sec']:.2f}s"
            )

    def build_payload(self, requested_sample_hz):
        if not self.samples:
            raise RuntimeError(
                "没有录到完整的左臂 /joint_states；不生成空示教文件。"
            )
        return {
            "schema_version": 1,
            "recording_type": "startup_to_left_grasp_ready_teach_raw",
            "read_only_recorder": True,
            "source_topic": "/joint_states",
            "requested_sample_hz": requested_sample_hz,
            "started_utc": self.first_received_iso,
            "finished_utc": self.last_received_iso,
            "joint_names": LEFT_ARM_JOINTS,
            "context_joint_names": self.context_joint_names,
            "sample_count": len(self.samples),
            "duration_sec": self.samples[-1]["time_from_start_sec"],
            "missing_message_count": self.missing_message_count,
            "invalid_message_count": self.invalid_message_count,
            "samples": self.samples,
        }


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "只读录制开机姿态到 left_grasp_ready 的左臂关节状态；"
            "不会控制机器人。"
        )
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=RAW_DATA_DIR / "left_grasp_ready_teach_raw.json",
        help="原始示教 JSON 输出路径",
    )
    parser.add_argument(
        "--sample-hz",
        type=float,
        default=30.0,
        help="记录频率，默认 30 Hz，允许 5~100 Hz",
    )
    parser.add_argument(
        "--countdown-sec",
        type=float,
        default=8.0,
        help="开始录制前的准备倒计时，默认 8 秒",
    )
    parser.add_argument(
        "--duration-sec",
        type=float,
        default=20.0,
        help="自动录制时长，默认 20 秒；必须在 3~120 秒",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    if not 5.0 <= args.sample_hz <= 100.0:
        raise SystemExit("--sample-hz 必须在 5~100 Hz 范围内")
    if not 0.0 <= args.countdown_sec <= 60.0:
        raise SystemExit("--countdown-sec 必须在 0~60 秒范围内")
    if not 3.0 <= args.duration_sec <= 120.0:
        raise SystemExit("--duration-sec 必须在 3~120 秒范围内")

    rclpy.init()
    scheduled_start = time.monotonic() + args.countdown_sec
    node = TeachRecorder(args.sample_hz, scheduled_start)
    node.get_logger().warn(
        f"Recording will start after {args.countdown_sec:.1f}s and stop "
        f"automatically after {args.duration_sec:.1f}s."
    )
    save_error = None
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if (
                node.started_monotonic is not None
                and time.monotonic() - node.started_monotonic
                >= args.duration_sec
            ):
                node.get_logger().info(
                    "Automatic recording duration reached; saving now."
                )
                break
    except KeyboardInterrupt:
        pass
    finally:
        try:
            payload = node.build_payload(args.sample_hz)
            atomic_write_json(args.output.expanduser().resolve(), payload)
            node.get_logger().info(
                f"Saved raw teach recording: {args.output.expanduser()} | "
                f"samples={payload['sample_count']} | "
                f"duration={payload['duration_sec']:.2f}s"
            )
        except Exception as exc:
            save_error = exc
            node.get_logger().error(str(exc))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if save_error is not None:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
