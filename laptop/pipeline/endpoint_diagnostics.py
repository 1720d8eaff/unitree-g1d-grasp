#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Read-only post-action joint and FK diagnostics for the G1-D left arm."""

import copy
import hashlib
import json
import math
import time
from pathlib import Path

import rclpy
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState


SAMPLE_OFFSETS_SEC = (0.0, 1.0, 3.0, 5.0)
FK_SERVICE = "/compute_fk"
FK_FRAME = "torso_link"
FK_LINK = "left_grasp_tcp"
FK_TIMEOUT_SEC = 5.0
TRACE_PERIOD_SEC = 0.05


def _stamp_dict(stamp):
    return {"sec": int(stamp.sec), "nanosec": int(stamp.nanosec)}


def _pose_dict(pose_stamped):
    position = pose_stamped.pose.position
    orientation = pose_stamped.pose.orientation
    return {
        "frame_id": pose_stamped.header.frame_id,
        "position_m": [
            float(position.x), float(position.y), float(position.z)
        ],
        "quaternion_xyzw": [
            float(orientation.x),
            float(orientation.y),
            float(orientation.z),
            float(orientation.w),
        ],
    }


def _quaternion_angle_rad(a, b):
    dot = abs(
        float(a.x) * float(b.x)
        + float(a.y) * float(b.y)
        + float(a.z) * float(b.z)
        + float(a.w) * float(b.w)
    )
    norm_a = math.sqrt(a.x*a.x + a.y*a.y + a.z*a.z + a.w*a.w)
    norm_b = math.sqrt(b.x*b.x + b.y*b.y + b.z*b.z + b.w*b.w)
    if norm_a < 1e-12 or norm_b < 1e-12:
        return float("inf")
    cosine = max(-1.0, min(1.0, dot / (norm_a * norm_b)))
    return 2.0 * math.acos(cosine)


class EndpointDiagnostics:
    """Collect fresh states without publishing commands or action goals."""

    def __init__(self, node, joint_names, output_file):
        self.node = node
        self.joint_names = list(joint_names)
        self.output_file = Path(output_file)
        self._collect_trace = False
        self._trace_messages = []
        self.trace_sub = node.create_subscription(
            JointState, "/joint_states", self._on_trace_state, 100
        )
        self.fk_client = node.create_client(GetPositionFK, FK_SERVICE)

    def _on_trace_state(self, message):
        if self._collect_trace:
            self._trace_messages.append(
                (time.monotonic(), copy.deepcopy(message))
            )

    def _fk(self, joint_state):
        request = GetPositionFK.Request()
        request.header.frame_id = FK_FRAME
        request.fk_link_names = [FK_LINK]
        request.robot_state = RobotState()
        request.robot_state.joint_state = copy.deepcopy(joint_state)
        future = self.fk_client.call_async(request)
        rclpy.spin_until_future_complete(
            self.node, future, timeout_sec=FK_TIMEOUT_SEC
        )
        response = future.result()
        if response is None:
            raise RuntimeError("/compute_fk未返回结果")
        if int(response.error_code.val) != 1:
            raise RuntimeError(
                f"/compute_fk失败：error_code={response.error_code.val}"
            )
        if len(response.pose_stamped) != 1:
            raise RuntimeError("/compute_fk未返回唯一left_grasp_tcp位姿")
        return response.pose_stamped[0]

    def compute_fk(self, joint_state):
        """Return torso_link -> left_grasp_tcp FK for a complete joint state."""
        if not self.fk_client.wait_for_service(timeout_sec=FK_TIMEOUT_SEC):
            raise RuntimeError("/compute_fk不可用")
        return self._fk(joint_state)

    def _target_joint_state(self, measured, target_positions):
        target = copy.deepcopy(measured)
        index = {name: i for i, name in enumerate(target.name)}
        missing = [name for name in self.joint_names if name not in index]
        if missing:
            raise RuntimeError(f"/joint_states缺少左臂关节：{missing}")
        positions = list(target.position)
        for name, value in zip(self.joint_names, target_positions):
            positions[index[name]] = float(value)
        target.position = positions
        if target.velocity:
            target.velocity = [0.0] * len(target.name)
        if target.effort:
            target.effort = [0.0] * len(target.name)
        return target

    def _sample(self, requested_offset, origin_monotonic, target_positions):
        deadline = origin_monotonic + requested_offset
        while rclpy.ok() and time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            rclpy.spin_once(self.node, timeout_sec=min(0.05, remaining))

        measured = self.node.wait_fresh_state(clear=True)
        captured_monotonic = time.monotonic()
        current = dict(zip(measured.name, measured.position))
        missing = [name for name in self.joint_names if name not in current]
        if missing:
            raise RuntimeError(f"终点诊断缺少左臂关节：{missing}")

        actual = [float(current[name]) for name in self.joint_names]
        signed_errors = [
            value - float(target)
            for value, target in zip(actual, target_positions)
        ]
        absolute_errors = [abs(value) for value in signed_errors]
        worst_index = max(range(len(absolute_errors)), key=absolute_errors.__getitem__)

        velocity_map = {}
        if len(measured.velocity) == len(measured.name):
            velocity_map = dict(zip(measured.name, measured.velocity))
        velocities = [
            float(velocity_map[name]) if name in velocity_map else None
            for name in self.joint_names
        ]

        result = {
            "requested_offset_sec": float(requested_offset),
            "captured_offset_sec": captured_monotonic - origin_monotonic,
            "joint_state_stamp": _stamp_dict(measured.header.stamp),
            "actual_positions_rad": actual,
            "target_positions_rad": [float(v) for v in target_positions],
            "signed_errors_rad_actual_minus_target": signed_errors,
            "absolute_errors_rad": absolute_errors,
            "actual_velocities_rad_sec": velocities,
            "maximum_absolute_error_rad": absolute_errors[worst_index],
            "worst_joint": self.joint_names[worst_index],
        }

        try:
            actual_fk = self._fk(measured)
            target_state = self._target_joint_state(measured, target_positions)
            target_fk = self._fk(target_state)
            actual_p = actual_fk.pose.position
            target_p = target_fk.pose.position
            delta = [
                float(actual_p.x - target_p.x),
                float(actual_p.y - target_p.y),
                float(actual_p.z - target_p.z),
            ]
            result["actual_tcp"] = _pose_dict(actual_fk)
            result["target_tcp"] = _pose_dict(target_fk)
            result["tcp_position_error_actual_minus_target_m"] = delta
            result["tcp_position_error_norm_m"] = math.sqrt(
                sum(value * value for value in delta)
            )
            result["tcp_orientation_error_rad"] = _quaternion_angle_rad(
                actual_fk.pose.orientation, target_fk.pose.orientation
            )
        except Exception as exc:
            result["fk_error"] = str(exc)
        return result

    def _build_trace(self, origin_monotonic, target_positions):
        valid = []
        for received, message in self._trace_messages:
            current = dict(zip(message.name, message.position))
            if all(name in current for name in self.joint_names):
                valid.append((received, message, current))
        if not valid:
            return []

        selected = []
        next_offset = 0.0
        for entry in valid:
            offset = entry[0] - origin_monotonic
            if offset + 1e-9 < next_offset:
                continue
            selected.append(entry)
            next_offset += TRACE_PERIOD_SEC

        trace = []
        for index, (received, message, current) in enumerate(selected):
            actual = [float(current[name]) for name in self.joint_names]
            errors = [
                value - float(target)
                for value, target in zip(actual, target_positions)
            ]
            velocity_source = "joint_states"
            if len(message.velocity) == len(message.name):
                velocity_map = dict(zip(message.name, message.velocity))
                velocities = [
                    float(velocity_map[name]) for name in self.joint_names
                ]
            else:
                velocity_source = "finite_difference"
                if len(selected) == 1:
                    velocities = [None] * len(self.joint_names)
                else:
                    other_index = index - 1 if index > 0 else 1
                    other_received, _, other_current = selected[other_index]
                    dt = received - other_received
                    if abs(dt) < 1e-9:
                        velocities = [None] * len(self.joint_names)
                    else:
                        velocities = [
                            (float(current[name]) - float(other_current[name]))
                            / dt
                            for name in self.joint_names
                        ]
            absolute_errors = [abs(value) for value in errors]
            worst_index = max(
                range(len(absolute_errors)), key=absolute_errors.__getitem__
            )
            trace.append({
                "offset_sec": received - origin_monotonic,
                "joint_state_stamp": _stamp_dict(message.header.stamp),
                "actual_positions_rad": actual,
                "signed_errors_rad_actual_minus_target": errors,
                "velocities_rad_sec": velocities,
                "velocity_source": velocity_source,
                "maximum_absolute_error_rad": absolute_errors[worst_index],
                "worst_joint": self.joint_names[worst_index],
            })
        return trace

    def collect(
        self,
        target_positions,
        trajectory_file,
        trajectory_sha256,
        tolerance_rad,
        origin_label,
        origin_monotonic=None,
        frozen_target=None,
    ):
        if origin_monotonic is None:
            origin_monotonic = time.monotonic()
        if not self.fk_client.wait_for_service(timeout_sec=FK_TIMEOUT_SEC):
            self.node.get_logger().warn(
                "/compute_fk不可用；仍将保存七关节诊断，FK字段会记录错误"
            )

        samples = []
        self._trace_messages = []
        self._collect_trace = True
        try:
            for offset in SAMPLE_OFFSETS_SEC:
                sample = self._sample(
                    offset, origin_monotonic, target_positions
                )
                samples.append(sample)
                self.node.get_logger().info(
                    "ENDPOINT SAMPLE | "
                    f"t={sample['captured_offset_sec']:.3f}s, "
                    f"max_error="
                    f"{sample['maximum_absolute_error_rad']:.6f}rad, "
                    f"joint={sample['worst_joint']}, "
                    f"tcp_error="
                    f"{sample.get('tcp_position_error_norm_m', float('nan')):.6f}m"
                )
        finally:
            self._collect_trace = False
        trace = self._build_trace(origin_monotonic, target_positions)

        all_within = all(
            sample["maximum_absolute_error_rad"] <= tolerance_rad
            for sample in samples
        )
        report = {
            "schema_version": 1,
            "artifact_type": "g1_left_endpoint_joint_and_fk_diagnostics",
            "read_only": True,
            "origin_label": origin_label,
            "generated_unix_time": time.time(),
            "joint_names": self.joint_names,
            "sample_offsets_sec": list(SAMPLE_OFFSETS_SEC),
            "trace_period_requested_sec": TRACE_PERIOD_SEC,
            "trace_sample_count": len(trace),
            "endpoint_tolerance_rad": float(tolerance_rad),
            "all_samples_within_tolerance": all_within,
            "trajectory_file": str(trajectory_file),
            "trajectory_sha256": trajectory_sha256,
            "frozen_anygrasp_target": frozen_target,
            "samples": samples,
            "joint_trace": trace,
            "maximum_error_initial_rad": samples[0][
                "maximum_absolute_error_rad"
            ],
            "maximum_error_final_rad": samples[-1][
                "maximum_absolute_error_rad"
            ],
            "maximum_error_change_rad": (
                samples[-1]["maximum_absolute_error_rad"]
                - samples[0]["maximum_absolute_error_rad"]
            ),
        }
        encoded = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
        temporary = self.output_file.with_suffix(self.output_file.suffix + ".tmp")
        temporary.write_text(encoded, encoding="utf-8")
        temporary.replace(self.output_file)
        self.node.get_logger().info(
            f"ENDPOINT DIAGNOSTICS SAVED | {self.output_file}"
        )
        return report


def file_sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()
