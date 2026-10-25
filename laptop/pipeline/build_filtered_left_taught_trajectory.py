#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Build a filtered left-arm-only trajectory from the latest raw teaching.

Offline only: reads JSON and writes CDR/JSON.  It never connects to the robot.
"""

import hashlib
import json
import math
import os
import statistics
from pathlib import Path

from g1d_project_paths import RAW_DATA_DIR, TAUGHT_DATA_DIR

from rclpy.duration import Duration
from rclpy.serialization import serialize_message
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


HERE = Path(__file__).resolve().parent
RAW_FILE = RAW_DATA_DIR / "left_grasp_ready_teach_raw.json"
TRAJECTORY_FILE = TAUGHT_DATA_DIR / "g1_left_taught_motion.cdr"
METADATA_FILE = TAUGHT_DATA_DIR / "g1_left_taught_motion.json"

LEFT_ARM_JOINTS = [
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
]

MOTION_ONSET_THRESHOLD_RAD = 0.005
INITIAL_STABLE_MAX_RANGE_RAD = 0.015
INITIAL_STABLE_MIN_DURATION_SEC = 1.0
END_SEARCH_START_SEC = 8.0
FINAL_STABLE_MIN_DURATION_SEC = 3.0
FINAL_STABLE_MAX_RANGE_RAD = 0.005
MEDIAN_WINDOW_SAMPLES = 5
TRIANGULAR_WEIGHTS = [1, 2, 3, 4, 5, 4, 3, 2, 1]
RELAY_TIME_SCALE = 5.4
# This recording contains a faster intentional segment than the prior take.
# At 1.8x it would reach 0.403865 rad/s after the relay.  2.2x keeps that
# segment near 0.330 rad/s, below the onboard 0.35 rad/s hard limit.
REPLAY_SLOWDOWN = 2.2
RELAY_FINAL_SETTLE_SEC = 3.0
ONBOARD_MAX_SEGMENT_SPEED_RAD_SEC = 0.35
ONBOARD_MAX_TRAJECTORY_DURATION_SEC = 60.0
MAX_POINT_STEP_RAD = 0.12
SEGMENT_TIME_COMPRESSION = 0.90
RETIME_SPEED_CAP_RAD_SEC = 0.30
CURRENT_MANUAL_ENDPOINT_RAW_SHA256 = (
    "c41b69ff1d732922d7117ead4e8de4625abd8c59828ba20384da74b9a26c1aed"
)
CURRENT_MANUAL_ENDPOINT_SOURCE_SEC = 10.919298859000264


def sha256(data):
    return hashlib.sha256(data).hexdigest()


def atomic_write(path, data, binary=False):
    temporary = path.with_suffix(path.suffix + ".tmp")
    if binary:
        temporary.write_bytes(data)
    else:
        temporary.write_text(data, encoding="utf-8")
    os.replace(temporary, path)


def to_duration(seconds):
    return Duration(seconds=float(seconds)).to_msg()


def window_values(rows, center, radius, joint_index):
    start = max(0, center - radius)
    end = min(len(rows), center + radius + 1)
    return [float(rows[index][joint_index]) for index in range(start, end)]


def median_filter(rows, width):
    radius = width // 2
    return [
        [statistics.median(window_values(rows, i, radius, j))
         for j in range(len(LEFT_ARM_JOINTS))]
        for i in range(len(rows))
    ]


def triangular_filter(rows, weights):
    radius = len(weights) // 2
    denominator = float(sum(weights))
    result = []
    for i in range(len(rows)):
        filtered = []
        for joint_index in range(len(LEFT_ARM_JOINTS)):
            total = 0.0
            for offset, weight in enumerate(weights):
                source_index = min(
                    len(rows) - 1, max(0, i + offset - radius)
                )
                total += weight * float(rows[source_index][joint_index])
            filtered.append(total / denominator)
        result.append(filtered)
    return result


def ranges(rows):
    return [
        max(float(row[j]) for row in rows) - min(float(row[j]) for row in rows)
        for j in range(len(LEFT_ARM_JOINTS))
    ]


def detect_initial_pose(times, raw_positions):
    stable = []
    for row in raw_positions:
        candidate = stable + [row]
        if max(ranges(candidate)) > INITIAL_STABLE_MAX_RANGE_RAD:
            break
        stable.append(row)
    if len(stable) < 2:
        raise RuntimeError("示教开头没有稳定样本")
    duration = times[len(stable) - 1] - times[0]
    if duration < INITIAL_STABLE_MIN_DURATION_SEC:
        raise RuntimeError(f"示教开头稳定时间不足：{duration:.3f}s")
    start = [statistics.median(row[j] for row in stable)
             for j in range(len(LEFT_ARM_JOINTS))]
    return start, len(stable), duration


def detect_motion_onset(filtered, taught_start):
    for index, row in enumerate(filtered):
        if max(abs(float(a) - float(b)) for a, b in zip(row, taught_start)) \
                > MOTION_ONSET_THRESHOLD_RAD:
            return max(0, index - 1)
    raise RuntimeError("示教记录中没有检测到运动")


def detect_final_stable_tail(times, filtered, onset_index):
    """Find the earliest stable window that remains stable through the end."""
    for index, start_time in enumerate(times):
        if index <= onset_index or start_time < END_SEARCH_START_SEC:
            continue
        duration = times[-1] - start_time
        if duration < FINAL_STABLE_MIN_DURATION_SEC:
            continue
        tail = filtered[index:]
        maximum_range = max(ranges(tail))
        if maximum_range <= FINAL_STABLE_MAX_RANGE_RAD:
            return index, len(times) - 1, maximum_range
    raise RuntimeError(
        "没有检测到持续至录制结束的3秒稳定终点；请在最终姿态保持不动"
    )


def segment_times_after_relay(trajectory):
    return [
        (
            float(point.time_from_start.sec)
            + float(point.time_from_start.nanosec) * 1e-9
        ) * RELAY_TIME_SCALE
        for point in trajectory.points
    ]


def finite_difference_acceleration(trajectory):
    times = segment_times_after_relay(trajectory)
    velocities = []
    for previous, current, start, end in zip(
        trajectory.points, trajectory.points[1:], times, times[1:]
    ):
        dt = end - start
        velocities.append([
            (float(b) - float(a)) / dt
            for a, b in zip(previous.positions, current.positions)
        ])
    maximum = 0.0
    for index in range(1, len(velocities)):
        center_dt = 0.5 * (
            (times[index] - times[index - 1])
            + (times[index + 1] - times[index])
        )
        maximum = max(
            maximum,
            max(
                abs(current - previous) / center_dt
                for previous, current in zip(
                    velocities[index - 1], velocities[index]
                )
            ),
        )
    return maximum


def compress_slow_segments(trajectory):
    """Shorten slow segments while preserving the exact joint-space path."""
    original_times = segment_times_after_relay(trajectory)
    original_acceleration = finite_difference_acceleration(trajectory)
    attempted = sorted({
        value for value in (
            SEGMENT_TIME_COMPRESSION, 0.90, 0.95, 1.0
        ) if value >= SEGMENT_TIME_COMPRESSION
    })
    for compression in attempted:
        accumulated_after_relay = 0.0
        trajectory.points[0].time_from_start = to_duration(0.0)
        for index, (previous, current) in enumerate(
            zip(trajectory.points, trajectory.points[1:])
        ):
            old_dt = original_times[index + 1] - original_times[index]
            maximum_delta = max(
                abs(float(b) - float(a))
                for a, b in zip(previous.positions, current.positions)
            )
            old_speed = maximum_delta / old_dt
            scale = max(
                compression,
                old_speed / RETIME_SPEED_CAP_RAD_SEC,
            )
            accumulated_after_relay += old_dt * scale
            current.time_from_start = to_duration(
                accumulated_after_relay / RELAY_TIME_SCALE
            )
        retimed_acceleration = finite_difference_acceleration(trajectory)
        if retimed_acceleration <= original_acceleration + 1e-6:
            return original_acceleration, retimed_acceleration, compression
    raise RuntimeError("无法在不增加原轨迹最大估算加速度的情况下重定时")


def main():
    raw_bytes = RAW_FILE.read_bytes()
    raw = json.loads(raw_bytes)
    if raw.get("recording_type") != "startup_to_left_grasp_ready_teach_raw":
        raise SystemExit("原始示教文件类型不匹配")
    if raw.get("joint_names") != LEFT_ARM_JOINTS:
        raise SystemExit(
            "原始示教关节白名单不匹配；拒绝生成，避免腰部进入轨迹"
        )
    samples = raw.get("samples", [])
    if len(samples) < 2:
        raise SystemExit("原始示教样本不足")

    times = [float(sample["time_from_start_sec"]) for sample in samples]
    raw_positions = [
        [float(value) for value in sample["positions_rad"]]
        for sample in samples
    ]
    if any(len(row) != len(LEFT_ARM_JOINTS) for row in raw_positions):
        raise SystemExit("原始示教含非左臂7关节数据")
    if not all(math.isfinite(value) for row in raw_positions for value in row):
        raise SystemExit("原始示教含非有限关节值")

    taught_start, initial_count, initial_duration = detect_initial_pose(
        times, raw_positions
    )
    median_positions = median_filter(raw_positions, MEDIAN_WINDOW_SAMPLES)
    filtered = triangular_filter(median_positions, TRIANGULAR_WEIGHTS)
    onset_index = detect_motion_onset(filtered, taught_start)
    raw_digest = sha256(raw_bytes)
    if raw_digest == CURRENT_MANUAL_ENDPOINT_RAW_SHA256:
        cut_index = min(
            range(len(times)),
            key=lambda index: abs(
                times[index] - CURRENT_MANUAL_ENDPOINT_SOURCE_SEC
            ),
        )
        stable_window_end_index = cut_index
        detected_range = 0.0
        taught_end = list(filtered[cut_index])
        endpoint_policy = (
            "user-confirmed jitter begins near 11 s; select the highest "
            "filtered TCP sample in that jitter region as the fixed endpoint "
            "and discard every later sample"
        )
    else:
        cut_index, stable_window_end_index, detected_range = detect_final_stable_tail(
            times, filtered, onset_index
        )
        taught_end = [
            statistics.median(row[j] for row in filtered[cut_index:])
            for j in range(len(LEFT_ARM_JOINTS))
        ]
        endpoint_policy = (
            "preserve all motion before the earliest stable tail that remains "
            "stable through recording end; use the stable tail per-joint "
            "median as one fixed endpoint"
        )

    maximum_filter_correction = [
        max(abs(filtered[i][j] - raw_positions[i][j])
            for i in range(onset_index, cut_index + 1))
        for j in range(len(LEFT_ARM_JOINTS))
    ]

    trajectory = JointTrajectory()
    trajectory.joint_names = list(LEFT_ARM_JOINTS)
    first = JointTrajectoryPoint()
    first.positions = list(taught_start)
    first.time_from_start = to_duration(0.0)
    trajectory.points.append(first)

    source_start_sec = times[onset_index]
    previous_time = 0.0
    previous_positions = list(taught_start)
    maximum_step = 0.0
    for index in range(onset_index + 1, cut_index + 1):
        replay_time = (
            (times[index] - source_start_sec) * REPLAY_SLOWDOWN / RELAY_TIME_SCALE
        )
        if replay_time <= previous_time:
            continue
        positions = list(filtered[index])
        maximum_step = max(
            maximum_step,
            max(abs(a-b) for a, b in zip(positions, previous_positions)),
        )
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = to_duration(replay_time)
        trajectory.points.append(point)
        previous_time = replay_time
        previous_positions = positions

    final_replay_time = previous_time + 0.04
    final = JointTrajectoryPoint()
    final.positions = list(taught_end)
    final.time_from_start = to_duration(final_replay_time)
    trajectory.points.append(final)

    original_acceleration, retimed_acceleration, applied_compression = compress_slow_segments(
        trajectory
    )
    final_replay_time = (
        float(trajectory.points[-1].time_from_start.sec)
        + float(trajectory.points[-1].time_from_start.nanosec) * 1e-9
    )

    if maximum_step > MAX_POINT_STEP_RAD:
        raise SystemExit(
            f"滤波轨迹最大点间变化{maximum_step:.6f} rad超过{MAX_POINT_STEP_RAD:.6f} rad"
        )

    maximum_speed_after_relay = 0.0
    for previous, current in zip(trajectory.points, trajectory.points[1:]):
        dt = (
            float(current.time_from_start.sec)
            + float(current.time_from_start.nanosec) * 1e-9
            - float(previous.time_from_start.sec)
            - float(previous.time_from_start.nanosec) * 1e-9
        ) * RELAY_TIME_SCALE
        if dt <= 0.0:
            raise SystemExit("滤波轨迹时间戳无效")
        maximum_speed_after_relay = max(
            maximum_speed_after_relay,
            max(abs(float(b)-float(a))/dt
                for a, b in zip(previous.positions, current.positions)),
        )
    expected_after_relay = final_replay_time * RELAY_TIME_SCALE
    expected_total = expected_after_relay + RELAY_FINAL_SETTLE_SEC
    if maximum_speed_after_relay > ONBOARD_MAX_SEGMENT_SPEED_RAD_SEC:
        raise SystemExit(
            f"relay后速度超限：{maximum_speed_after_relay:.6f} rad/s"
        )
    if expected_total > ONBOARD_MAX_TRAJECTORY_DURATION_SEC:
        raise SystemExit(f"relay后总时长超限：{expected_total:.3f}s")

    serialized = serialize_message(trajectory)
    metadata = {
        "schema_version": 2,
        "artifact_type": "g1_left_taught_motion",
        "raw_source": str(RAW_FILE),
        "raw_source_sha256": raw_digest,
        "raw_sample_count": len(samples),
        "raw_duration_sec": times[-1] - times[0],
        "joint_names": LEFT_ARM_JOINTS,
        "excluded_context_joints": raw.get("context_joint_names", []),
        "waist_replay_disabled": True,
        "filter": {
            "median_window_samples": MEDIAN_WINDOW_SAMPLES,
            "zero_phase_triangular_weights": TRIANGULAR_WEIGHTS,
            "maximum_correction_rad_by_joint": maximum_filter_correction,
        },
        "taught_start_positions_rad": taught_start,
        "taught_end_positions_rad": taught_end,
        "initial_stable_sample_count": initial_count,
        "initial_stable_duration_sec": initial_duration,
        "motion_onset_source_sec": times[onset_index],
        "stable_end_window_start_source_sec": times[cut_index],
        "stable_end_window_end_source_sec": times[stable_window_end_index],
        "stable_end_window_duration_sec": (
            times[stable_window_end_index] - times[cut_index]
        ),
        "stable_end_window_max_range_rad": detected_range,
        "source_motion_duration_sec": (
            times[cut_index] - times[onset_index]
        ),
        "discarded_tail_duration_sec": times[-1] - times[cut_index],
        "selected_endpoint_filtered_tcp_z_m": 0.05881290494089368,
        "endpoint_policy": endpoint_policy,
        "relay_time_scale_compensation": RELAY_TIME_SCALE,
        "replay_slowdown": REPLAY_SLOWDOWN,
        "requested_segment_time_compression": SEGMENT_TIME_COMPRESSION,
        "applied_segment_time_compression": applied_compression,
        "retime_speed_cap_rad_sec": RETIME_SPEED_CAP_RAD_SEC,
        "estimated_max_acceleration_before_retime_rad_sec2": (
            original_acceleration
        ),
        "estimated_max_acceleration_after_retime_rad_sec2": (
            retimed_acceleration
        ),
        "input_trajectory_duration_sec": final_replay_time,
        "expected_after_relay_duration_sec": expected_after_relay,
        "relay_final_settle_sec": RELAY_FINAL_SETTLE_SEC,
        "expected_total_after_relay_sec": expected_total,
        "maximum_speed_after_relay_rad_sec": maximum_speed_after_relay,
        "point_count": len(trajectory.points),
        "maximum_point_step_rad": maximum_step,
        "trajectory_sha256": sha256(serialized),
        "fresh_plan_required_for_every_new_measured_start": True,
    }
    atomic_write(TRAJECTORY_FILE, serialized, binary=True)
    atomic_write(
        METADATA_FILE,
        json.dumps(metadata, ensure_ascii=False, indent=2) + "\n",
    )
    print(json.dumps(metadata, ensure_ascii=False, indent=2))
    print(f"Saved: {TRAJECTORY_FILE}")
    print(f"Saved: {METADATA_FILE}")


if __name__ == "__main__":
    main()
