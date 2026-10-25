#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Read-only full manual-teach recorder with second-segment extraction.

The node subscribes to joint, AnyGrasp target, and backend-state topics.  It
uses MoveIt's read-only FK service after recording.  It has no publisher,
action client, ARM/DISARM client, or hand command.
"""

import argparse
import copy
import json
import math
import os
import statistics
import threading
import time
from datetime import datetime, timezone

from g1d_project_paths import RUN_DIR, TAUGHT_DATA_DIR, ensure_run_dir


os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
os.environ.setdefault(
    "CYCLONEDDS_URI", "/home/yii/cyclonedds_ws/cyclonedds.xml"
)
os.environ.setdefault("ROS_DOMAIN_ID", "0")
os.environ.setdefault("ROS_LOCALHOST_ONLY", "0")

import rclpy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionFK
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String


LEFT_ARM_JOINTS = [
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
]
CONTEXT_JOINTS = ["Yaw_Joint", "torso_Joint"]
TARGET_TOPIC = "/anygrasp/left_target_candidate_a"
BACKEND_STATE_TOPIC = "/g1_d/trajectory_backend_state"
FK_SERVICE = "/compute_fk"
FK_FRAME = "torso_link"
FK_LINK = "left_grasp_tcp"
RAW_FILE = RUN_DIR / "left_planner_calibration_teach_raw.json"
REPORT_FILE = RUN_DIR / "left_planner_calibration_teach_report.json"
TAUGHT_METADATA_FILE = TAUGHT_DATA_DIR / "g1_left_taught_motion.json"
MIN_DIRECTION_STEP_RAD = 0.0005
ENDPOINT_WINDOW_SEC = 1.0
MAX_FK_SAMPLES = 31


def utc_now_iso():
    return datetime.now(timezone.utc).isoformat()


def atomic_write_json(path, payload):
    if path.exists():
        raise RuntimeError(f"拒绝覆盖已有记录：{path}")
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary, path)


def pose_dict(pose):
    p = pose.pose.position
    q = pose.pose.orientation
    return {
        "frame_id": pose.header.frame_id,
        "position_m": [float(p.x), float(p.y), float(p.z)],
        "quaternion_xyzw": [float(q.x), float(q.y), float(q.z), float(q.w)],
    }


def quaternion_angle(a, b):
    dot = abs(
        float(a.x) * float(b.x)
        + float(a.y) * float(b.y)
        + float(a.z) * float(b.z)
        + float(a.w) * float(b.w)
    )
    norm_a = math.sqrt(a.x*a.x + a.y*a.y + a.z*a.z + a.w*a.w)
    norm_b = math.sqrt(b.x*b.x + b.y*b.y + b.z*b.z + b.w*b.w)
    if norm_a < 1e-12 or norm_b < 1e-12:
        return None
    return 2.0 * math.acos(max(-1.0, min(1.0, dot / (norm_a * norm_b))))


def approach_axis(quaternion):
    x, y, z, w = (
        float(quaternion.x),
        float(quaternion.y),
        float(quaternion.z),
        float(quaternion.w),
    )
    norm_sq = x*x + y*y + z*z + w*w
    if norm_sq < 1e-12:
        return None
    return [
        1.0 - 2.0 * (y*y + z*z) / norm_sq,
        2.0 * (x*y + z*w) / norm_sq,
        2.0 * (x*z - y*w) / norm_sq,
    ]


def approach_axis_angle(a, b):
    axis_a = approach_axis(a)
    axis_b = approach_axis(b)
    if axis_a is None or axis_b is None:
        return None
    dot = sum(x*y for x, y in zip(axis_a, axis_b))
    return math.acos(max(-1.0, min(1.0, dot)))


class CalibrationTeachRecorder(Node):
    def __init__(self, sample_hz):
        super().__init__("left_planner_calibration_teach_recorder")
        self.sample_period = 1.0 / sample_hz
        self.recording = False
        self.started_monotonic = None
        self.started_utc = None
        self.last_sample_monotonic = None
        self.latest_joint_state = None
        self.latest_target = None
        self.target_at_start = None
        self.latest_backend_state = None
        self.samples = []
        self.invalid_messages = 0
        self.create_subscription(JointState, "/joint_states", self.on_joint, 100)
        self.create_subscription(PoseStamped, TARGET_TOPIC, self.on_target, 20)
        self.create_subscription(
            String, BACKEND_STATE_TOPIC, self.on_backend_state, 20
        )
        self.fk_client = self.create_client(GetPositionFK, FK_SERVICE)
        self.get_logger().warn(
            "READ-ONLY MANUAL TEACH: no publisher, action, ARM/DISARM, "
            "trajectory, or hand command exists in this recorder"
        )

    def on_target(self, message):
        self.latest_target = copy.deepcopy(message)

    def on_backend_state(self, message):
        try:
            self.latest_backend_state = json.loads(message.data)
        except (TypeError, ValueError):
            self.latest_backend_state = {"raw": message.data}

    def on_joint(self, message):
        if len(message.name) != len(message.position):
            self.invalid_messages += 1
            return
        values = dict(zip(message.name, message.position))
        if any(name not in values for name in LEFT_ARM_JOINTS):
            self.invalid_messages += 1
            return
        self.latest_joint_state = copy.deepcopy(message)
        if not self.recording:
            return
        now = time.monotonic()
        if (
            self.last_sample_monotonic is not None
            and now - self.last_sample_monotonic < self.sample_period
        ):
            return
        positions = [float(values[name]) for name in LEFT_ARM_JOINTS]
        if not all(math.isfinite(value) for value in positions):
            self.invalid_messages += 1
            return
        velocity_map = {}
        if len(message.velocity) == len(message.name):
            velocity_map = dict(zip(message.name, message.velocity))
        velocities = [
            float(velocity_map[name]) if name in velocity_map else None
            for name in LEFT_ARM_JOINTS
        ]
        context = {
            name: float(values[name])
            for name in CONTEXT_JOINTS
            if name in values and math.isfinite(float(values[name]))
        }
        self.samples.append({
            "time_from_start_sec": now - self.started_monotonic,
            "positions_rad": positions,
            "velocities_rad_sec": velocities,
            "context_positions_rad": context,
            "ros_stamp": {
                "sec": int(message.header.stamp.sec),
                "nanosec": int(message.header.stamp.nanosec),
            },
        })
        self.last_sample_monotonic = now
        if len(self.samples) % 250 == 0:
            self.get_logger().info(
                f"已记录{len(self.samples)}点，"
                f"时长={self.samples[-1]['time_from_start_sec']:.2f}s"
            )

    def wait_for_state(self, timeout_sec=5.0):
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.latest_joint_state is not None:
                return
        raise RuntimeError("5秒内没有收到完整左臂/joint_states")

    def start(self):
        self.samples = []
        self.started_monotonic = time.monotonic()
        self.started_utc = utc_now_iso()
        self.last_sample_monotonic = None
        self.target_at_start = copy.deepcopy(self.latest_target)
        self.recording = True

    def stop(self):
        self.recording = False

    def state_with_positions(self, positions):
        state = copy.deepcopy(self.latest_joint_state)
        index = {name: i for i, name in enumerate(state.name)}
        values = list(state.position)
        for name, value in zip(LEFT_ARM_JOINTS, positions):
            values[index[name]] = float(value)
        state.position = values
        if state.velocity:
            state.velocity = [0.0] * len(state.name)
        if state.effort:
            state.effort = [0.0] * len(state.name)
        return state

    def compute_fk(self, positions):
        request = GetPositionFK.Request()
        request.header.frame_id = FK_FRAME
        request.fk_link_names = [FK_LINK]
        request.robot_state = RobotState()
        request.robot_state.joint_state = self.state_with_positions(positions)
        future = self.fk_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        response = future.result()
        if response is None or int(response.error_code.val) != 1:
            code = None if response is None else int(response.error_code.val)
            raise RuntimeError(f"/compute_fk失败：error_code={code}")
        if len(response.pose_stamped) != 1:
            raise RuntimeError("/compute_fk没有返回唯一left_grasp_tcp")
        return response.pose_stamped[0]


def direction_change_count(values):
    last_sign = 0
    changes = 0
    for previous, current in zip(values, values[1:]):
        delta = current - previous
        if abs(delta) < MIN_DIRECTION_STEP_RAD:
            continue
        sign = 1 if delta > 0.0 else -1
        if last_sign and sign != last_sign:
            changes += 1
        last_sign = sign
    return changes


def joint_metrics(samples):
    result = {}
    times = [float(sample["time_from_start_sec"]) for sample in samples]
    endpoint_start = max(times[0], times[-1] - ENDPOINT_WINDOW_SEC)
    endpoint_rows = [
        sample for sample in samples
        if float(sample["time_from_start_sec"]) >= endpoint_start
    ]
    for joint_index, name in enumerate(LEFT_ARM_JOINTS):
        values = [
            float(sample["positions_rad"][joint_index]) for sample in samples
        ]
        endpoint_values = [
            float(sample["positions_rad"][joint_index])
            for sample in endpoint_rows
        ]
        steps = [abs(b-a) for a, b in zip(values, values[1:])]
        segment_speeds = [
            abs(values[i] - values[i-1]) / max(times[i] - times[i-1], 1e-9)
            for i in range(1, len(values))
        ]
        reported_speeds = []
        for sample in samples:
            value = sample["velocities_rad_sec"][joint_index]
            if value is not None and math.isfinite(float(value)):
                reported_speeds.append(abs(float(value)))
        total_variation = sum(steps)
        direct_change = abs(values[-1] - values[0])
        endpoint = statistics.median(endpoint_values)
        result[name] = {
            "start_rad": values[0],
            "end_last_sample_rad": values[-1],
            "endpoint_median_rad": endpoint,
            "endpoint_median_deg": math.degrees(endpoint),
            "signed_change_rad": values[-1] - values[0],
            "minimum_rad": min(values),
            "maximum_rad": max(values),
            "range_rad": max(values) - min(values),
            "total_variation_rad": total_variation,
            "reversal_detour_rad": max(0.0, total_variation-direct_change),
            "direction_change_count_noise_filtered": direction_change_count(values),
            "maximum_sample_step_rad": max(steps, default=0.0),
            "maximum_finite_difference_speed_rad_sec": max(
                segment_speeds, default=0.0
            ),
            "maximum_reported_speed_rad_sec": (
                max(reported_speeds) if reported_speeds else None
            ),
            "endpoint_window_range_rad": (
                max(endpoint_values) - min(endpoint_values)
            ),
        }
    return result


def selected_indices(count):
    if count <= MAX_FK_SAMPLES:
        return list(range(count))
    return sorted({
        round(index * (count - 1) / float(MAX_FK_SAMPLES - 1))
        for index in range(MAX_FK_SAMPLES)
    })


def load_taught_endpoint():
    metadata = json.loads(TAUGHT_METADATA_FILE.read_text(encoding="utf-8"))
    if metadata.get("artifact_type") != "g1_left_taught_motion":
        raise RuntimeError("现有第一段施教元数据类型错误")
    joint_names = list(metadata.get("joint_names", LEFT_ARM_JOINTS))
    if joint_names != LEFT_ARM_JOINTS:
        raise RuntimeError("现有第一段施教关节顺序错误")
    endpoint = [
        float(value) for value in metadata["taught_end_positions_rad"]
    ]
    if len(endpoint) != len(LEFT_ARM_JOINTS):
        raise RuntimeError("现有第一段施教终点长度错误")
    return endpoint, metadata.get("trajectory_sha256")


def split_at_taught_endpoint(samples, taught_endpoint):
    scored = []
    for index, sample in enumerate(samples):
        errors = [
            abs(float(actual) - float(expected))
            for actual, expected in zip(
                sample["positions_rad"], taught_endpoint
            )
        ]
        scored.append((max(errors), sum(value*value for value in errors), index))
    maximum_error, squared_error, index = min(scored)
    if len(samples) - index < 5:
        raise RuntimeError(
            "整段记录在现有第一段终点之后不足5个样本；"
            "请完整移动到最终抓取点后再停止"
        )
    confidence = (
        "high" if maximum_error <= 0.02
        else "review" if maximum_error <= 0.08
        else "poor"
    )
    return index, {
        "sample_index": index,
        "time_from_start_sec": samples[index]["time_from_start_sec"],
        "maximum_joint_error_rad": maximum_error,
        "sum_squared_joint_error_rad2": squared_error,
        "confidence": confidence,
        "automatic_gate": False,
    }


def tcp_analysis(node, samples, endpoint_positions):
    if not node.fk_client.wait_for_service(timeout_sec=5.0):
        return {"error": "/compute_fk不可用"}
    try:
        trace = []
        for index in selected_indices(len(samples)):
            pose = node.compute_fk(samples[index]["positions_rad"])
            trace.append({
                "sample_index": index,
                "time_from_start_sec": samples[index]["time_from_start_sec"],
                "tcp": pose_dict(pose),
            })
        endpoint_pose = node.compute_fk(endpoint_positions)
        path_length = 0.0
        for previous, current in zip(trace, trace[1:]):
            a = previous["tcp"]["position_m"]
            b = current["tcp"]["position_m"]
            path_length += math.sqrt(sum((y-x)**2 for x, y in zip(a, b)))
        start = trace[0]["tcp"]["position_m"]
        end = trace[-1]["tcp"]["position_m"]
        direct = math.sqrt(sum((y-x)**2 for x, y in zip(start, end)))
        result = {
            "fk_frame": FK_FRAME,
            "fk_link": FK_LINK,
            "sampled_trace": trace,
            "sampled_path_length_m": path_length,
            "sampled_direct_displacement_m": direct,
            "sampled_position_detour_m": max(0.0, path_length-direct),
            "endpoint_median_tcp": pose_dict(endpoint_pose),
            "endpoint_approach_axis_xyz": approach_axis(
                endpoint_pose.pose.orientation
            ),
        }
        if node.latest_target is not None:
            target = node.latest_target
            p = endpoint_pose.pose.position
            t = target.pose.position
            result["latest_anygrasp_target"] = pose_dict(target)
            result["endpoint_tcp_minus_target_position_m"] = [
                float(p.x-t.x), float(p.y-t.y), float(p.z-t.z)
            ]
            result["endpoint_to_target_orientation_rad"] = quaternion_angle(
                endpoint_pose.pose.orientation, target.pose.orientation
            )
            result["endpoint_to_target_approach_axis_rad"] = (
                approach_axis_angle(
                    endpoint_pose.pose.orientation, target.pose.orientation
                )
            )
        return result
    except Exception as exc:
        return {"error": str(exc)}


def build_outputs(node, sample_hz):
    samples = node.samples
    if len(samples) < 5:
        raise RuntimeError("有效样本少于5个，不生成校准记录")
    duration = float(samples[-1]["time_from_start_sec"])
    if duration <= 0.0:
        raise RuntimeError("录制时长无效")
    taught_endpoint, taught_sha256 = load_taught_endpoint()
    split_index, split_diagnostics = split_at_taught_endpoint(
        samples, taught_endpoint
    )
    second_segment_samples = copy.deepcopy(samples[split_index:])
    second_start_time = float(
        second_segment_samples[0]["time_from_start_sec"]
    )
    for sample in second_segment_samples:
        sample["time_from_start_sec"] = (
            float(sample["time_from_start_sec"]) - second_start_time
        )
    full_metrics = joint_metrics(samples)
    second_metrics = joint_metrics(second_segment_samples)
    endpoint_positions = [
        second_metrics[name]["endpoint_median_rad"] for name in LEFT_ARM_JOINTS
    ]
    raw = {
        "schema_version": 1,
        "recording_type": "left_full_motion_for_second_segment_calibration_raw",
        "read_only_recorder": True,
        "program_contains_arm_disarm_calls": False,
        "program_contains_motion_or_hand_publishers": False,
        "started_utc": node.started_utc,
        "finished_utc": utc_now_iso(),
        "source_topic": "/joint_states",
        "requested_sample_hz": sample_hz,
        "joint_names": LEFT_ARM_JOINTS,
        "sample_count": len(samples),
        "duration_sec": duration,
        "invalid_message_count": node.invalid_messages,
        "backend_state_observed": node.latest_backend_state,
        "anygrasp_target_at_start": (
            pose_dict(node.target_at_start) if node.target_at_start else None
        ),
        "anygrasp_target_at_end": (
            pose_dict(node.latest_target) if node.latest_target else None
        ),
        "samples": samples,
    }
    report = {
        "schema_version": 1,
        "analysis_type": "left_full_motion_with_second_segment_calibration_report",
        "scope": "full_manual_recording_original_taught_path_unchanged",
        "source_file": str(RAW_FILE),
        "joint_names": LEFT_ARM_JOINTS,
        "sample_count": len(samples),
        "duration_sec": duration,
        "effective_sample_hz": (len(samples)-1) / duration,
        "endpoint_window_sec": ENDPOINT_WINDOW_SEC,
        "existing_taught_reference": {
            "metadata_file": str(TAUGHT_METADATA_FILE),
            "trajectory_sha256": taught_sha256,
            "taught_endpoint_positions_rad": taught_endpoint,
        },
        "detected_first_to_second_segment_seam": split_diagnostics,
        "full_motion_joint_metrics": full_metrics,
        "second_segment_joint_metrics": second_metrics,
        "full_motion_tcp_analysis": tcp_analysis(
            node, samples, endpoint_positions
        ),
        "second_segment_tcp_analysis": tcp_analysis(
            node, second_segment_samples, endpoint_positions
        ),
        "planner_review_values": {
            "use_second_segment_metrics_only": True,
            "manual_endpoint_positions_rad": endpoint_positions,
            "manual_left_wrist_yaw_endpoint_rad": second_metrics[
                "left_wrist_yaw_joint"
            ]["endpoint_median_rad"],
            "manual_left_wrist_yaw_endpoint_deg": second_metrics[
                "left_wrist_yaw_joint"
            ]["endpoint_median_deg"],
            "observed_reversal_detour_rad_by_joint": {
                name: second_metrics[name]["reversal_detour_rad"]
                for name in LEFT_ARM_JOINTS
            },
            "observed_maximum_sample_step_rad_by_joint": {
                name: second_metrics[name]["maximum_sample_step_rad"]
                for name in LEFT_ARM_JOINTS
            },
            "automatic_code_or_gate_changes_applied": False,
        },
    }
    return raw, report


def parse_args():
    parser = argparse.ArgumentParser(
        description="只读记录下垂到抓取点全程，并自动提取AnyGrasp第二段"
    )
    parser.add_argument("--sample-hz", type=float, default=50.0)
    parser.add_argument("--max-duration-sec", type=float, default=120.0)
    return parser.parse_args()


def main():
    args = parse_args()
    if not 10.0 <= args.sample_hz <= 100.0:
        raise SystemExit("--sample-hz必须在10~100Hz")
    if not 5.0 <= args.max_duration_sec <= 300.0:
        raise SystemExit("--max-duration-sec必须在5~300秒")
    ensure_run_dir()
    rclpy.init()
    node = CalibrationTeachRecorder(args.sample_hz)
    stop_event = threading.Event()
    failure = None
    try:
        node.wait_for_state()
        print(
            "\n本程序不会修改原施教轨迹。请让手臂处于下垂起点，"
            "保持轨迹控制DISARMED；准备好后按回车开始记录全程。",
            flush=True,
        )
        input()
        node.start()
        print(
            "记录已开始：请从下垂状态缓慢手动施教到正确TCP抓取点，"
            "终点保持至少1秒，"
            "再按回车结束。",
            flush=True,
        )

        def wait_for_enter():
            try:
                input()
            except EOFError:
                pass
            stop_event.set()

        threading.Thread(target=wait_for_enter, daemon=True).start()
        deadline = time.monotonic() + args.max_duration_sec
        while rclpy.ok() and not stop_event.is_set():
            rclpy.spin_once(node, timeout_sec=0.05)
            if time.monotonic() >= deadline:
                node.get_logger().warn("达到最大录制时长，自动停止")
                break
        node.stop()
        for _ in range(5):
            rclpy.spin_once(node, timeout_sec=0.05)
        raw, report = build_outputs(node, args.sample_hz)
        atomic_write_json(RAW_FILE, raw)
        atomic_write_json(REPORT_FILE, report)
        wrist = report["planner_review_values"]
        print(f"\nCALIBRATION_TEACH_COMPLETE: {RUN_DIR}")
        print(f"raw={RAW_FILE}")
        print(f"report={REPORT_FILE}")
        print(
            "manual_left_wrist_yaw_endpoint="
            f"{wrist['manual_left_wrist_yaw_endpoint_rad']:+.6f}rad/"
            f"{wrist['manual_left_wrist_yaw_endpoint_deg']:+.3f}deg"
        )
    except KeyboardInterrupt:
        node.stop()
        failure = RuntimeError("用户中断；未保存不完整记录")
    except Exception as exc:
        node.stop()
        failure = exc
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if failure is not None:
        raise SystemExit(str(failure))


if __name__ == "__main__":
    main()
