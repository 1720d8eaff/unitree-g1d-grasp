#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Add the configured physical work table to MoveIt's PlanningScene.

This node only updates collision geometry. It never plans or executes motion.
It refuses unverified geometry profiles, including the initial 15 cm profile.
"""

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

# Match the laptop's existing G1-D ROS graph before importing rclpy.
os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
os.environ["CYCLONEDDS_URI"] = "/home/yii/cyclonedds_ws/cyclonedds.xml"
os.environ["ROS_DOMAIN_ID"] = "0"
os.environ["ROS_LOCALHOST_ONLY"] = "0"

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Pose
from moveit_msgs.msg import (
    CollisionObject,
    ObjectColor,
    PlanningScene,
    PlanningSceneComponents,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_PROFILE = PROJECT_ROOT / "config" / "active_geometry.json"
POSITION_VERIFY_TOLERANCE_M = 0.002
ORIENTATION_VERIFY_TOLERANCE_RAD = 0.01


def normalized_quaternion(value):
    result = (
        float(value.x),
        float(value.y),
        float(value.z),
        float(value.w),
    )
    norm = math.sqrt(sum(component * component for component in result))
    if norm < 1e-12:
        raise RuntimeError("PlanningScene返回了无效的零四元数")
    return tuple(component / norm for component in result)


def multiply_quaternions(left, right):
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def rotate_vector(quaternion, vector):
    x, y, z, w = quaternion
    vx, vy, vz = vector
    # Rotation matrix generated from a normalized xyzw quaternion.
    return (
        (1 - 2 * (y * y + z * z)) * vx
        + 2 * (x * y - z * w) * vy
        + 2 * (x * z + y * w) * vz,
        2 * (x * y + z * w) * vx
        + (1 - 2 * (x * x + z * z)) * vy
        + 2 * (y * z - x * w) * vz,
        2 * (x * z - y * w) * vx
        + 2 * (y * z + x * w) * vy
        + (1 - 2 * (x * x + y * y)) * vz,
    )


def compose_pose(parent_position, parent_orientation, child_pose):
    child_position = (
        float(child_pose.position.x),
        float(child_pose.position.y),
        float(child_pose.position.z),
    )
    rotated = rotate_vector(parent_orientation, child_position)
    position = tuple(
        float(parent_position[index]) + rotated[index]
        for index in range(3)
    )
    orientation = multiply_quaternions(
        parent_orientation,
        normalized_quaternion(child_pose.orientation),
    )
    return position, orientation


def orientation_distance(left, right):
    dot = abs(sum(a * b for a, b in zip(left, right)))
    return 2.0 * math.acos(max(-1.0, min(1.0, dot)))


def load_profile(path):
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    if not data.get("verified", False):
        raise RuntimeError(
            "几何配置尚未现场确认，拒绝加入桌面碰撞体："
            f"{data.get('profile_id', path)}"
        )
    table = data.get("work_table", {})
    required = ("id", "frame_id", "size_xyz_m", "center_xyz_m")
    missing = [key for key in required if key not in table]
    if missing:
        raise RuntimeError(f"几何配置缺少字段：{missing}")
    size = tuple(float(value) for value in table["size_xyz_m"])
    center = tuple(float(value) for value in table["center_xyz_m"])
    if len(size) != 3 or len(center) != 3:
        raise RuntimeError("桌面size_xyz_m和center_xyz_m必须各包含3个数")
    if any(value <= 0.0 for value in size):
        raise RuntimeError("桌面尺寸必须为正数")
    return data, table, size, center


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--profile", type=Path, default=DEFAULT_PROFILE,
        help="经过现场确认的几何JSON配置",
    )
    return parser.parse_args()


class AddTableCollision(Node):
    def __init__(self, profile, table, table_size, table_center):
        super().__init__("add_g1_table_collision")
        self.profile = profile
        self.table = table
        self.table_size = table_size
        self.table_center = table_center
        self.apply_client = self.create_client(
            ApplyPlanningScene, "/apply_planning_scene"
        )
        self.get_client = self.create_client(
            GetPlanningScene, "/get_planning_scene"
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer, self, spin_thread=False
        )

    def make_scene(self):
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = list(self.table_size)

        pose = Pose()
        pose.position.x = self.table_center[0]
        pose.position.y = self.table_center[1]
        pose.position.z = self.table_center[2]
        pose.orientation.w = 1.0

        table = CollisionObject()
        table.id = self.table["id"]
        table.header.frame_id = self.table["frame_id"]
        table.primitives.append(box)
        table.primitive_poses.append(pose)
        table.operation = CollisionObject.ADD

        color = ObjectColor()
        color.id = self.table["id"]
        color.color.r = 0.55
        color.color.g = 0.34
        color.color.b = 0.18
        color.color.a = 0.75

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.world.collision_objects.append(table)
        scene.object_colors.append(color)
        return scene

    def apply(self):
        if not self.apply_client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(
                "/apply_planning_scene 不可用；请先启动笔记本 start_g1_control"
            )

        request = ApplyPlanningScene.Request()
        request.scene = self.make_scene()
        future = self.apply_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError("MoveIt 拒绝添加桌面碰撞体")

    def verify(self):
        if not self.get_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(
                "桌面已提交，但 /get_planning_scene 不可用，无法回读验证"
            )

        request = GetPlanningScene.Request()
        request.components.components = (
            PlanningSceneComponents.WORLD_OBJECT_NAMES
            | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        )
        future = self.get_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        response = future.result()
        if response is None:
            raise RuntimeError("无法回读 PlanningScene")

        objects = {
            obj.id: obj for obj in response.scene.world.collision_objects
        }
        actual = objects.get(self.table["id"])
        if actual is None:
            raise RuntimeError(
                "PlanningScene 回读中没有 "
                f"{self.table['id']!r}；当前对象: {sorted(objects)}"
            )
        if not actual.primitives or not actual.primitive_poses:
            raise RuntimeError("PlanningScene桌面没有完整几何")
        if actual.primitives[0].type != SolidPrimitive.BOX:
            raise RuntimeError("PlanningScene桌面不是BOX几何")
        dimensions = tuple(float(v) for v in actual.primitives[0].dimensions)
        if any(abs(a - b) > 1e-6 for a, b in zip(dimensions, self.table_size)):
            raise RuntimeError("PlanningScene桌面尺寸与配置不一致")

        actual_position, actual_orientation = compose_pose(
            (
                float(actual.pose.position.x),
                float(actual.pose.position.y),
                float(actual.pose.position.z),
            ),
            normalized_quaternion(actual.pose.orientation),
            actual.primitive_poses[0],
        )

        expected_pose = Pose()
        expected_pose.position.x = self.table_center[0]
        expected_pose.position.y = self.table_center[1]
        expected_pose.position.z = self.table_center[2]
        expected_pose.orientation.w = 1.0
        storage_frame = actual.header.frame_id
        configured_frame = self.table["frame_id"]
        if storage_frame == configured_frame:
            expected_position = self.table_center
            expected_orientation = (0.0, 0.0, 0.0, 1.0)
        else:
            deadline = time.monotonic() + 3.0
            transform_stamped = None
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.1)
                if self.tf_buffer.can_transform(
                    storage_frame, configured_frame, Time()
                ):
                    transform_stamped = self.tf_buffer.lookup_transform(
                        storage_frame, configured_frame, Time()
                    )
                    break
            if transform_stamped is None:
                raise RuntimeError(
                    "PlanningScene将桌面规范化到"
                    f"{storage_frame!r}，但3秒内无法读取"
                    f"{configured_frame!r}到该坐标系的TF"
                )
            try:
                transform = transform_stamped.transform
            except Exception as exc:
                raise RuntimeError(
                    "PlanningScene将桌面规范化到"
                    f"{storage_frame!r}，但无法读取"
                    f"{configured_frame!r}到该坐标系的TF：{exc}"
                ) from exc
            expected_position, expected_orientation = compose_pose(
                (
                    float(transform.translation.x),
                    float(transform.translation.y),
                    float(transform.translation.z),
                ),
                normalized_quaternion(transform.rotation),
                expected_pose,
            )

        position_error = math.sqrt(sum(
            (actual - expected) ** 2
            for actual, expected in zip(actual_position, expected_position)
        ))
        orientation_error = orientation_distance(
            actual_orientation, expected_orientation
        )
        if position_error > POSITION_VERIFY_TOLERANCE_M:
            raise RuntimeError(
                "PlanningScene桌面空间位置与配置不一致："
                f"storage_frame={storage_frame}, "
                f"position_error={position_error:.6f}m"
            )
        if orientation_error > ORIENTATION_VERIFY_TOLERANCE_RAD:
            raise RuntimeError(
                "PlanningScene桌面空间姿态与配置不一致："
                f"storage_frame={storage_frame}, "
                f"orientation_error={orientation_error:.6f}rad"
            )
        return storage_frame, position_error, orientation_error


def main():
    args = parse_args()
    profile, table, table_size, table_center = load_profile(args.profile)
    rclpy.init()
    node = AddTableCollision(profile, table, table_size, table_center)
    try:
        node.apply()
        storage_frame, position_error, orientation_error = node.verify()
        print("TABLE_COLLISION_READY")
        print(f"profile_id={profile['profile_id']}")
        print(f"id={table['id']}")
        print(f"frame={table['frame_id']}")
        print(f"size_xyz_m={list(table_size)}")
        print(f"center_xyz_m={list(table_center)}")
        print(f"planning_scene_storage_frame={storage_frame}")
        print(f"position_verify_error_m={position_error:.6f}")
        print(f"orientation_verify_error_rad={orientation_error:.6f}")
        return_code = 0
    except Exception as exc:
        node.get_logger().error(str(exc))
        return_code = 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(return_code)


if __name__ == "__main__":
    main()
