#!/usr/bin/env python3
import os
import math
import sys
from pathlib import Path

ANYGRASP_SDK_ROOT = Path(
    os.environ.get(
        "ANYGRASP_SDK_ROOT", "/home/yii/anygrasp_sdk/grasp_detection"
    )
).expanduser()
_sdk_root = str(ANYGRASP_SDK_ROOT)
# The AnyGrasp Conda environment places gsnet_versions before the SDK root on
# PYTHONPATH.  Merely checking for the root is therefore insufficient: promote
# it to index zero so the licensed root build is selected deterministically.
sys.path[:] = [_sdk_root] + [entry for entry in sys.path if entry != _sdk_root]

import zmq
import numpy as np
import open3d as o3d
import gsnet
from gsnet import AnyGrasp

print(f"AnyGrasp SDK module: {Path(gsnet.__file__).resolve()}", flush=True)

class Args:
    def __init__(self):
        self.checkpoint_path = os.environ.get(
            "ANYGRASP_CHECKPOINT",
            "/home/yii/下载/checkpoint_detection.tar",
        )
        self.max_gripper_width = 0.15
        self.gripper_height = 0.03
        self.top_down_grasp = False
        self.debug = False

SHOW_VISUALIZATION = os.environ.get(
    "ANYGRASP_SHOW_VISUALIZATION", "1"
).strip().lower() in {"1", "true", "yes", "on"}
DESIRED_TILT_DEG = 35.0
MIN_TILT_DEG = 20.0
MAX_TILT_DEG = 55.0
TOP_N = 30
MAX_TRACK_POS_JUMP_M = 0.080
MAX_TRACK_ROT_JUMP_DEG = 45.0
RESEED_AFTER_MISSES = 12
POS_ALPHA = 0.30
ROT_ALPHA = 0.22

R_TORSO_CAMERA = np.array([
    [-0.018012825, -0.734779856, +0.678066443],
    [-0.999829739, +0.010521744, -0.015158680],
    [+0.004003851, -0.678224045, -0.734844278],
], dtype=np.float64)

cfgs = Args()
print("Loading AnyGrasp on RTX 4060...", flush=True)
anygrasp = AnyGrasp(cfgs)
anygrasp.load_net()

context = zmq.Context()
socket = context.socket(zmq.REP)
socket.bind("tcp://0.0.0.0:5555")

lims = [-0.8, 0.8, -0.8, 0.8, 0.0, 1.8]
print("AnyGrasp slanted/stable server ready.", flush=True)
print(
    f"Preferred tilt: {DESIRED_TILT_DEG:.0f} deg, "
    f"allowed {MIN_TILT_DEG:.0f}..{MAX_TILT_DEG:.0f} deg",
    flush=True,
)

vis = None
pcd = None
old_gripper_geoms = []
is_first_frame = True

if SHOW_VISUALIZATION:
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="AnyGrasp - slanted stable grasp", width=1280, height=720)
    pcd = o3d.geometry.PointCloud()
    coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
    vis.add_geometry(pcd)
    vis.add_geometry(coord_frame)

def rotation_angle_deg(R1, R2):
    dR = R1.T @ R2
    c = (np.trace(dR) - 1.0) / 2.0
    c = float(np.clip(c, -1.0, 1.0))
    return math.degrees(math.acos(c))

def R_to_quat_wxyz(R):
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
    q = np.array([qw, qx, qy, qz], dtype=np.float64)
    q /= np.linalg.norm(q)
    return q

def quat_wxyz_to_R(q):
    q = np.asarray(q, dtype=np.float64)
    q /= np.linalg.norm(q)
    w, x, y, z = q
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x*x + y*y)],
    ], dtype=np.float64)

def slerp_R(R0, R1, alpha):
    q0 = R_to_quat_wxyz(R0)
    q1 = R_to_quat_wxyz(R1)
    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    dot = float(np.clip(dot, -1.0, 1.0))
    if dot > 0.9995:
        q = q0 + alpha * (q1 - q0)
        q /= np.linalg.norm(q)
        return quat_wxyz_to_R(q)
    theta0 = math.acos(dot)
    sin_theta0 = math.sin(theta0)
    theta = theta0 * alpha
    s0 = math.sin(theta0 - theta) / sin_theta0
    s1 = math.sin(theta) / sin_theta0
    q = s0 * q0 + s1 * q1
    q /= np.linalg.norm(q)
    return quat_wxyz_to_R(q)

def tilt_from_vertical_down_deg(R_cam_grasp):
    approach_cam = R_cam_grasp[:, 0]
    approach_torso = R_TORSO_CAMERA @ approach_cam
    approach_torso /= np.linalg.norm(approach_torso)
    down = np.array([0.0, 0.0, -1.0], dtype=np.float64)
    c = float(np.clip(np.dot(approach_torso, down), -1.0, 1.0))
    return math.degrees(math.acos(c))

prev_t = None
prev_R = None
miss_count = 0

def choose_candidate(gg_sorted):
    global prev_t, prev_R, miss_count
    n = min(TOP_N, len(gg_sorted))
    candidates = []

    for i in range(n):
        g = gg_sorted[i]
        R = np.asarray(g.rotation_matrix, dtype=np.float64)
        t = np.asarray(g.translation, dtype=np.float64).reshape(3)
        score = float(g.score)
        tilt = tilt_from_vertical_down_deg(R)

        if tilt < MIN_TILT_DEG or tilt > MAX_TILT_DEG:
            continue

        angle_quality = math.exp(-0.5 * ((tilt - DESIRED_TILT_DEG) / 10.0) ** 2)
        pos_jump = 0.0
        rot_jump = 0.0
        continuity = 0.0

        if prev_t is not None:
            pos_jump = float(np.linalg.norm(t - prev_t))
            rot_jump = rotation_angle_deg(prev_R, R)
            pos_quality = math.exp(-pos_jump / 0.030)
            rot_quality = math.exp(-rot_jump / 20.0)
            continuity = 0.8 * pos_quality + 1.0 * rot_quality

        total = 2.0 * score + 1.15 * angle_quality + continuity
        candidates.append((total, g, R, t, score, tilt, pos_jump, rot_jump))

    if not candidates:
        miss_count += 1
        return None

    if prev_t is not None and miss_count < RESEED_AFTER_MISSES:
        near = [
            c for c in candidates
            if c[6] <= MAX_TRACK_POS_JUMP_M and c[7] <= MAX_TRACK_ROT_JUMP_DEG
        ]
        if near:
            candidates = near
        else:
            miss_count += 1
            return None

    best = max(candidates, key=lambda c: c[0])
    _, g, R, t, score, tilt, pos_jump, rot_jump = best
    miss_count = 0

    if prev_t is None:
        filt_t = t
        filt_R = R
    else:
        filt_t = (1.0 - POS_ALPHA) * prev_t + POS_ALPHA * t
        filt_R = slerp_R(prev_R, R, ROT_ALPHA)

    prev_t = filt_t
    prev_R = filt_R
    return g, filt_R, filt_t, score, tilt, pos_jump, rot_jump

while True:
    try:
        data = socket.recv_pyobj()
        points = data.get('points', None)
        colors = data.get('colors', None)

        if points is None or len(points) == 0:
            socket.send_pyobj({'success': False, 'msg': 'Empty points'})
            continue

        points = np.ascontiguousarray(points, dtype=np.float32)
        if colors is not None and len(colors) == len(points):
            colors = np.ascontiguousarray(colors, dtype=np.float32)
        else:
            colors = np.ones((len(points), 3), dtype=np.float32)

        if SHOW_VISUALIZATION and vis is not None:
            pcd.points = o3d.utility.Vector3dVector(points)
            pcd.colors = o3d.utility.Vector3dVector(colors)
            vis.update_geometry(pcd)
            if is_first_frame:
                vis.reset_view_point(True)
                is_first_frame = False

        gg, cloud = anygrasp.get_grasp(
            points,
            colors,
            lims=lims,
            voxel_size=0.004,
            apply_object_mask=False,
            dense_grasp=False,
            collision_detection=True,
        )

        result = {'success': False, 'msg': 'No suitable slanted grasp'}

        if gg is not None and len(gg) > 0:
            gg_nms = gg.nms().sort_by_score()
            selected = choose_candidate(gg_nms)

            if selected is not None:
                g, filt_R, filt_t, score, tilt, pos_jump, rot_jump = selected
                pose_matrix = np.eye(4, dtype=np.float32)
                pose_matrix[:3, :3] = filt_R.astype(np.float32)
                pose_matrix[:3, 3] = filt_t.astype(np.float32)

                result = {
                    'success': True,
                    'score': score,
                    'pose': pose_matrix.tolist(),
                    'tilt_deg': float(tilt),
                    'tracked': True,
                }

                print(
                    f"slanted grasp | score={score:.3f} "
                    f"tilt={tilt:.1f}deg "
                    f"pos_jump={pos_jump*1000:.1f}mm "
                    f"rot_jump={rot_jump:.1f}deg"
                )

        if SHOW_VISUALIZATION and vis is not None:
            vis.poll_events()
            vis.update_renderer()

        socket.send_pyobj(result)

    except Exception as e:
        print(f"AnyGrasp inference error: {e}")
        if SHOW_VISUALIZATION and vis is not None:
            vis.poll_events()
            vis.update_renderer()
        socket.send_pyobj({'success': False, 'error': str(e)})
