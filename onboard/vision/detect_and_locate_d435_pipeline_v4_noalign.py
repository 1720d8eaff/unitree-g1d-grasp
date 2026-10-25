#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
G1-D D435I + YOLO(CUDA) + AnyGrasp low-latency pipeline V4 (NO rs.align)

Key change:
  - RealSense capture no longer calls rs.align().
  - Color frames go directly to YOLO.
  - Raw depth is filtered in a separate worker.
  - Depth points are transformed/projected to the COLOR optical frame
    with the D435I factory depth->color extrinsics.
  - Only newest data are kept; stale frames are dropped.

This preserves the calibrated color-optical coordinate chain while avoiding
the ~95 ms capture+align bottleneck measured on the Jetson Orin.

This script DOES NOT control the robot.
"""

import json
import math
import queue
import socket as pysocket
import threading
import time

import cv2
import numpy as np
import pyrealsense2 as rs
import torch
import zmq
from ultralytics import YOLO


# ============================================================
# Configuration
# ============================================================
LAPTOP_IP = "192.168.123.222"
ANYGRASP_PORT = 5555
POSE_UDP_PORT = 5011

CAM_W = 640
CAM_H = 480
CAM_FPS = 30

YOLO_MODEL = "/home/unitree/best.pt"
YOLO_CONF = 0.50
YOLO_IMGSZ = 640
YOLO_DEVICE = 0

GRASP_REQUEST_HZ = 15.0
GRASP_PERIOD = 1.0 / GRASP_REQUEST_HZ

# Use every second depth pixel before projection: 640x480 -> 320x240.
# Plenty for the cube while keeping CPU projection light.
DEPTH_STRIDE = 2
MAX_POINTS_TO_SEND = 3000

# PARK hand-eye calibration: ^torso T_camera_color
T_TORSO_CAMERA = np.array([
    [-0.018012825, -0.734779856, +0.678066443, +0.084031145],
    [-0.999829739, +0.010521744, -0.015158680, +0.028936626],
    [+0.004003851, -0.678224045, -0.734844278, +0.353826536],
    [+0.000000000, +0.000000000, +0.000000000, +1.000000000],
], dtype=np.float64)


# ============================================================
# Shared state
# ============================================================
class Shared:
    def __init__(self):
        self.stop = threading.Event()

        self.color_lock = threading.Lock()
        self.color_id = -1
        self.color_time = 0.0
        self.color = None
        self.camera_hz = 0.0

        self.depth_lock = threading.Lock()
        self.depth_time = 0.0
        self.depth_mm = None
        self.depth_hz = 0.0

        self.depth_queue = queue.Queue(maxsize=1)

        # Filled once camera pipeline starts.
        self.calib_ready = threading.Event()
        self.fx_d = self.fy_d = self.cx_d = self.cy_d = None
        self.fx_c = self.fy_c = self.cx_c = self.cy_c = None
        self.R_dc = None
        self.t_dc = None
        self.ud_grid = None
        self.vd_grid = None


S = Shared()


# ============================================================
# RealSense capture: NO rs.align here
# ============================================================
def camera_worker():
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, CAM_W, CAM_H, rs.format.z16, CAM_FPS)
    config.enable_stream(rs.stream.color, CAM_W, CAM_H, rs.format.bgr8, CAM_FPS)

    profile = pipeline.start(config)

    depth_profile = profile.get_stream(
        rs.stream.depth
    ).as_video_stream_profile()
    color_profile = profile.get_stream(
        rs.stream.color
    ).as_video_stream_profile()

    di = depth_profile.get_intrinsics()
    ci = color_profile.get_intrinsics()
    ex = depth_profile.get_extrinsics_to(color_profile)

    S.fx_d, S.fy_d = float(di.fx), float(di.fy)
    S.cx_d, S.cy_d = float(di.ppx), float(di.ppy)
    S.fx_c, S.fy_c = float(ci.fx), float(ci.fy)
    S.cx_c, S.cy_c = float(ci.ppx), float(ci.ppy)

    S.R_dc = np.asarray(
        ex.rotation, dtype=np.float64
    ).reshape(3, 3)
    S.t_dc = np.asarray(
        ex.translation, dtype=np.float64
    ).reshape(3)

    # Precompute sampled native-depth pixel coordinates.
    vv, uu = np.mgrid[
        0:CAM_H:DEPTH_STRIDE,
        0:CAM_W:DEPTH_STRIDE
    ]
    S.ud_grid = uu.astype(np.float32)
    S.vd_grid = vv.astype(np.float32)

    print("D435I calibration loaded:")
    print(
        f"  depth intr: fx={S.fx_d:.3f} fy={S.fy_d:.3f} "
        f"cx={S.cx_d:.3f} cy={S.cy_d:.3f}"
    )
    print(
        f"  color intr: fx={S.fx_c:.3f} fy={S.fy_c:.3f} "
        f"cx={S.cx_c:.3f} cy={S.cy_c:.3f}"
    )
    print("  depth->color t =", S.t_dc)

    S.calib_ready.set()

    frame_id = 0
    times = []

    try:
        while not S.stop.is_set():
            frames = pipeline.wait_for_frames()

            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()

            if not color_frame or not depth_frame:
                continue

            now = time.monotonic()
            color = np.asanyarray(
                color_frame.get_data()
            ).copy()

            frame_id += 1
            times.append(now)
            times = [t for t in times if now - t <= 2.0]

            camera_hz = 0.0
            if len(times) >= 2:
                span = times[-1] - times[0]
                if span > 1e-6:
                    camera_hz = (len(times) - 1) / span

            with S.color_lock:
                S.color_id = frame_id
                S.color_time = now
                S.color = color
                S.camera_hz = camera_hz

            # Keep only newest frameset for depth processing.
            item = (frames, now)
            try:
                while True:
                    S.depth_queue.get_nowait()
            except queue.Empty:
                pass

            try:
                S.depth_queue.put_nowait(item)
            except queue.Full:
                pass

    finally:
        pipeline.stop()


# ============================================================
# Depth filtering in a separate worker, still NO rs.align
# ============================================================
def depth_worker():
    spatial = rs.spatial_filter()
    temporal = rs.temporal_filter()
    hole = rs.hole_filling_filter()
    threshold = rs.threshold_filter(0.2, 1.2)

    times = []

    while not S.stop.is_set():
        try:
            frames, source_time = S.depth_queue.get(timeout=0.05)
        except queue.Empty:
            continue

        depth = frames.get_depth_frame()
        if not depth:
            continue

        depth = spatial.process(depth)
        depth = temporal.process(depth)
        depth = hole.process(depth)
        depth = threshold.process(depth)

        depth_mm = np.asanyarray(
            depth.get_data()
        ).copy()

        now = time.monotonic()
        times.append(now)
        times = [t for t in times if now - t <= 2.0]

        hz = 0.0
        if len(times) >= 2:
            span = times[-1] - times[0]
            if span > 1e-6:
                hz = (len(times) - 1) / span

        with S.depth_lock:
            S.depth_time = source_time
            S.depth_mm = depth_mm
            S.depth_hz = hz


# ============================================================
# Vectorized native-depth -> color-optical point cloud
# ============================================================
def build_color_frame_cloud(depth_mm, color_bgr, box):
    """
    Return points/colors in the D435I COLOR OPTICAL frame.

    We deproject native depth pixels in the depth frame, transform them with
    factory depth->color extrinsics, project into color pixels, then keep only
    points whose projected color pixels fall inside the YOLO box.
    """
    t0 = time.perf_counter()

    x1, y1, x2, y2 = box
    bw = x2 - x1
    bh = y2 - y1
    if bw <= 4 or bh <= 4:
        return None, None, 0.0

    # Same 10% border trim as prior clients.
    rx1 = max(0.0, x1 + 0.10 * bw)
    rx2 = min(float(CAM_W - 1), x2 - 0.10 * bw)
    ry1 = max(0.0, y1 + 0.10 * bh)
    ry2 = min(float(CAM_H - 1), y2 - 0.10 * bh)

    z = depth_mm[
        0:CAM_H:DEPTH_STRIDE,
        0:CAM_W:DEPTH_STRIDE
    ].astype(np.float32) * 0.001

    valid = (z > 0.20) & (z < 1.20)
    if np.count_nonzero(valid) < 50:
        return None, None, (time.perf_counter() - t0) * 1000.0

    ud = S.ud_grid[valid]
    vd = S.vd_grid[valid]
    zd = z[valid].astype(np.float64)

    xd = (
        (ud.astype(np.float64) - S.cx_d)
        * zd / S.fx_d
    )
    yd = (
        (vd.astype(np.float64) - S.cy_d)
        * zd / S.fy_d
    )

    pd = np.column_stack((xd, yd, zd))

    # depth optical -> color optical
    pc = pd @ S.R_dc.T + S.t_dc
    zc = pc[:, 2]

    valid_z = zc > 1e-4
    pc = pc[valid_z]
    zc = zc[valid_z]

    if len(pc) < 50:
        return None, None, (time.perf_counter() - t0) * 1000.0

    uc = S.fx_c * pc[:, 0] / zc + S.cx_c
    vc = S.fy_c * pc[:, 1] / zc + S.cy_c

    inside = (
        (uc >= rx1) & (uc <= rx2) &
        (vc >= ry1) & (vc <= ry2) &
        (uc >= 0.0) & (uc < CAM_W) &
        (vc >= 0.0) & (vc < CAM_H)
    )

    pc = pc[inside]
    uc = uc[inside]
    vc = vc[inside]

    if len(pc) < 50:
        return None, None, (time.perf_counter() - t0) * 1000.0

    # Keep near object surface/volume inside the YOLO ROI.
    z_front = float(np.percentile(pc[:, 2], 5))
    obj = pc[:, 2] <= (z_front + 0.12)

    pc = pc[obj]
    uc = uc[obj]
    vc = vc[obj]

    if len(pc) < 50:
        return None, None, (time.perf_counter() - t0) * 1000.0

    ui = np.clip(np.rint(uc).astype(np.int32), 0, CAM_W - 1)
    vi = np.clip(np.rint(vc).astype(np.int32), 0, CAM_H - 1)

    bgr = color_bgr[vi, ui]
    rgb = bgr[:, ::-1].astype(np.float32) / 255.0

    if len(pc) > MAX_POINTS_TO_SEND:
        step = int(
            math.ceil(len(pc) / float(MAX_POINTS_TO_SEND))
        )
        pc = pc[::step]
        rgb = rgb[::step]

    cloud_ms = (time.perf_counter() - t0) * 1000.0

    return (
        np.ascontiguousarray(pc, dtype=np.float32),
        np.ascontiguousarray(rgb, dtype=np.float32),
        cloud_ms,
    )


# ============================================================
# AnyGrasp background worker
# ============================================================
class AnyGraspWorker:
    def __init__(self):
        self.q = queue.Queue(maxsize=1)
        self.lock = threading.Lock()
        self.latest = None
        self.stop = threading.Event()
        self.thread = threading.Thread(
            target=self._run,
            name="anygrasp_zmq",
            daemon=True
        )
        self.thread.start()

    def _socket(self, ctx):
        s = ctx.socket(zmq.REQ)
        s.setsockopt(zmq.RCVTIMEO, 2000)
        s.setsockopt(zmq.SNDTIMEO, 1000)
        s.setsockopt(zmq.LINGER, 0)
        s.connect(
            f"tcp://{LAPTOP_IP}:{ANYGRASP_PORT}"
        )
        return s

    def submit_latest(
        self, points, colors, label,
        source_color_time, source_depth_time
    ):
        item = {
            "points": points,
            "colors": colors,
            "label": label,
            "source_color_time": source_color_time,
            "source_depth_time": source_depth_time,
        }

        try:
            while True:
                self.q.get_nowait()
        except queue.Empty:
            pass

        try:
            self.q.put_nowait(item)
        except queue.Full:
            pass

    def get_latest(self):
        with self.lock:
            if self.latest is None:
                return None
            out = dict(self.latest)
            if out.get("pose_torso") is not None:
                out["pose_torso"] = out[
                    "pose_torso"
                ].copy()
            return out

    def _run(self):
        ctx = zmq.Context()
        sock = self._socket(ctx)

        recent = []
        next_allowed = 0.0

        try:
            while not self.stop.is_set():
                try:
                    item = self.q.get(timeout=0.05)
                except queue.Empty:
                    continue

                now = time.monotonic()
                if now < next_allowed:
                    time.sleep(next_allowed - now)

                # If a newer cloud arrived while waiting, use it.
                try:
                    while True:
                        item = self.q.get_nowait()
                except queue.Empty:
                    pass

                next_allowed = (
                    time.monotonic() + GRASP_PERIOD
                )

                t0 = time.perf_counter()

                try:
                    sock.send_pyobj({
                        "points": item["points"],
                        "colors": item["colors"],
                    })
                    result = sock.recv_pyobj()
                except (zmq.Again, zmq.ZMQError) as exc:
                    with self.lock:
                        self.latest = {
                            "success": False,
                            "msg": f"ZMQ error: {exc}",
                            "receive_time": time.monotonic(),
                            "grasp_hz": 0.0,
                            "roundtrip_ms": 0.0,
                        }
                    try:
                        sock.close()
                    except Exception:
                        pass
                    sock = self._socket(ctx)
                    continue

                rt_ms = (
                    time.perf_counter() - t0
                ) * 1000.0

                done = time.monotonic()

                recent.append(done)
                recent = [
                    t for t in recent
                    if done - t <= 2.0
                ]

                hz = 0.0
                if len(recent) >= 2:
                    span = recent[-1] - recent[0]
                    if span > 1e-6:
                        hz = (
                            len(recent) - 1
                        ) / span

                if not result.get(
                    "success", False
                ):
                    with self.lock:
                        self.latest = {
                            "success": False,
                            "msg": result.get(
                                "msg", "no grasp"
                            ),
                            "receive_time": done,
                            "source_color_time":
                                item["source_color_time"],
                            "source_depth_time":
                                item["source_depth_time"],
                            "grasp_hz": hz,
                            "roundtrip_ms": rt_ms,
                        }
                    continue

                pose_camera = np.asarray(
                    result["pose"],
                    dtype=np.float64
                )

                if pose_camera.shape != (4, 4):
                    continue

                pose_torso = (
                    T_TORSO_CAMERA @ pose_camera
                )

                with self.lock:
                    self.latest = {
                        "success": True,
                        "pose_torso": pose_torso,
                        "score": float(
                            result.get("score", 0.0)
                        ),
                        "tilt": float(
                            result.get(
                                "tilt_deg", -1.0
                            )
                        ),
                        "label": item["label"],
                        "receive_time": done,
                        "source_color_time":
                            item["source_color_time"],
                        "source_depth_time":
                            item["source_depth_time"],
                        "grasp_hz": hz,
                        "roundtrip_ms": rt_ms,
                        "msg": "ok",
                    }

        finally:
            try:
                sock.close()
            except Exception:
                pass
            ctx.term()

    def close(self):
        self.stop.set()
        self.thread.join(timeout=1.0)


# ============================================================
# Helpers
# ============================================================
def cuda_timed_yolo(model, image):
    torch.cuda.synchronize()
    t0 = time.perf_counter()

    result = model(
        image,
        conf=YOLO_CONF,
        imgsz=YOLO_IMGSZ,
        device=YOLO_DEVICE,
        verbose=False,
    )[0]

    torch.cuda.synchronize()

    ms = (
        time.perf_counter() - t0
    ) * 1000.0

    return result, ms


# ============================================================
# Main
# ============================================================
def main():
    print("=== ENV ===")
    print("torch:", torch.__version__)
    print("torch.version.cuda:", torch.version.cuda)
    print("CUDA available:", torch.cuda.is_available())

    if not torch.cuda.is_available():
        raise RuntimeError(
            "CUDA unavailable. Use: conda activate tv_jetson"
        )

    print("GPU:", torch.cuda.get_device_name(0))

    print("\n=== YOLO LOAD + WARM-UP ===")
    model = YOLO(YOLO_MODEL)

    dummy = np.zeros(
        (CAM_H, CAM_W, 3),
        dtype=np.uint8
    )

    # Three warm-ups: first is expensive on Jetson.
    for i in range(3):
        _, ms = cuda_timed_yolo(
            model, dummy
        )
        print(
            f"warm-up {i+1}: "
            f"{ms:.1f} ms "
            f"({1000.0/ms:.1f} FPS)"
        )

    print("YOLO READY.\n")

    cam_thread = threading.Thread(
        target=camera_worker,
        name="realsense_capture",
        daemon=True
    )
    dep_thread = threading.Thread(
        target=depth_worker,
        name="depth_filter",
        daemon=True
    )

    cam_thread.start()
    dep_thread.start()

    if not S.calib_ready.wait(timeout=5.0):
        raise RuntimeError(
            "D435I calibration/profile did not initialize."
        )

    anygrasp = AnyGraspWorker()

    pose_udp = pysocket.socket(
        pysocket.AF_INET,
        pysocket.SOCK_DGRAM
    )

    cv2.namedWindow(
        "RealSense D435 + Pipeline V4 NO-ALIGN",
        cv2.WINDOW_NORMAL
    )
    # Smaller remote window = less NoMachine bandwidth/load.
    cv2.resizeWindow(
        "RealSense D435 + Pipeline V4 NO-ALIGN",
        800, 600
    )

    last_color_id = -1

    yolo_times = []
    yolo_hz = 0.0
    yolo_ms = 0.0
    cloud_ms = 0.0

    locked = False
    locked_pose = None
    locked_meta = None

    ui_count = 0
    ui_t0 = time.monotonic()
    ui_hz = 0.0

    print("Pipeline V4 ready.")
    print("L=LOCK  U=UNLOCK  Q=QUIT")
    print("This script DOES NOT control the robot.")

    try:
        while True:
            with S.color_lock:
                cid = S.color_id
                if (
                    cid < 0
                    or cid == last_color_id
                    or S.color is None
                ):
                    color = None
                else:
                    color = S.color.copy()
                    color_time = S.color_time
                    camera_hz = S.camera_hz

            if color is None:
                time.sleep(0.001)
                continue

            last_color_id = cid

            # ----------------------------------------------------
            # YOLO on latest COLOR frame
            # ----------------------------------------------------
            result, yolo_ms = cuda_timed_yolo(
                model, color
            )

            now = time.monotonic()

            yolo_times.append(now)
            yolo_times = [
                t for t in yolo_times
                if now - t <= 2.0
            ]

            if len(yolo_times) >= 2:
                span = (
                    yolo_times[-1]
                    - yolo_times[0]
                )
                if span > 1e-6:
                    yolo_hz = (
                        len(yolo_times) - 1
                    ) / span

            box = None
            label = None
            det_conf = 0.0

            if (
                result.boxes is not None
                and len(result.boxes) > 0
            ):
                best = max(
                    result.boxes,
                    key=lambda b: float(
                        b.conf[0]
                    )
                )

                box = tuple(
                    map(
                        int,
                        best.xyxy[0].tolist()
                    )
                )
                cls_id = int(best.cls[0])
                label = str(
                    model.names[cls_id]
                )
                det_conf = float(
                    best.conf[0]
                )

            # ----------------------------------------------------
            # Build cloud from latest filtered NATIVE depth.
            # NO rs.align.
            # ----------------------------------------------------
            depth_hz = 0.0
            depth_age = 999.0

            if box is not None and not locked:
                with S.depth_lock:
                    depth = (
                        None
                        if S.depth_mm is None
                        else S.depth_mm.copy()
                    )
                    depth_time = S.depth_time
                    depth_hz = S.depth_hz

                if depth is not None:
                    depth_age = (
                        now - depth_time
                    )

                    pts, cols, cloud_ms = (
                        build_color_frame_cloud(
                            depth,
                            color,
                            box
                        )
                    )

                    if pts is not None:
                        anygrasp.submit_latest(
                            pts,
                            cols,
                            label,
                            color_time,
                            depth_time,
                        )

            else:
                with S.depth_lock:
                    depth_hz = S.depth_hz
                    if S.depth_time > 0:
                        depth_age = (
                            now - S.depth_time
                        )

            # ----------------------------------------------------
            # Latest AnyGrasp result
            # ----------------------------------------------------
            g = anygrasp.get_latest()

            active_pose = None
            active_score = 0.0
            active_tilt = -1.0
            active_label = (
                label or "object"
            )

            grasp_hz = 0.0
            rt_ms = 0.0
            grasp_age = 999.0
            e2e_ms = -1.0
            grasp_msg = "waiting"

            if g is not None:
                grasp_hz = float(
                    g.get("grasp_hz", 0.0)
                )
                rt_ms = float(
                    g.get(
                        "roundtrip_ms", 0.0
                    )
                )
                grasp_msg = str(
                    g.get("msg", "waiting")
                )

                if g.get(
                    "receive_time"
                ) is not None:
                    grasp_age = (
                        now
                        - float(
                            g["receive_time"]
                        )
                    )

                if g.get(
                    "source_color_time"
                ) is not None:
                    e2e_ms = (
                        now
                        - float(
                            g[
                                "source_color_time"
                            ]
                        )
                    ) * 1000.0

                if g.get(
                    "success", False
                ):
                    active_pose = g[
                        "pose_torso"
                    ]
                    active_score = float(
                        g["score"]
                    )
                    active_tilt = float(
                        g["tilt"]
                    )
                    active_label = str(
                        g["label"]
                    )

            if (
                locked
                and locked_pose is not None
            ):
                active_pose = locked_pose
                active_score = (
                    locked_meta["score"]
                )
                active_tilt = (
                    locked_meta["tilt"]
                )
                active_label = (
                    locked_meta["label"]
                )

            # ----------------------------------------------------
            # UDP target pose to laptop ROS bridge
            # ----------------------------------------------------
            if active_pose is not None:
                payload = {
                    "frame_id": "torso_link",
                    "child_frame_id":
                        "anygrasp_target_raw",
                    "score": active_score,
                    "label": active_label,
                    "pose_matrix":
                        active_pose.tolist(),
                    "stamp": time.time(),
                    "locked": bool(locked),
                }

                pose_udp.sendto(
                    json.dumps(
                        payload
                    ).encode("utf-8"),
                    (
                        LAPTOP_IP,
                        POSE_UDP_PORT
                    ),
                )

            # ----------------------------------------------------
            # Draw
            # ----------------------------------------------------
            if box is not None:
                x1, y1, x2, y2 = box

                draw_color = (
                    (0, 255, 255)
                    if locked
                    else (0, 255, 0)
                )

                cv2.rectangle(
                    color,
                    (x1, y1),
                    (x2, y2),
                    draw_color,
                    2
                )

                txt = (
                    f"{label} {det_conf:.2f} | "
                    f"{'LOCKED' if locked else 'TRACK'} "
                    f"S={active_score:.2f} "
                    f"Tilt={active_tilt:.0f}"
                )

                cv2.putText(
                    color,
                    txt,
                    (x1, max(20, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.50,
                    draw_color,
                    2
                )

            ui_count += 1
            if now - ui_t0 >= 1.0:
                ui_hz = (
                    ui_count
                    / (now - ui_t0)
                )
                ui_count = 0
                ui_t0 = now

            line1 = (
                f"Cam {camera_hz:.1f}Hz | "
                f"YOLO {yolo_hz:.1f}Hz/{yolo_ms:.0f}ms | "
                f"Depth {depth_hz:.1f}Hz"
            )

            line2 = (
                f"Cloud {cloud_ms:.1f}ms | "
                f"AnyGrasp {grasp_hz:.1f}Hz/{rt_ms:.0f}ms | "
                f"E2E {e2e_ms:.0f}ms"
            )

            line3 = (
                f"depth age {depth_age*1000:.0f}ms | "
                f"grasp age {grasp_age:.2f}s | "
                f"UI {ui_hz:.1f} | "
                f"{grasp_msg}"
            )

            line4 = (
                f"{'LOCKED' if locked else 'TRACKING'} | "
                "L=LOCK U=UNLOCK Q=QUIT"
            )

            cv2.putText(
                color, line1, (10, 24),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.46, (0, 255, 0), 2
            )
            cv2.putText(
                color, line2, (10, 47),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.46, (0, 255, 0), 2
            )
            cv2.putText(
                color, line3, (10, 70),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.43, (0, 255, 0), 2
            )
            cv2.putText(
                color, line4, (10, 93),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.46,
                (
                    (0, 255, 255)
                    if locked
                    else (255, 255, 255)
                ),
                2
            )

            cv2.imshow(
                "RealSense D435 + Pipeline V4 NO-ALIGN",
                color
            )

            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                break

            elif key == ord("l"):
                if (
                    active_pose is not None
                    and not locked
                ):
                    locked = True
                    locked_pose = (
                        active_pose.copy()
                    )
                    locked_meta = {
                        "score":
                            active_score,
                        "tilt":
                            active_tilt,
                        "label":
                            active_label,
                    }

                    p = locked_pose[:3, 3]
                    print(
                        "GRASP LOCKED | "
                        f"score={active_score:.3f} "
                        f"tilt={active_tilt:.1f} "
                        f"xyz=[{p[0]:+.4f},"
                        f"{p[1]:+.4f},"
                        f"{p[2]:+.4f}]"
                    )

            elif key == ord("u"):
                locked = False
                locked_pose = None
                locked_meta = None
                print("GRASP UNLOCKED.")

    finally:
        S.stop.set()
        anygrasp.close()
        time.sleep(0.2)
        cv2.destroyAllWindows()
        pose_udp.close()


if __name__ == "__main__":
    main()
