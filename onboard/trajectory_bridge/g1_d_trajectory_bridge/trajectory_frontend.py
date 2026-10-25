#!/usr/bin/env python3

"""ROS 2 Foxy front end for the split G1-D trajectory bridge.

This process deliberately does not import unitree_sdk2py. It only owns ROS 2
subscriptions/services and forwards commands to the pure Unitree DDS backend
through a local Unix domain socket.
"""

import json
import socket
import threading
import time
import uuid
from typing import Any, Dict, Optional, Tuple

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory

from .ipc_protocol import (
    DEFAULT_SOCKET_PATH,
    PROTOCOL_VERSION,
    FramedSocket,
)


CONTROL_REQUEST_TIMEOUT = 100.0
COMMAND_ACK_TIMEOUT = 5.0
RECONNECT_DELAY = 1.0


class LocalBackendClient:
    """Persistent reconnecting client for the local Unitree backend."""

    def __init__(self, node: Node, socket_path: str) -> None:
        self._node = node
        self._socket_path = socket_path
        self._shutdown = threading.Event()
        self._connected = threading.Event()
        self._connection_lock = threading.Lock()
        self._connection: Optional[FramedSocket] = None

        self._pending_lock = threading.Lock()
        self._pending: Dict[str, Tuple[threading.Event, Dict[str, Any]]] = {}

        self._thread = threading.Thread(
            target=self._connection_loop,
            name="g1_backend_socket_client",
            daemon=True,
        )
        self._thread.start()

    @property
    def connected(self) -> bool:
        return self._connected.is_set()

    def request(
        self,
        operation: str,
        payload: Optional[Dict[str, Any]] = None,
        timeout: float = COMMAND_ACK_TIMEOUT,
    ) -> Tuple[bool, str, Dict[str, Any]]:
        request_id = uuid.uuid4().hex
        event = threading.Event()
        holder: Dict[str, Any] = {}
        with self._pending_lock:
            self._pending[request_id] = (event, holder)

        message: Dict[str, Any] = {
            "version": PROTOCOL_VERSION,
            "kind": "request",
            "request_id": request_id,
            "operation": operation,
        }
        if payload:
            message["payload"] = payload

        try:
            self._send(message)
        except Exception as exc:
            with self._pending_lock:
                self._pending.pop(request_id, None)
            return False, f"无法发送到 Unitree 后端：{exc}", {}

        if not event.wait(timeout):
            with self._pending_lock:
                self._pending.pop(request_id, None)
            return False, f"等待 Unitree 后端响应超时（{timeout:.1f}s）", {}

        response = holder.get("response")
        if not isinstance(response, dict):
            return False, "Unitree 后端响应格式错误", {}
        return (
            bool(response.get("success", False)),
            str(response.get("message", "")),
            response,
        )

    def close(self) -> None:
        self._shutdown.set()
        self._drop_connection("ROS 前端正在关闭")
        self._thread.join(timeout=3.0)
        self._fail_all_pending("ROS 前端已关闭")

    def _send(self, message: Dict[str, Any]) -> None:
        with self._connection_lock:
            connection = self._connection
        if connection is None or not self._connected.is_set():
            raise ConnectionError("Unitree 后端尚未连接")
        try:
            connection.send(message)
        except Exception:
            self._drop_connection("发送失败")
            raise

    def _connection_loop(self) -> None:
        while not self._shutdown.is_set():
            if not self._connect_once():
                self._shutdown.wait(RECONNECT_DELAY)
                continue

            with self._connection_lock:
                connection = self._connection
            if connection is None:
                continue

            try:
                hello_id = uuid.uuid4().hex
                connection.send(
                    {
                        "version": PROTOCOL_VERSION,
                        "kind": "request",
                        "request_id": hello_id,
                        "operation": "hello",
                        "payload": {"client": "g1_d_trajectory_frontend"},
                    }
                )
                while not self._shutdown.is_set():
                    message = connection.recv()
                    self._handle_message(message)
            except EOFError:
                self._node.get_logger().warning("Unitree 后端连接已断开")
            except Exception as exc:
                self._node.get_logger().error(f"Unitree 后端 IPC 异常：{exc}")
            finally:
                self._drop_connection("Unitree 后端连接中断")

    def _connect_once(self) -> bool:
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        sock.settimeout(2.0)
        try:
            sock.connect(self._socket_path)
            sock.settimeout(None)
        except OSError:
            sock.close()
            return False

        connection = FramedSocket(sock)
        with self._connection_lock:
            old = self._connection
            self._connection = connection
            self._connected.set()
        if old is not None:
            old.close()
        self._node.get_logger().info(
            f"已连接本机 Unitree DDS 后端：{self._socket_path}"
        )
        return True

    def _handle_message(self, message: Dict[str, Any]) -> None:
        if int(message.get("version", -1)) != PROTOCOL_VERSION:
            self._node.get_logger().error("忽略协议版本不匹配的 IPC 消息")
            return

        kind = str(message.get("kind", ""))
        if kind == "response":
            request_id = str(message.get("request_id", ""))
            with self._pending_lock:
                pending = self._pending.pop(request_id, None)
            if pending is not None:
                event, holder = pending
                holder["response"] = message
                event.set()
            return

        if kind == "trajectory_status":
            self._node.publish_backend_trajectory_status(message)
            return

        if kind == "backend_state":
            self._node.publish_backend_state(message)
            return

        if kind == "log":
            level = str(message.get("level", "info"))
            text = str(message.get("message", ""))
            logger = self._node.get_logger()
            if level == "error":
                logger.error(f"[Unitree 后端] {text}")
            elif level == "warning":
                logger.warning(f"[Unitree 后端] {text}")
            else:
                logger.info(f"[Unitree 后端] {text}")
            return

        self._node.get_logger().warning(f"忽略未知 IPC 消息：{kind}")

    def _drop_connection(self, reason: str) -> None:
        with self._connection_lock:
            connection = self._connection
            self._connection = None
            was_connected = self._connected.is_set()
            self._connected.clear()
        if connection is not None:
            connection.close()
        if was_connected:
            self._node.publish_backend_state(
                {
                    "version": PROTOCOL_VERSION,
                    "kind": "backend_state",
                    "state": "DISCONNECTED",
                    "message": reason,
                    "stamp_monotonic": time.monotonic(),
                }
            )
        self._fail_all_pending(reason)

    def _fail_all_pending(self, reason: str) -> None:
        with self._pending_lock:
            pending_items = list(self._pending.values())
            self._pending.clear()
        for event, holder in pending_items:
            holder["response"] = {
                "version": PROTOCOL_VERSION,
                "kind": "response",
                "success": False,
                "message": reason,
            }
            event.set()


class TrajectoryFrontend(Node):
    """ROS-only front end; all Unitree work is delegated over local IPC."""

    def __init__(self, socket_path: str = DEFAULT_SOCKET_PATH) -> None:
        super().__init__("g1_d_trajectory_executor")
        self._callback_group = ReentrantCallbackGroup()

        self._status_publisher = self.create_publisher(
            String, "/g1_d/trajectory_status", 20
        )
        self._backend_state_publisher = self.create_publisher(
            String, "/g1_d/trajectory_backend_state", 10
        )
        self._trajectory_subscription = self.create_subscription(
            JointTrajectory,
            "/g1_d/trajectory_command",
            self._trajectory_callback,
            10,
            callback_group=self._callback_group,
        )
        self._cancel_subscription = self.create_subscription(
            String,
            "/g1_d/trajectory_cancel",
            self._cancel_callback,
            10,
            callback_group=self._callback_group,
        )
        self._control_service = self.create_service(
            SetBool,
            "/g1_d/set_trajectory_control",
            self._set_control_callback,
            callback_group=self._callback_group,
        )

        self._client = LocalBackendClient(self, socket_path)
        self.get_logger().info("板载 ROS 轨迹前端已启动")
        self.get_logger().warning(
            "该进程不导入 Unitree SDK；默认 DISARMED，需后端连接并显式启用"
        )

    def _set_control_callback(self, request, response):
        enabled = bool(request.data)
        payload: Dict[str, Any] = {"enabled": enabled}

        if enabled:
            state_ok, state_message, state_response = self._client.request(
                "get_state", timeout=COMMAND_ACK_TIMEOUT
            )
            if not state_ok:
                response.success = False
                response.message = state_message
                return response

            if str(state_response.get("state", "")) == "ARMED":
                response.success = True
                response.message = "轨迹控制已经启用"
                return response

            publishers = self.count_publishers("/lowcmd")
            subscriptions = self.count_subscribers("/lowcmd")
            if publishers != 0 or subscriptions != 1:
                response.success = False
                response.message = (
                    "/lowcmd 安全基线不满足："
                    f"publishers={publishers}, subscriptions={subscriptions}；"
                    "要求 0/1"
                )
                return response

            payload.update(
                {
                    "lowcmd_publishers": int(publishers),
                    "lowcmd_subscriptions": int(subscriptions),
                    "preflight_monotonic": time.monotonic(),
                }
            )

        success, message, _ = self._client.request(
            "set_control", payload, timeout=CONTROL_REQUEST_TIMEOUT
        )
        response.success = success
        response.message = message
        return response

    def _trajectory_callback(self, message: JointTrajectory) -> None:
        goal_id = self._extract_goal_id(message.header.frame_id)
        if goal_id is None:
            self._publish_local_status(
                "", "rejected", "header.frame_id 缺少 g1d:<goal_id>"
            )
            return

        payload = {
            "goal_id": goal_id,
            "joint_names": list(message.joint_names),
            "points": [self._serialize_point(point) for point in message.points],
        }
        success, text, _ = self._client.request(
            "submit_trajectory", payload, timeout=COMMAND_ACK_TIMEOUT
        )
        if not success:
            self._publish_local_status(goal_id, "rejected", text)

    def _cancel_callback(self, message: String) -> None:
        goal_id = message.data.strip()
        if not goal_id:
            return
        success, text, _ = self._client.request(
            "cancel_trajectory",
            {"goal_id": goal_id},
            timeout=COMMAND_ACK_TIMEOUT,
        )
        if not success:
            self.get_logger().warning(f"取消请求未被后端接受：{text}")

    def publish_backend_trajectory_status(self, message: Dict[str, Any]) -> None:
        payload = {
            "goal_id": str(message.get("goal_id", "")),
            "state": str(message.get("state", "")),
            "message": str(message.get("message", "")),
            "stamp_monotonic": float(message.get("stamp_monotonic", 0.0)),
        }
        ros_message = String()
        ros_message.data = json.dumps(
            payload, ensure_ascii=False, separators=(",", ":"), allow_nan=False
        )
        self._status_publisher.publish(ros_message)

    def publish_backend_state(self, message: Dict[str, Any]) -> None:
        ros_message = String()
        ros_message.data = json.dumps(
            message, ensure_ascii=False, separators=(",", ":"), allow_nan=False
        )
        self._backend_state_publisher.publish(ros_message)

    def _publish_local_status(self, goal_id: str, state: str, message: str) -> None:
        self.publish_backend_trajectory_status(
            {
                "goal_id": goal_id,
                "state": state,
                "message": message,
                "stamp_monotonic": time.monotonic(),
            }
        )

    @staticmethod
    def _serialize_point(point) -> Dict[str, Any]:
        duration_ns = int(point.time_from_start.sec) * 1_000_000_000 + int(
            point.time_from_start.nanosec
        )
        return {
            "positions": [float(value) for value in point.positions],
            "velocities": [float(value) for value in point.velocities],
            "accelerations": [float(value) for value in point.accelerations],
            "time_from_start_ns": duration_ns,
        }

    @staticmethod
    def _extract_goal_id(frame_id: str) -> Optional[str]:
        prefix = "g1d:"
        if not frame_id.startswith(prefix):
            return None
        value = frame_id[len(prefix):].strip()
        return value or None

    def close(self) -> None:
        self._client.close()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrajectoryFrontend()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
