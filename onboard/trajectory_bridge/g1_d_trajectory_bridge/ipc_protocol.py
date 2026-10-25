#!/usr/bin/env python3

"""Length-prefixed JSON protocol for the local trajectory bridge socket."""

import json
import socket
import struct
import threading
from typing import Any, Dict


PROTOCOL_VERSION = 1
DEFAULT_SOCKET_PATH = "/tmp/g1_d_trajectory_backend.sock"
MAX_MESSAGE_BYTES = 16 * 1024 * 1024
_HEADER = struct.Struct("!I")


class ProtocolError(RuntimeError):
    """Raised when a peer sends an invalid framed message."""


class FramedSocket:
    """Thread-safe sender and single-reader wrapper around a stream socket."""

    def __init__(self, sock: socket.socket) -> None:
        self._sock = sock
        self._send_lock = threading.Lock()
        self._close_lock = threading.Lock()
        self._closed = False

    def send(self, message: Dict[str, Any]) -> None:
        if not isinstance(message, dict):
            raise TypeError("IPC message must be a dict")
        payload = json.dumps(
            message,
            ensure_ascii=False,
            separators=(",", ":"),
            allow_nan=False,
        ).encode("utf-8")
        if not payload or len(payload) > MAX_MESSAGE_BYTES:
            raise ProtocolError(
                f"IPC payload size {len(payload)} is outside 1..{MAX_MESSAGE_BYTES}"
            )
        frame = _HEADER.pack(len(payload)) + payload
        with self._send_lock:
            if self._closed:
                raise EOFError("IPC socket is closed")
            self._sock.sendall(frame)

    def recv(self) -> Dict[str, Any]:
        header = self._recv_exact(_HEADER.size)
        (size,) = _HEADER.unpack(header)
        if size <= 0 or size > MAX_MESSAGE_BYTES:
            raise ProtocolError(f"invalid IPC payload size: {size}")
        payload = self._recv_exact(size)
        try:
            message = json.loads(payload.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise ProtocolError(f"invalid IPC JSON: {exc}") from exc
        if not isinstance(message, dict):
            raise ProtocolError("IPC JSON root must be an object")
        return message

    def _recv_exact(self, size: int) -> bytes:
        chunks = bytearray()
        while len(chunks) < size:
            chunk = self._sock.recv(size - len(chunks))
            if not chunk:
                raise EOFError("IPC peer closed the connection")
            chunks.extend(chunk)
        return bytes(chunks)

    def close(self) -> None:
        with self._close_lock:
            if self._closed:
                return
            self._closed = True
            try:
                self._sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                self._sock.close()
            except OSError:
                pass
