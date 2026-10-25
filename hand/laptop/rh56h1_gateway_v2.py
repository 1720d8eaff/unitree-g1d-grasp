#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socket
import serial
import time

SERIAL_PORT = "/dev/ttyUSB0"
BAUDRATE = 115200
UDP_BIND_IP = "0.0.0.0"
UDP_PORT = 5005
SERIAL_TIMEOUT = 0.20
MAX_UDP_SIZE = 4096


def hexstr(data):
    return " ".join(f"{b:02X}" for b in data)


def checksum_ok(frame):
    if len(frame) < 9:
        return False
    return (sum(frame[2:-1]) & 0xFF) == frame[-1]


def find_complete_frame(buf):
    # 请求帧头通常 EB 90，回复帧头按手册为 90 EB。
    # 为兼容现有 SDK，两种都识别。
    for i in range(max(0, len(buf) - 2048), len(buf) - 1):
        if not (
            (buf[i] == 0x90 and buf[i + 1] == 0xEB)
            or
            (buf[i] == 0xEB and buf[i + 1] == 0x90)
        ):
            continue

        if len(buf) < i + 4:
            return None

        length = buf[i + 3]

        # 手册帧结构：
        # 2字节帧头 + ID + LEN + LEN字节数据部分 + CHECKSUM
        # 所以总长 = LEN + 5
        total_len = length + 5

        if total_len < 9 or total_len > 2048:
            continue

        if len(buf) < i + total_len:
            return None

        frame = bytes(buf[i:i + total_len])

        if checksum_ok(frame):
            return frame

        # 已收满但校验失败，不把坏帧当成有效响应
        return b""

    return None


def read_hand_response(ser, timeout=SERIAL_TIMEOUT):
    deadline = time.monotonic() + timeout
    buf = bytearray()

    while time.monotonic() < deadline:
        waiting = ser.in_waiting
        if waiting:
            buf.extend(ser.read(waiting))
            frame = find_complete_frame(buf)

            if frame is None:
                pass
            elif frame == b"":
                return b"ERR:BAD_CHECKSUM"
            else:
                return frame

        time.sleep(0.001)

    if buf:
        return b"ERR:INCOMPLETE_FRAME:" + bytes(buf).hex().upper().encode()

    return b"ERR:NO_RESPONSE"


def main():
    print("===== RH56H1 双向 UDP ↔ RS485 网关 v2 =====")
    print(f"串口: {SERIAL_PORT}")
    print(f"波特率: {BAUDRATE}")
    print(f"UDP监听: {UDP_BIND_IP}:{UDP_PORT}")

    ser = serial.Serial(
        SERIAL_PORT,
        BAUDRATE,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.01,
    )

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_BIND_IP, UDP_PORT))

    print("✅ 串口已打开")
    print("✅ UDP 网关已启动")
    print("✅ 按手册读取完整帧并校验 checksum")
    print("等待板载机请求...")

    try:
        while True:
            request, client_addr = sock.recvfrom(MAX_UDP_SIZE)
            if not request:
                continue

            print()
            print(f"📥 UDP {client_addr[0]}:{client_addr[1]}")
            print(f"→ RH56H1: {hexstr(request)}")

            ser.reset_input_buffer()
            ser.write(request)
            ser.flush()

            response = read_hand_response(ser)

            if response.startswith(b"ERR:"):
                print("⚠️", response.decode(errors="replace"))
            else:
                print(f"← RH56H1: {hexstr(response)}")

            sock.sendto(response, client_addr)
            print("📤 已返回板载机")

    except KeyboardInterrupt:
        print("\n网关退出")
    finally:
        ser.close()
        sock.close()


if __name__ == "__main__":
    main()
