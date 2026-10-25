import json
import os
import threading
import time

from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelSubscriber,
)
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_


# G1-D waist: yaw=12, pitch=14. Motor 13 is invalid on G1-D.
WAIST_YAW_INDEX = 12
WAIST_PITCH_INDEX = 14

# G1 双臂对应的电机编号：15～28，共 14 个关节
MOTOR_INDEXES = list(range(15, 29))

STATE_FILE = "/tmp/g1_lowstate.json"
TEMP_FILE = "/tmp/g1_lowstate.json.tmp"

# JSON 文件更新频率：100 Hz
WRITE_PERIOD = 0.01

latest_state = None
state_lock = threading.Lock()


def lowstate_callback(msg):
    """保存最新状态，不在 DDS 回调中进行磁盘写入。"""
    global latest_state

    data = {
        "waist_yaw_position": float(
            msg.motor_state[WAIST_YAW_INDEX].q
        ),
        "waist_pitch_position": float(
            msg.motor_state[WAIST_PITCH_INDEX].q
        ),
        "waist_yaw_velocity": float(
            msg.motor_state[WAIST_YAW_INDEX].dq
        ),
        "waist_pitch_velocity": float(
            msg.motor_state[WAIST_PITCH_INDEX].dq
        ),
        "position": [
            float(msg.motor_state[index].q)
            for index in MOTOR_INDEXES
        ],
        "velocity": [
            float(msg.motor_state[index].dq)
            for index in MOTOR_INDEXES
        ],
        "effort": [
            float(msg.motor_state[index].tau_est)
            for index in MOTOR_INDEXES
        ],
    }

    with state_lock:
        latest_state = data


def write_latest_state():
    """以固定频率把最新状态原子写入 JSON 文件。"""
    global latest_state

    write_count = 0
    report_start = time.monotonic()
    next_write_time = time.monotonic()

    while True:
        now = time.monotonic()

        if now < next_write_time:
            time.sleep(next_write_time - now)

        next_write_time += WRITE_PERIOD

        # 如果程序一度落后，不补写旧帧，直接追上当前时间
        current_time = time.monotonic()
        if current_time - next_write_time > WRITE_PERIOD:
            next_write_time = current_time + WRITE_PERIOD

        with state_lock:
            data = latest_state

        if data is None:
            continue

        try:
            with open(TEMP_FILE, "w") as file:
                json.dump(
                    data,
                    file,
                    separators=(",", ":"),
                )

            # 原子替换，state_bridge 不会读到写了一半的文件
            os.replace(TEMP_FILE, STATE_FILE)

        except OSError as error:
            print(f"Failed to write state file: {error}")
            time.sleep(0.1)
            continue

        write_count += 1

        # 每 5 秒打印一次状态，避免逐帧输出拖慢程序
        now = time.monotonic()
        elapsed = now - report_start

        if False and elapsed >= 5.0:
            rate = write_count / elapsed
            print(f"State file update rate: {rate:.1f} Hz")

            write_count = 0
            report_start = now


def main():
    ChannelFactoryInitialize(0)

    subscriber = ChannelSubscriber(
        "rt/lowstate",
        LowState_,
    )

    # 队列深度设为 1，优先使用最新状态，避免旧消息堆积
    subscriber.Init(
        lowstate_callback,
        1,
    )

    print("Optimized Unitree lowstate reader started.")
    print(f"Writing {STATE_FILE} at up to 100 Hz.")

    write_latest_state()


if __name__ == "__main__":
    main()
