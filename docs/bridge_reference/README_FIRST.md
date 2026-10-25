# G1-D 真实轨迹桥 Phase 2：ROS / Unitree DDS 双进程隔离

## 为什么必须拆分

板载机实测确认，同一 Python 进程内混用 ROS 2 DDS 与 Unitree Python CycloneDDS 不稳定：

- `rmw_cyclonedds_cpp`：创建 `rt/lowstate` Topic 时出现 `DDS_RETCODE_BAD_PARAMETER`。
- `rmw_fastrtps_cpp`：即使禁用共享内存、只用 UDP，仍出现 `std::bad_alloc` 和进程被杀死。

因此本版本将板载侧拆分为：

1. `trajectory_frontend`：纯 ROS 2 Foxy，不导入 Unitree SDK。
2. `trajectory_backend`：纯 Unitree DDS，不导入 rclpy。
3. 两者只通过本机 Unix Domain Socket `/tmp/g1_d_trajectory_backend.sock` 通信。

500 Hz LowCmd、轨迹插值、MotionSwitcher、安全检查全部保留在后端本地。

## 安全状态

- 后端启动默认为 `DISARMED`。
- DISARMED 时不会创建 `rt/lowcmd`，不会调用 `ReleaseMode`。
- 启用前后端会额外调用独立 `ros2` 子进程检查 `/lowcmd` 必须是 Publisher=0、Subscription=1。
- ROS 前端断开时，后端不会停发 LowCmd；活动轨迹会取消并冻结在实测位置，等待前端重连后显式停用。
- 后端关闭时若 LowCmd 正在运行，会持续尝试恢复 `ai`，确认后才停止。

## 安装顺序

先只安装、编译和 DISARMED 验证。不要立即启用控制。详细命令见 `NEXT_STEPS.md`。
