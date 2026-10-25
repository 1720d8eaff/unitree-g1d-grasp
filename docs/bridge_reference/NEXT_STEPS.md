# 下一步：只安装、编译与 DISARMED 验证

## 1. 板载机安装和编译

将 `onboard` 目录复制到 `/home/unitree/g1_real_bridge_split_onboard`，然后：

```bash
cd ~/g1_real_bridge_split_onboard
bash install_onboard.sh
conda deactivate 2>/dev/null || true
source /opt/ros/foxy/setup.bash
cd ~/ros2_ws
rm -rf build/g1_d_trajectory_bridge install/g1_d_trajectory_bridge
colcon build --packages-select g1_d_trajectory_bridge --event-handlers console_direct+
source ~/g1_trajectory_bridge_env.sh
ros2 pkg executables g1_d_trajectory_bridge
```

必须看到 `trajectory_backend`、`trajectory_frontend`、`trajectory_executor`（安全阻断入口）和 `trajectory_topic_dry_run`。

## 2. 启动纯 Unitree 后端（终端 1）

```bash
source ~/g1_trajectory_bridge_env.sh
ros2 topic info -v /lowcmd
~/ros2_ws/src/g1_d_trajectory_bridge/run_backend.sh
```

后端必须显示 DISARMED。它此时只订阅 LowState，不创建 LowCmd。

## 3. 后端启动后确认 `/lowcmd`（终端 2）

```bash
source ~/g1_trajectory_bridge_env.sh
ros2 topic info -v /lowcmd
```

仍必须 Publisher=0、Subscription=1。

## 4. 启动 ROS 前端（终端 2）

```bash
source ~/g1_trajectory_bridge_env.sh
~/ros2_ws/src/g1_d_trajectory_bridge/run_frontend.sh
```

前端应显示已连接本机后端。此时仍不得调用启用服务。

## 5. DISARMED 端点检查（终端 3）

```bash
source ~/g1_trajectory_bridge_env.sh
ros2 node info /g1_d_trajectory_executor
ros2 service type /g1_d/set_trajectory_control
ros2 topic info -v /g1_d/trajectory_command
ros2 topic echo --once /g1_d/trajectory_backend_state
ros2 topic info -v /lowcmd
```

不要运行：

```bash
ros2 service call /g1_d/set_trajectory_control std_srvs/srv/SetBool "{data: true}"
```
