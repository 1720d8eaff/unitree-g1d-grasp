# 运行依赖与环境边界

离线完整性测试只使用 Python 标准库，不需要安装 ROS 或连接机器人。真机部署分为多个运行环境，不能用一个 `pip install` 代替。

| 部分 | 原参考环境 | 外部依赖 |
| --- | --- | --- |
| 板端 ROS 前端与状态桥 | Ubuntu 20.04 / ROS 2 Foxy | `rclpy`、ROS 消息包、已构建的 ROS 工作空间 |
| 板端轨迹后端 | 独立 Unitree DDS Python 进程 | Unitree SDK2、CycloneDDS 与对应网络配置 |
| 板端视觉 | `tv_jetson` Conda 环境 | D435 / RealSense、原视觉管线的模型与依赖 |
| 上位机规划 | Ubuntu 22.04 / ROS 2 Humble | MoveIt 2、机器人描述、控制器与轨迹中继 ROS 包 |
| AnyGrasp 检测 | 独立 Conda 环境 | 单独安装并获授权的 AnyGrasp SDK、checkpoint、原环境的 PyTorch / CUDA 组合 |
| RH56H1 网关 | 上位机 Python 环境 | PySerial、串口权限、UDP 网络与实际硬件 |

安装入口分别位于各 ROS 包的 `package.xml` / `setup.py`、`laptop/bridge/install_laptop.sh`、`onboard/trajectory_bridge/install_onboard.sh`，以及[板端部署](BOARD_DEPLOYMENT.md)。原包未提供完整锁定的 Conda / pip 环境导出，本次整理不推测补写一个未经验证的统一依赖版本表。

## 部署时核对的配置

- `config/external_paths.json`：外部工作空间、AnyGrasp SDK / checkpoint 和 DDS XML 的参考路径。
- `config/processes_*.json`：启动命令及环境切换。
- `laptop/startup/`、`onboard/startup/`：部分机器相关安装路径和入口。
- `config/active_geometry.json`、`config/geometry/`：原现场几何与标定配置。
- 机器人描述中的外部资源引用及 ROS 工作空间安装结果。

源码来源通过 `config/onboard_source_provenance.json` 绑定精确字节。修改这些文件后，应按实际变更更新来源说明和预期哈希，再运行完整性检查；不要仅为让检查通过而覆盖来源记录。
