# 来源与变更

- 笔记本 relay：来自已成功完成 MoveIt Execute dry-run 的 Phase 1 源码。
- 后端控制、安全阈值、关节映射、500 Hz writer、MotionSwitcher 流程：来自 Phase 1 `trajectory_executor.py` 与已验证真机脚本。
- Phase 2 仅将 ROS 2 接口与 Unitree DDS 控制拆成两个进程，并增加本机长度前缀 JSON 协议、单实例锁及启用前 `/lowcmd` 独立进程预检。
- 旧单进程 `trajectory_executor` 入口保留为安全阻断程序，防止误启动已知不稳定架构。
