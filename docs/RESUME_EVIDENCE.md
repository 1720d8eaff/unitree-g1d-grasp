# 简历技术证据索引

| 技术点 | 代码 / 配置 | 测试 | 验证范围 |
| --- | --- | --- | --- |
| AnyGrasp 与候选选择 | `laptop/anygrasp/` | `tests/test_project_integrity.py` | 工程源码与离线完整性检查；实际运行依赖 SDK 与权重 |
| MoveIt、IK 与碰撞检查 | `laptop/pipeline/`, `laptop/moveit/` | `tests/test_project_integrity.py` | 工程及配置完整性；现场执行需配置环境 |
| ROS 2、SDK2 和手部 | `onboard/trajectory_bridge/`, `hand/` | `tests/test_project_integrity.py` | 拆分执行链及离线检查 |
| FailureVLA 集成 | `integrations/failurevla_bridge.py` | `tests/test_failurevla_bridge.py` | 注入式回调接口和模拟候选测试；实际感知、规划与执行回调需现场接入 |
| 失败和异步组件 | `vla/failure_adapter.py`, `vla/action_buffer.py` | `tests/test_vla_components.py` | 接口及梯度测试；机器人任务指标由真实运行日志提供 |
