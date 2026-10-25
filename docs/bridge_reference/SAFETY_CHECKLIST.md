# 安全检查清单

- 实体急停可随时按下，现场有人看护。
- 机器人周围和双臂运动范围内无人员、桌面和障碍物。
- 启动前 `/lowcmd` 必须 Publisher count: 0，Subscription count: 1。
- 不得同时运行旧 `trajectory_executor`、任何 lowcmd 微动脚本或 arm_sdk 示例。
- 首次只做双进程 DISARMED 验证；不得调用 `{data: true}`。
- 第一次真实轨迹仍只允许右腕 yaw 约 +3°、约 4 秒。
- 错误关节运动、快速运动、抖动、下坠、撞击趋势或异常声音：立即实体急停。
