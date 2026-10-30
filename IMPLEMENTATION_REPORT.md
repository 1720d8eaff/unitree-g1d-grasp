# 实施报告

基线提交：`e43b91fd7d6acc1b985b975d3be009aa67a5a226`。在 `resume-alignment` 分支增量开发。

保留现有 AnyGrasp、MoveIt、ROS 2、SDK2 和 RH56H1 工程，新增 FailureVLA 恢复桥。桥接受观测、候选、碰撞、可达性、规划和可选执行回调，并按得分选择可规划候选。离线测试检查过滤、恢复预算与默认无执行回调；跨仓库使用模拟回调验证消息兼容。

验证范围：具体 AnyGrasp 服务、MoveIt 和设备回调需要按现场环境接入并验证；桥本身不重复实现 SDK 或直接发送硬件命令。机器人仓库的 PyTorch 组件以本机 PyTorch 验证，Linux 离线测试在无 PyTorch 环境中跳过该项。
