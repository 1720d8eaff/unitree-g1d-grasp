# 仓库核查修复记录

本次基于三个仓库最新主分支修复可运行代码。历史指标、真实数据量、设备现场标定和历史模型训练仍需要原始记录；本次新增结果保留自身来源，不回填历史事实。

## ContainerLoop-VLA

- 修复默认依赖：移除默认安装中的 GPU 模型栈与非法 Markdown 标记，保留原清单作为模型环境参考。
- 新增实际 MuJoCo 接触评测：重力、摩擦、圆柱接触代理、双指夹爪和位置伺服。48 个明确配置 × 5 组控制开关，共 240 次新实验，逐步 CSV、原始 JSONL、场景哈希和汇总全部保存于 `artifacts/contact/`
- 新增官方 GR00T checkpoint 加载入口、RGB/状态/语言模态组装及物理动作验证。官方 `get_action` 已完成物理量还原；新增显式 7D codec 单独处理标准化的 xyz/旋转向量/夹爪输入，不对官方输出二次反归一化。
- 修复原主分支 `tests/test_policy.py` 引用不存在的 `decode_chunk` 和缺失 fixture 导致的测试收集失败。改为手算 codec 测试；真实权重测试需要明确提供 checkpoint、观测 NPZ 和 embodiment，缺失时跳过。
- 新增 RGB/深度文件读取、Brown-Conrady 相机模型反投影、仿射时钟修正与丢帧间隙报告。真实相机内参必须从设备/标定提供。
- 从现有硬件仓库提取并保存关节名、DDS 索引、URDF 哈希和限位、RH56H1 夹爪、D435I 来源以及 `CONTROL_DT=0.002` 的绑定索引。500 Hz 发布仍由原 DDS 后端实现。
- 本地测试：18 通过、1 个真实权重测试跳过。

## FailureVLA

- 新增人工逐帧标注表导出/导入；拒绝空白和未知标签。新增标注轨迹到历史窗口和未来动作块的转换，按 episode 分割 train/validation/test 并记录源文件、标注哈希。没有从关节数据擅自推断视觉故障。
- 新增独立测试集评估：分类混淆矩阵、进度 MAE、动作 MSE 与基础动作误差；原训练入口可产生实际可读的 checkpoint。
- 新增可复现程序化 fixture 全链路 smoke run，实际运行两轮训练与测试。输出是 `synthetic_fixture`，日志及拆分记录在 `artifacts/synthetic_smoke/`，本地打包还包含实际训练出的 fixture 权重。
- 新增 LeRobot SmolVLA 的 `from_pretrained` 权重加载、RGB/状态/任务组装及 checkpoint 预后处理。未下载或运行真实 SmolVLA/G1-D 权重，未改变现有动作归一化假设。
- 新增 30 Hz deadline 调度与逐周期日志；在原 Executor 上实际运行同步、朴素异步和状态对齐异步三种轨迹回放，保存耗时与拒绝记录。
- 完整 episode 统计与写入拒绝缺失测量、重复 episode、无效恢复数及混合来源。部分诊断仍可用原宽松聚合接口。
- 数据加载器显式返回右臂信号可用性，保留原始全零通道。
- 本地测试：21 通过。B0–B7 的真实任务配置、策略权重、现场传感与恢复执行仍需配置。


