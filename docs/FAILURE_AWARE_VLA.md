# 失败表征与异步动作组件

该目录提供可与语言条件动作策略结合的训练和执行组件。`vla/failure_adapter.py` 使用 GRU 编码短时状态历史，输出失败类别和任务进度，并通过残差动作头调整策略给出的动作块。`vla/action_buffer.py` 提供线程安全的动作块替换、逐步读取和提前重规划触发。

## 失败表征与训练接口

安装 PyTorch 后使用：

```python
import torch
from vla.failure_adapter import FailureAdapter, training_loss, sampling_weights

adapter = FailureAdapter(state_dim=14, action_dim=14)
# 替换为数据加载器和策略模型的输出
history = torch.randn(8, 10, 14)       # [batch, history, state]
base = torch.randn(8, 16, 14)          # [batch, horizon, action]
target = torch.randn(8, 16, 14)
failure = torch.zeros(8, dtype=torch.long)
progress = torch.rand(8)              # 归一化至 [0, 1]
output = adapter(history, base)
loss = training_loss(output, target, failure, progress)
loss.backward()
weights = sampling_weights(failure, hard_case_weight=3.0)
```

上面的随机张量仅用于演示接口。类别编号依次为正常、视觉定位、抓取、执行偏移、目标变化、提前终止。`sampling_weights` 可用于 PyTorch 的 `WeightedRandomSampler`；划分数据集时应按完整轨迹划分，再在训练集内采样。接入 SmolVLA 或其他策略时，需将策略动作块及对应状态历史转换成上述张量，并使用同一动作表示和归一化方式。

## 状态对齐的动作块替换

```python
from vla.action_buffer import ActionBuffer, Chunk

buffer = ActionBuffer(max_age=0.25, max_state_error=0.1)
chunk = Chunk(request_id=0, observed_at=10.0,
              reference_state=(0.0, 0.1), actions=((0.2, 0.3), (0.4, 0.5)))
accepted, reason = buffer.replace(chunk, state=(0.01, 0.1), now=10.1)
if accepted:
    action = buffer.pop()
    if buffer.needs_replan(remaining_threshold=1):
        pass  # 提交下一次推理请求
```

推理请求应携带单调递增 ID、采样时刻和参考状态；返回后使用同一单调时钟及最新状态调用 `replace`。缓冲器拒绝过期、乱序、状态偏差过大或包含非有限数值的块，并保留此前接受的队列。`max_state_error` 是各状态分量绝对偏差的最大值；状态分量应统一量纲或事先归一化。调用方需根据控制频率和机器人状态设定阈值，在执行时继续检查动作有效期、限位与现场安全条件。该组件只返回动作，不发送硬件命令。

## 验证

```bash
python -m unittest discover -s tests -p 'test_vla_components.py' -v
```

测试覆盖动作块替换、过期和状态偏差拒绝、重规划阈值及训练梯度。训练接口测试需要 PyTorch。
