# v0.5.8 感知验收矩阵与断流降级

日期：2026-09-03

## ROADMAP 对齐

本版本收口 `ROADMAP.md` 中 v0.5.0“真实传感器链路和动态目标跟踪”，并记录已发布标签与原规划编号的偏移。后续未完成工作顺延为 v0.5.9 能量模型、v0.5.10 电量返航、v0.5.11 真实尺度世界、v0.5.12 总验收封板。

## 实现

- 新增六场景确定性感知验收矩阵。
- 覆盖空域无目标、单目标噪声、短遮挡、长遮挡、双目标交叉和感知断流。
- 感知 `STALE` 时立即将任务速度降至警告比例。
- 连续断流 2 秒后同步悬停。
- 数据恢复后仍保持悬停，连续 `SAFE` 2 秒才恢复正常任务。
- 新增真实 `/fleet/dynamic_risk` 到 `/fleet/mission_state` 的 ROS 自动验收和 JSON 报告。

## 验证

- 感知矩阵 6/6 通过。
- 空域目标样本为 0；单目标噪声保持一个 ID。
- 短遮挡保持 `lidar_target_0`；长遮挡正确生成新 ID。
- 双目标交叉保持两个连续 ID。
- 断流动作序列为 `SLOW → HOLD → HOLD → NORMAL`。
- ROS 断流报告：`slowed=true`、`held=true`、`release_delayed=true`、`released=true`、`pass=true`。
- 121 项 Python 测试通过，catkin 构建无警告。
