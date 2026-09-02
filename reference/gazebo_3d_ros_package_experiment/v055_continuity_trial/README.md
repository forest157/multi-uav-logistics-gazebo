# v0.5.5 轨迹连续性自动验收

日期：2026-09-02

## 目标

- 将短时遮挡 ID 连续性验证固化为可重复运行的 ROS launch。
- 按连续可见周期衡量任务 CSV 中的 ID 连续性，避免把长时间离场后的新轨迹误判为 ID 跳变。

## 实现

- 新增 `track_continuity_trial.launch` 和 `track_continuity_trial`。
- 验收数据实际经过 `DetectionAssociator`、`/perception/dynamic_detections`、`dynamic_target_tracker` 与 `/perception/dynamic_tracks`。
- 自动模拟匀速目标确认、0.6 秒遮挡和按预测位置重现，输出机器可读 JSON，并以退出码表示通过或失败。
- `analyze_track_reliability` 新增可见周期数、连续周期数、周期内 ID 跳变数和 ID 连续率。

## 运行

```bash
roslaunch logistics_gazebo_sim track_continuity_trial.launch \
  report_path:=/tmp/track_continuity_report.json
```

## 验证

- 112 项 Python 测试全部通过。
- catkin 构建成功且无警告。
- ROS 自动验收通过：3 个遮挡样本、最大遮挡 0.6 秒，遮挡前后均为 `lidar_target_0`。
- 对 v0.5.4 完整任务 CSV 复算：12 个可见周期全部保持单一 ID，周期内 ID 跳变 0，连续率 1.0。
- 原始任务记录：`~/.ros/logistics_runs/mission_20260902_133005.csv`。

## 结论

此前完整任务出现 11 个唯一 ID，来自 12 个彼此分隔的进出量程周期；新指标确认这些周期内部没有 ID 跳变。短遮挡连续性同时由独立 ROS 链路验收覆盖。
