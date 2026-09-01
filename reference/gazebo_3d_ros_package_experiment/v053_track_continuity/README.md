# v0.5.3 遮挡轨迹维持与置信度标定

日期：2026-09-01

## 目标

- 让短时遮挡轨迹按真实时间维持和衰减，不受点云回调频率影响。
- 将点云支撑度与运动一致性合成为可解释的检测置信度。
- 解耦雷达点云聚合与可视化参数，关闭可视化不再切断物理感知。

## 实现

- 跟踪维持窗口由 1.0 秒调整为 1.5 秒，遮挡期间继续按恒速模型外推。
- 遮挡置信度按 `observed_confidence * exp(-0.7 * occluded_for_s)` 衰减，下限 0.15；重捕获后恢复观测置信度。
- 动态轨迹增加 `occluded_for_s`，保留 `observed` 标志。
- 确认置信度由 65% 点云支撑度与 35% 运动一致性构成。
- `/perception/status` 增加 `mean_confidence`，上位机显示确认目标平均置信度。
- 新增 `lidar_cloud_aggregation` 参数；`lidar_visualization:=false` 不再关闭检测所依赖的聚合器。

## 验证

- 105 项 Python 单元测试全部通过。
- catkin 构建成功，无警告。
- ROS launch 展开验证：关闭 `lidar_visualization` 时 `/lidar_cloud_aggregator` 仍存在；显式关闭 `lidar_cloud_aggregation` 时才移除。
- scene 0、单鸟、三机物理雷达、collective_offset 闭环在 `lidar_visualization:=false` 下完成全任务。

完整任务结果：

- 270.17 秒进入 COMPLETE，三机均解除武装并进入 AUTO.LAND。
- SAFE 1656、WARNING 94、CRITICAL 26；真实目标检测与动态响应有效。
- 最大偏移 XYZ 为 1.860 / 2.018 / 0.029 m，没有空域自发上爬。
- 最小机间距 3.227 m，最小动态净空 2.230 m。
- 原始记录：`~/.ros/logistics_runs/mission_20260901_182338.csv`。
