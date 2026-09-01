# v0.5.1 物理雷达感知稳健化

日期：2026-09-01

## 目标

- 区分“新鲜空检测”和“感知数据断流”。
- 无动态障碍时风险状态为 SAFE，不再错误进入 DYNAMIC_HOLD。
- 按真实鸟类尺寸过滤建筑、地面等大尺寸聚类碎片。
- 上位机显示原始簇、过滤数、候选数和确认目标数。

## 实现

- `obstacle_feed_state` 统一判断感知传输新鲜度。
- `target_sized_detections` 统一执行目标尺寸门控，默认最大半径 1.4 m、最大高度 2.0 m。
- 雷达检测状态新增 `raw_clusters` 与 `rejected_oversized`。
- 数据新鲜且障碍列表为空时发布 SAFE；超过 1 秒未收到数据时仍发布 STALE 并触发安全悬停。

## 验证要求

- Python 单元测试全部通过。
- `catkin build logistics_gazebo_sim` 成功。
- ROS launch 展开成功。
- 三机物理雷达场景连续飞行，不因空检测错误悬停。
- 动态鸟进入量程后仍能形成候选和确认轨迹。

## 2026-09-01 完整回归结果

- 96 项 Python 回归测试全部通过。
- catkin 构建与 ROS launch 展开通过。
- scene 0、双鸟、三机物理雷达、collective_offset 闭环任务在 271.89 秒进入 COMPLETE。
- 去程曾同时产生 50 个原始簇，其中 49 个建筑碎片被尺寸门控过滤，1 个真实目标完成跨帧确认。
- 投递、返航和下降阶段均保持 SAFE/NORMAL，没有错误 STALE 或 DYNAMIC_HOLD。
- 三架 PX4 最终全部解除武装，模式 AUTO.LAND。

## 2026-09-01 无鸟与单鸟补充矩阵

两轮均使用 scene 0、三机 16 线物理雷达、collective_offset 闭环，保持既有机间距、动态净空和超时门槛。

| 场景 | 结果 | 任务时间 | 最小机间距 | 最小动态净空 | 感知表现 |
| --- | --- | ---: | ---: | ---: | --- |
| 无鸟 | COMPLETE，三机解除武装并进入 AUTO.LAND | 281.17 s | 3.217 m | 2.312 m | 最终 0 候选、0 确认目标，但航程中出现 7 段短暂 WARNING，并触发 HOLD/SLOW/AVOID |
| 单鸟 | COMPLETE，三机解除武装并进入 AUTO.LAND | 263.88 s | 3.204 m | 2.188 m | 真实鸟形成 1 个跨帧确认目标并触发 WARNING/AVOID；同时存在与无鸟轮同型的额外短暂 WARNING |

原始任务记录：

- `~/.ros/logistics_runs/mission_20260901_172748.csv`（无鸟）
- `~/.ros/logistics_runs/mission_20260901_173452.csv`（单鸟）

结论：新鲜空列表语义正确，任务没有因空检测进入 STALE；真实目标检测链路也有效。但无鸟轮并未满足“全程 SAFE”，说明仅靠尺寸门控和三帧确认仍会把部分移动视角下的静态碎片误确认为动态目标。下一步应在 v0.5.2 增加运动一致性门控，不应通过放宽安全距离或超时来掩盖该问题。

复现注意：非交互 shell 必须显式加载 `Tools/setup_gazebo.bash`；`lidar_visualization:=false` 当前也会关闭物理雷达检测所依赖的点云聚合器。
