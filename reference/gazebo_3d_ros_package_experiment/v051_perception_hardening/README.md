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
