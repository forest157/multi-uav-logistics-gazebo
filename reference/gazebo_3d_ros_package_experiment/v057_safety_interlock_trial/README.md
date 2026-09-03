# v0.5.7 安全联锁 ROS 自动验收

日期：2026-09-03

## 目标

- 将 v0.5.6 的实际位姿安全联锁验证固化为可重复运行的 ROS 场景。
- 验证联锁不只在纯函数测试中有效，而是能通过真实话题控制任务目标。

## 实现

- 新增 `safety_interlock_trial.launch` 和 `safety_interlock_trial`。
- 启动真实 `fleet_mission_player`，通过 MAVROS 位姿话题注入三机实际位置。
- 通过 `/fleet/diagnostics` 验证 `STALE` 不误触发。
- 将相邻机间距压缩到 2.5 m，验证 2.7 m 联锁阈值、同步目标锁定和 JSON 诊断。
- 恢复正常间距后保持超过 1 秒，验证 3.0 m 释放阈值和滞回延时。

## 运行

```bash
roslaunch logistics_gazebo_sim safety_interlock_trial.launch \
  report_path:=/tmp/safety_interlock_report.json
```

## 验证结果

- `close_minimum_separation_m`: 2.5
- `hold_triggered`: true
- `targets_locked`: true
- `released`: true
- `stale_triggered`: false
- `pass`: true
- 119 项 Python 测试全部通过。
- catkin 构建成功且无警告。

v0.5.6 的完整三机飞行验证仍有效：任务完成并解除武装，最小机间距 3.182 m，避障 Z 偏移为 0。
