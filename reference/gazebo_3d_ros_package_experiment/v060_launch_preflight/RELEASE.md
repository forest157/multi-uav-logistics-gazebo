# v0.6.0：八机 PX4 SITL 与分组通行

本版按 [`ROADMAP.md`](../../../ROADMAP.md) 的 v0.6.0 范围，解除规模试验对三机固定 launch 的依赖。稳定的三机物理雷达任务保持原入口；规模试验使用独立的轻量 `iris` 世界与控制链路，**不声称八机雷达/动态避障能力**。

## 交付

- `sitl_instance.launch` 按实例 ID 推导命名空间、MAVROS 系统号和每机端口；新 `fleet_scale_sitl.launch` 支持 1、3、5、8 架，其他数量在解析阶段拒绝。世界 `outdoor_scale_yard` 的八个出生位、两栋 22 m 碰撞建筑和 JSON 元数据经 SDF 一致性校验。
- `preflight_scale_sitl` / `launch_scale_checked` 失败关闭地检查 CPU、可用内存、世界容量及所有 TCP/UDP 端口；八机检查 73 个端口，检查后启动默认不解锁。
- `scale_group_controller` 需操作员显式调用 `start`。解锁前逐机设置并读回 PX4 的 `COM_RCL_EXCEPT` Offboard 位，避免全新实例因 RC-loss 自动进入 `AUTO.RTL`。走廊几何容量为三机，五机按 3+2、八机按 3+3+2 自动分组；通行和返航均由实测到位事件驱动，而非仅靠固定计时。
- 遥测过期、MAVROS 断连、机间距小于 2.7 m、静态障碍包络侵入、已起飞后退出 `OFFBOARD` 或阶段超时均停止推进并悬停等待人工处置；不会压缩整队强行穿越。
- `benchmark_scale_passage` 以 20、35、50 架轻量运动学实例压测分组排程；这不是 PX4 实飞。

## 验收与边界

单机、三机、五机、八机依次运行；实际飞行均从起点出发，分组穿过狭廊到达终点，再逆序分组返航并解除武装。`audit_scale_flight` 独立读取 Gazebo 真值，检查全阶段覆盖、机间距、建筑包络与最终解锁状态。详细指标见同目录的 `V060_SCALE_ACCEPTANCE_OCT06.json`。

完整 Python 回归 273 项通过；七个 catkin 包构建成功且零警告。20/35/50 架运动学压力测试均通过。八机飞行 32 个阶段全覆盖，最小机间距 3.102 m、建筑包络净空 1.001 m、零危险事件、八架全部解除武装。

本版不包含八机 lidar 感知、鸟类动态避障、ORCA/MPC 八机闭环、自动任务分配或外部 Baylands 世界的飞行许可；这些能力不能从轻量规模验收外推。Gazebo Classic / PX4 旧环境会拒绝若干降落参数设置并可能在退出时打印关停异常；飞行结束与解除武装以独立审计为准。

## 复现

```bash
source /opt/ros/noetic/setup.bash
source /home/devuser/catkin_ws/devel/setup.bash
cd /home/devuser/catkin_ws/src/logistics_gazebo_sim
scripts/launch_scale_checked --count 8 --dry-run
scripts/launch_scale_checked --count 8 --gui false
# 另一终端先启动独立审计，再人工确认环境后显式起飞：
scripts/audit_scale_flight --count 8 --output /tmp/v060_flight_8.json
rosservice call /scale_group_controller/start
rosservice call /scale_group_controller/status
scripts/benchmark_scale_passage
```

参见 [开发与安全细节](README.md)。
