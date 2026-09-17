# 新对话交接：multi-uav-logistics-gazebo

更新时间：2026-09-17

## 新对话首条提示词

请读取 `reference/gazebo_3d_ros_package_experiment/v051_perception_hardening/NEW_CONVERSATION_HANDOFF.md`，进入 Docker 容器 `gazebo-dev`，在 `/home/devuser` Git 仓库继续开发。先核对分支、工作区、SSH 和最近提交，再按“下一步”继续；不要重做已完成版本。

## 环境与仓库

- 容器：`docker exec -it -u devuser gazebo-dev bash`
- Git 根目录：`/home/devuser`
- ROS 主包：`~/catkin_ws/src/logistics_gazebo_sim`
- GitHub：`git@github.com:forest157/multi-uav-logistics-gazebo.git`
- SSH 必须走 `ssh.github.com:443`，密钥 `~/.ssh/id_ed25519_multi_uav_github`。
- 重启后先运行：`ssh -G github.com | grep -E "^(hostname|port|identityfile) "`。

## 版本状态

- `main`：v0.4.5 稳定基线。
- 当前开发分支：`feature/v0511-real-scale-worlds`，远端同名分支持续更新；稳定标签为 `v0.5.10`。
- v0.5.0～v0.5.10 已发布；当前按 `ROADMAP.md` 推进顺延后的 v0.5.11 真实尺度世界、物理雷达/ORCA 收口，随后做 v0.5.12 总验收。
- 已有三个模块化室外世界、一个室外物流场以及固定版本/校验值的第三方仓库资产获取与检查链路；不要重做这些资产工作。
- 物理雷达为项目自有 Gazebo RaySensor 插件，720×32、35 m、5 Hz；点云不向飞行节点提供 Gazebo 真值。
- ORCA limited 已用 MAVROS raw velocity 真正接管，带测得速度起点、控制延迟、全加速轨迹安全门、20 Hz 加速度限制、目标记忆与安全恢复。

## 关键安全语义

- `dynamic_state_source=perception` 是默认稳定模式。
- `dynamic_state_source=lidar` 是物理雷达实验模式。
- 空目标列表不等于数据断流。新鲜空列表必须 SAFE；超过 1 秒没有消息必须 STALE 并悬停。
- 物理雷达候选必须经过地面/机体过滤、正确场景静态地图及 1.5 m 边缘缓冲、背景学习、尺寸门控和跨帧确认。
- 不允许通过降低机间距、动态净空或超时门槛掩盖问题。
- `--allow-no-birds` 独立审计要求零鸟真值、零感知目标、飞行阶段零避障接管，并检查收敛后的高度误差，不能只看任务是否完成。

## 常用验证

```bash
cd ~/catkin_ws/src/logistics_gazebo_sim
source /opt/ros/noetic/setup.bash
export PYTHONPATH=$PWD/src:$PYTHONPATH
python3 -m unittest discover -s test -p "test_*.py" -q
cd ~/catkin_ws
catkin build logistics_gazebo_sim --no-status
```

启动上位机：

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export ROS_PACKAGE_PATH=$HOME/PX4_Firmware:$HOME/PX4_Firmware/Tools/sitl_gazebo:$ROS_PACKAGE_PATH
roslaunch logistics_gazebo_sim operator_station.launch
```

## 下一步

1. 单鸟横穿物理雷达 + ORCA limited 完整任务已通过：鸟机保守包络最小净空 2.5539 m、机间最小距离 3.1795 m、最大估计误差 0.5460 m，任务完成并解除武装。摘要见 `v0511_real_scale_worlds/ORCA_LIDAR_FULL_MISSION_SEP17.json`。
2. 纯无鸟完整任务已通过：4983/4983 有效样本，感知目标 0、飞行阶段避障接管 0、机间最小距离 3.2116 m、收敛后最大高度误差 0.4982 m。摘要见 `v0511_real_scale_worlds/NO_BIRD_LIDAR_FULL_MISSION_SEP17.json`。
3. 下一步做雷达/跟踪断流与恢复完整回归，必须验证新鲜空列表仍 SAFE、真正断流才 SLOW/HOLD、恢复后无旧目标跳变。
4. 然后做迎面动态障碍和整队动态避障回归，再完成至少一个室外世界的完整投递返航及 RTF/CPU/RSS 预算。
5. 矩阵完成后才更新 0.5 总验收、合并 `main` 和发布 `v0.5.11`/`v0.5.12`；不要提前声明 0.5 完成。

## 已知边界

- 当前物理雷达检测仍依赖已知场景地图、简单体素背景与欧式聚类，不是完整 SLAM/OctoMap 动静分离；室外世界必须提供匹配占据图。
- 鸟离开雷达范围时应发布新鲜空列表，任务正常继续。
- OMPL 路线为越过建筑可能从巡航高度爬升后下降，这是规划行为；高频上下振荡才是故障。
- 分布式 MPC 仍为 shadow，动态避障选择已收敛为 ORCA limited 主路径；整队偏移仅保留稳定回退。
- 独立审计使用采样真值和接收时间做验收，不是连续接触传感器证明；大型 JSONL 原始记录保留在容器 `/tmp`，Git 只提交摘要。
