# 新对话交接：multi-uav-logistics-gazebo

更新时间：2026-09-01

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
- 当前开发分支：`feature/v050-perception-tracking`。
- v0.5.0 已完成三机 16 线 Gazebo 雷达、点云融合、背景过滤、聚类、跨帧确认、RViz 和上位机感知源选择。
- v0.5.1 完成感知稳健化：新鲜空检测为 SAFE，断流才 STALE；按鸟类尺寸过滤静态碎片；上位机显示过滤诊断。

## 关键安全语义

- `dynamic_state_source=perception` 是默认稳定模式。
- `dynamic_state_source=lidar` 是物理雷达实验模式。
- 空目标列表不等于数据断流。新鲜空列表必须 SAFE；超过 1 秒没有消息必须 STALE 并悬停。
- 物理雷达候选必须经过地面/机体过滤、背景学习、尺寸门控、跨帧三次确认。
- 不允许通过降低机间距、动态净空或超时门槛掩盖问题。

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

1. v0.5.1 已完成 271.89 秒三机物理雷达全任务回归，下一步先核对 main 与 v0.5.1 标签。
2. 后续补充独立的无鸟与单鸟矩阵；双鸟完整任务已经通过。
3. 若 v0.5.1 通过，合并到 main 并打 `v0.5.1` 标签。
4. 再讨论 v0.5.2：运动一致性门控、遮挡后的轨迹维持、检测置信度标定；不要直接扩大到电量返航或新世界地图。

## 已知边界

- 当前物理雷达检测仍依赖简单体素背景与欧式聚类，不是完整 SLAM/OctoMap 动静分离。
- 鸟离开雷达范围时应发布新鲜空列表，任务正常继续。
- OMPL 路线为越过建筑可能从巡航高度爬升后下降，这是规划行为；高频上下振荡才是故障。
- 分布式 MPC 仍为 shadow，ORCA limited 才允许受限接管。
