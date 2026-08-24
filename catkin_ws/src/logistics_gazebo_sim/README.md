# 多无人机物流 Gazebo 三维仿真

## 工程定位

`~/catkin_ws/src/logistics_gazebo_sim` 是当前唯一主包，ROS 包名为 `logistics_gazebo_sim`。原稳定版与实验版已经统一，旧实现保留在 Git 历史标签中，不再通过两个互相依赖的包并行维护。

## 算法链路

当前主线算法链路为：

1. ROS Noetic 提供的 OMPL 1.6。
2. 三维 RealVectorStateSpace(x, y, z)。
3. InformedRRTstar 路径长度优化。
4. OMPL PathSimplifier 的 reduceVertices、shortcutPath 和 smoothBSpline。
5. TOPPRA 三维速度/加速度时间参数化。
6. PX4/MAVROS 三机跟踪、投递与返航。
7. `collective_offset` 闭环避障与 `orca3d` 影子模式，外加独立风险监测和安全回退。

障碍物按编队尺度进行水平 4.5 m、竖直 2.0 m 膨胀。状态边界为 x/y ±46 m、z 3~45 m。规划器允许通过爬升越过低障碍物，输出轨迹字段为 `[t,x,y,z]`。

## 环境准备

完整环境安装可参考 [XTDrone 中文手册：基础配置](https://www.yuque.com/xtdrone/manual_cn/basic_config_13)。本项目默认 ROS Noetic、Gazebo、PX4 SITL、MAVROS、XTDrone 和 catkin 工作空间已经按该教程配置完成。

## 已安装依赖

- ros-noetic-ompl
- ros-noetic-octomap-server
- ros-noetic-navigation
- scipy
- toppra

OctoMap 和 Navigation 已安装用于后续传感器在线地图/二维基线对照；当前静态场景首先使用与 Gazebo 障碍物一致的三维膨胀体作为 OMPL validity checker，避免从仿真真值到 OctoMap 的离散误差影响首轮验证。

## 验证结果

v0.4.2 统一基线通过 65 项 Python 回归测试，`catkin build logistics_gazebo_sim` 构建成功。ORCA 已完成 Gazebo 三机受限闭环验证，并保留整队偏移与影子模式作为可切换方案。

七个场景均完成 OMPL 3D + TOPPRA 规划。典型场景 0：

- OMPL 平滑路径：约 166 个插值状态。
- TOPPRA 轨迹：约 840 个 0.1 s 采样。
- 高度范围：8.0~16.49 m。
- 未放宽速度/加速度约束。
- 任务播放器运行时目标高度验证覆盖同一范围。

场景 1 在 37 m 已高于主要障碍物，因此最优路径保持恒高；其余复杂低空场景会按需要爬升或侧向绕障。

## v0.4.2 闭环边界

ORCA 仅在 WARNING 且命令新鲜、车辆集合完整、动态净空、静态场景、地图边界、高度与机间距全部复核通过时接管。CRITICAL、超时、无解或约束失败均同步悬停；上位机可选择“整队偏移（稳定闭环）”“3D ORCA（影子模式）”或“3D ORCA（受限接管）”。

## 启动

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch logistics_gazebo_sim operator_station.launch
```
