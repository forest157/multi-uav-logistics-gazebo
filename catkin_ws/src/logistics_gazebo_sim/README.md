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
7. `collective_offset`、3D ORCA 与分布式 MPC 可插拔局部规划，外加独立风险监测和安全回退。

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

## v0.5.0 初版感知链路

三架无人机均挂载 16 线 3D 激光雷达，点云统一转换到 world 坐标系；实验检测链路包含地面与机体剔除、体素背景学习、欧式聚类、跨帧关联和三帧确认。RViz 可同时查看融合点云、确认目标和速度预测，上位机可选择稳定仿真感知、物理 3D 雷达实验感知或 Gazebo 真值对照，并显示背景学习、候选簇和确认目标数量。物理雷达尚属实验输入，默认仍使用稳定仿真感知。

## 验证结果

v0.4.5 稳定基线通过 84 项 Python 回归测试，`catkin build logistics_gazebo_sim` 构建成功。ORCA 已接入实时制动 TTC、规划等待和风险迟滞；分布式 MPC 完成统一影子对比，但仍不接管飞控。

七个场景均完成 OMPL 3D + TOPPRA 规划。典型场景 0：

- OMPL 平滑路径：约 166 个插值状态。
- TOPPRA 轨迹：约 840 个 0.1 s 采样。
- 高度范围：8.0~16.49 m。
- 未放宽速度/加速度约束。
- 任务播放器运行时目标高度验证覆盖同一范围。

场景 1 在 37 m 已高于主要障碍物，因此最优路径保持恒高；其余复杂低空场景会按需要爬升或侧向绕障。

## v0.4.4 算法边界

ORCA 仅在远期 `WARNING`、命令新鲜、车辆集合完整、动态净空、静态场景、地图边界、高度与机间距全部复核通过时接管。规划请求等待期间全队减速；进入按实时速度和制动能力计算的临界 TTC 后同步悬停。分布式 MPC 每机独立优化有限时域三维轨迹，并由独立安全检查复核；v0.4.4 仍只显示影子预测，无解时回退 ORCA，再失败则悬停。

## 离线算法对比

```bash
rosrun logistics_gazebo_sim benchmark_local_avoidance --cases 30 --json /tmp/avoidance.json --csv /tmp/avoidance.csv
```

输出整队偏移、3D ORCA 和分布式 MPC 的统一可行率、耗时、安全距离、轨迹偏差、平滑度和能耗代理。

## 启动

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch logistics_gazebo_sim operator_station.launch
```
