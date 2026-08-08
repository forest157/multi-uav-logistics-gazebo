# Multi-UAV Logistics Gazebo Simulation

三机物流配送 Gazebo/PX4/ROS 仿真工程。本仓库按容器内原始目录布局保存，便于完整恢复。

## 目录

- `catkin_ws/src/logistics_gazebo_sim`：稳定版本
- `catkin_ws/src/logistics_gazebo_sim_ros`：ROS/OMPL/TOPPRA 三维规划实验版本
- `reference/gazebo_3d_upgrade_docs`：升级设计、决策和测试记录
- `reference/gazebo_3d_ros_package_experiment`：ROS 算法实验说明

构建产物、ROS/Gazebo/PX4 运行日志、缓存及用户配置均不进入版本控制。

## 版本历史与后续计划

已发布版本、当前能力边界以及 ORCA、MPC、传感器、电量返航、扩编和实机迁移计划见 [`ROADMAP.md`](ROADMAP.md)。

## 恢复

将仓库克隆到容器的 `/home/devuser`，安装 README/文档中记录的系统依赖，然后在工作空间中构建：

```bash
cd /home/devuser/catkin_ws
source /opt/ros/noetic/setup.bash
catkin build logistics_gazebo_sim logistics_gazebo_sim_ros
```

实验版控制台：

```bash
source /home/devuser/catkin_ws/devel/setup.bash
roslaunch logistics_gazebo_sim_ros operator_station.launch
```

## 版本策略

- `main`：已验证、可恢复的基线
- `develop`：后续集成开发
- 功能与修复使用 `feature/*`、`fix/*` 分支
