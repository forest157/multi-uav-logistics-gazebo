# Multi-UAV Logistics Gazebo Simulation

三机物流配送 Gazebo/PX4/ROS 仿真工程。本仓库按容器内原始目录布局保存，便于完整恢复。

## 目录

- `catkin_ws/src/logistics_gazebo_sim`：统一的主 ROS 包，包含 Gazebo/PX4 仿真、OMPL 三维规划、TOPPRA 时间参数化和 ORCA 局部避障
- `reference/gazebo_3d_upgrade_docs`：升级设计、决策和测试记录
- `reference/gazebo_3d_ros_package_experiment`：算法演进与实验记录

构建产物、ROS/Gazebo/PX4 运行日志、缓存及用户配置均不进入版本控制。

## 环境安装

本项目运行环境的安装与基础配置可参考 [XTDrone 中文手册：基础配置](https://www.yuque.com/xtdrone/manual_cn/basic_config_13)。该教程用于准备 Ubuntu、ROS、Gazebo、PX4 SITL、MAVROS 和 XTDrone 等外部依赖；本仓库主要保存 `logistics_gazebo_sim` 业务代码、场景、配置和项目文档，不重复保存系统安装产物。

完成教程后，请确认容器内存在 `/home/devuser/PX4_Firmware`、ROS Noetic 可以正常加载，再按下方命令构建本项目；首次构建成功后才会生成 `~/catkin_ws/devel/setup.bash`。

## 版本历史与后续计划

已发布版本、当前能力边界以及 ORCA、MPC、传感器、电量返航、扩编和实机迁移计划见 [`ROADMAP.md`](ROADMAP.md)。

## 恢复

将仓库克隆到容器的 `/home/devuser`，安装 README/文档中记录的系统依赖，然后在工作空间中构建：

```bash
cd /home/devuser/catkin_ws
source /opt/ros/noetic/setup.bash
catkin build logistics_gazebo_sim
```

启动上位机：

```bash
source /home/devuser/catkin_ws/devel/setup.bash
roslaunch logistics_gazebo_sim operator_station.launch
```

## 版本策略

- `main`：已验证、可恢复的基线
- `develop`：后续集成开发
- 功能与修复使用 `feature/*`、`fix/*` 分支
