# logistics_gazebo_sim

基于 ROS Noetic、Gazebo Classic 11 和 PX4 v1.13.2 的三机物流配送仿真包。工程提供七种真实比例三维场景、编队飞行、实体包裹投递、自动返航、软降落、RViz 可视化、rqt 上位机和 CSV 任务记录。

## 与实验版的区别

- 稳定版包：`logistics_gazebo_sim`
- ROS 算法实验包：`logistics_gazebo_sim_ros`
- 稳定版保留原项目的自定义规划链路，实验版使用 ROS OMPL 三维规划。
- 两个包的节点名称相同，运行时只能启动其中一个。

## 环境准备

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
source ~/PX4_Firmware/Tools/setup_gazebo.bash \
  ~/PX4_Firmware ~/PX4_Firmware/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4_Firmware:~/PX4_Firmware/Tools/sitl_gazebo
```

## 启动上位机

```bash
roslaunch logistics_gazebo_sim operator_station.launch
```

上位机可以选择场景、起点、终点、飞行高度和巡航队形。参数改变后会重新规划；规划失败、起终点无效、高度或净空不足时会显示对应错误，未通过检查时不会启动仿真。

## 直接启动三机任务

```bash
roslaunch logistics_gazebo_sim three_uav_mission.launch \
  scene_id:=0 spawn_x:=-40 spawn_y:=-40 target_z:=5 \
  mission_config:=$(rospack find logistics_gazebo_sim)/config/mission_scene0.yaml
```

默认任务配置位于 `config/mission_scene*.yaml`，对应 scene 0～6。

## 往返任务流程

起点起飞 → 出发编队 → 前往配送点 → 投递点变为一字队形 → 下降并释放实体包裹 → 恢复巡航队形 → 返航 → 起点恢复降落队形 → 软降落并自动解除武装。

紧急降落会终止正常任务流程，在当前位置执行受控下降。任务重置会清除当前进度并恢复初始状态。

## 轨迹算法

稳定版使用以下链路：

1. 根据编队包络膨胀场景障碍物；
2. 使用 NumPy 占据栅格 A* 搜索中心航路；
3. 使用视线检测（LOS）简化路径；
4. 使用原项目自定义的三次 B 样条平滑。该 B 样条不是 ROS 标准规划包实现；
5. 使用 TOPPRA 对三维轨迹进行速度和加速度时间参数化；
6. 逐机检查轨迹、编队间距和障碍净空；
7. 通过 MAVROS 位置目标交给 PX4 OFFBOARD 控制器跟踪。

当前缩放场景使用约 2.0 m/s 合速度和 1.0 m/s² 合加速度约束，TOPPRA 以零起终速度生成去程轨迹，返航按验证后的时间律反向执行。

## 队形与安全

支持正三角、倒三角、横队等基础队形。队形变换采用连续插值，并在变换期间检查无人机之间的最小安全距离。遇到窄通道或当前参数不可行时，规划器会拒绝任务并提示调整高度、队形或起终点。

## 任务状态与记录

- RViz：显示场景、起终点、无人机位置、目标点和飞行轨迹；
- rqt：显示任务阶段、进度、安全状态和错误信息；
- CSV：保存在 `~/.ros/logistics_runs/mission_*.csv`；
- ROS/PX4/Gazebo 运行日志与构建缓存不进入 Git 版本库。

## 构建

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin build logistics_gazebo_sim
```

TOPPRA 和 SciPy 需在容器的 Python 3 环境中可用。
