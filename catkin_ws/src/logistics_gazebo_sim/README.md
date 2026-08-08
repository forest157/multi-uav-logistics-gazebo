# logistics_gazebo_sim

ROS Noetic + Gazebo Classic 11 + PX4 v1.13.2 oziO�SU+ 7 k	 0.2kߋ{�: 100 m � 100 m ��~o_��[�h�RVizs��6rqtnOM~� CSV��U

## �
o:

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
source ~/PX4_Firmware/Tools/setup_gazebo.bash ~/PX4_Firmware ~/PX4_Firmware/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4_Firmware:~/PX4_Firmware/Tools/sitl_gazebo
roslaunch logistics_gazebo_sim operator_station.launch
```
_( rqtn�	�~o|��\�o~�?�_�� READYt��Y�go�|��g�M}�\n�RViz ���yo:�w����

�ϰ����e `~/.ros/logistics_runs/mission_*.csv�PX4 ULogoM� SITL}�~�U� `log/` ;

##v���

```bash
roslaunch logistics_gazebo_sim three_uav_mission.launch \
  scene_id:=0 spawn_x:=-40 spawn_y:=-40 target_z:=5 \
  mission_config:=$(rospack find logistics_gazebo_sim)/config/mission_scene0.yaml
```
_~o�u1Oo:��9}scene 06 ���oM� `config/mission_scene*.yaml`

## 往返任务流程

起点起飞 → 出发编队 → 前往配送点 → 投递悬停 → 返航 → 起点恢复编队 → 起点降落并自动解锁。紧急降落会跳过返航并在当前位置降落。

## 轨迹算法

航迹采用“编队安全膨胀 A* → LOS 简化 → 原项目自定义三次 B 样条 → TOPPRA 时间最优参数化”。当前缩放场景使用 2.0 m/s 合速度和 1.0 m/s² 合加速度目标约束，TOPPRA 以零起终速度生成去程轨迹，返程按同一时间律反向执行。依赖：`python3 -m pip install --user scipy toppra`。

## 自定义规划与任务进度

上位机支持自选起点、终点、飞行高度和正三角/倒三角/横队。点击启动前使用编队膨胀的 NumPy 占据栅格 A* 规划；不可行时弹窗并禁止启动。支持任务重置，并通过只读进度条实时反映 Gazebo 物理仿真进度；实时物理仿真不支持拖动快进。
