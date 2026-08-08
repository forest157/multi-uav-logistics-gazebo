# logistics_gazebo_sim_ros

ROS Noetic + Gazebo Classic 11 + PX4 v1.13.2 oziO�SU+ 7 k	 0.2kߋ{�: 100 m � 100 m ��~o_��[�h�RVizs��6rqtnOM~� CSV��U

## �
o:

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
source ~/PX4_Firmware/Tools/setup_gazebo.bash ~/PX4_Firmware ~/PX4_Firmware/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4_Firmware:~/PX4_Firmware/Tools/sitl_gazebo
roslaunch logistics_gazebo_sim_ros operator_station.launch
```
_( rqtn�	�~o|��\�o~�?�_�� READYt��Y�go�|��g�M}�\n�RViz ���yo:�w����

�ϰ����e `~/.ros/logistics_runs/mission_*.csv�PX4 ULogoM� SITL}�~�U� `log/` ;

##v���

```bash
roslaunch logistics_gazebo_sim_ros three_uav_mission.launch \
  scene_id:=0 spawn_x:=-40 spawn_y:=-40 target_z:=5 \
  mission_config:=$(rospack find logistics_gazebo_sim_ros)/config/mission_scene0.yaml
```
_~o�u1Oo:��9}scene 06 ���oM� `config/mission_scene*.yaml`

## 往返任务流程

起点起飞 → 出发编队 → 前往配送点 → 投递悬停 → 返航 → 起点恢复编队 → 起点降落并自动解锁。紧急降落会跳过返航并在当前位置降落。

## 轨迹算法

v0.2 航迹采用“OMPL Informed RRT* 三维规划 → OMPL PathSimplifier/B 样条平滑 → 三维净空分析 → TOPPRA 时间参数化”。规划后会逐采样点检查每架无人机，而不是只检查编队中心。

路径队形调度器按首选队形、纵向一字、垂直错层、横队的顺序寻找可行方案；变换在安全窗口内连续插值，并强制保持最小机间距。返航按相反时间顺序复用已验证的队形计划。

## 自定义规划与任务进度

上位机支持地图点选起终点、飞行高度、队形和任务重置。参数改变后，旧结果立即失效并自动重新运行 OMPL、净空、阶段容量、队形调度和 TOPPRA 分析；未通过时禁止启动并显示错误分类、位置和处理建议。只读进度条实时反映 Gazebo 物理仿真进度；实时物理仿真不支持拖动快进。

## v0.2 验证

- 19 项 Python 单元测试通过。
- 7 个内置场景均通过 9 项任务阶段和逐机全轨迹检查。
