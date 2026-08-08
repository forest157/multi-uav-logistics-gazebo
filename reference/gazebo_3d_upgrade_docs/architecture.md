# 三维仿真架构基线

## 已验证环境

- ROS Noetic，系统 Python 3.8
- Gazebo Classic 11.15.1
- PX4 Firmware v1.13.2（已有 `px4_sitl_default` 构建）
- MAVROS 已安装
- Catkin 工作空间：`/home/devuser/catkin_ws`

实现必须使用 Gazebo Classic 支持的 SDF 1.6，不能使用 Gazebo Sim（Ignition/GZ）系统插件。

## 边界

原始 `/home/devuser/reference/logistics` 只作为算法与场景参考，不直接成为 ROS
运行时依赖。ROS 代码位于 `catkin_ws/src/logistics_gazebo_sim`。规划、场景、PX4
适配和安全控制通过清晰接口解耦。

## 坐标和尺度

- Gazebo 与 ROS 外部接口统一使用 ENU 米制坐标。
- 参考地图坐标范围为 0..500，使用 0.2 m/unit，形成 100 m x 100 m 场地。
- 参考点 `(250, 250)` 映射至 Gazebo `(0, 0)`。
- PX4 NED 转换交由 MAVROS 接口完成，应用节点不自行重复变换。
- 水平速度基线由原 10 unit/s 调整为 2 m/s；加速度由 5 unit/s² 调整为 1 m/s²。

## 场景三维化

保留七类场景的二维 footprint：矩形、圆形、L 形和 T 形。Z 高度不是简单乘
水平比例，而是按语义设置真实高度，例如高楼 24–32 m、普通建筑 8–22 m、
工业塔体 16–35 m、低矮花坛/水池 0.3–1.5 m、山区体块 18–35 m。

首版使用垂直拉伸碰撞体。山峰暂为圆柱碰撞体，保证规划边界确定；后续视觉模型
可以换成锥台或网格，但不得缩小碰撞范围。

## 轨迹策略

参考工程的 B 样条不是标准 ROS spline，本项目不把其内部实现直接移植为飞控依赖。
其输出只用于行为对照。ROS 侧先采用分段轨迹和 MAVROS ENU setpoint，完成 PX4
轨迹层已接入参考项目的自定义三次 B 样条与 TOPPRA 时间参数化。所有轨迹接口从一开始包含 x/y/z、速度、
加速度、yaw、时间和阶段编号。
