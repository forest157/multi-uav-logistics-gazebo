# v0.4.1 可插拔局部避障与可扩展机队接口

## 目标

本版本在 v0.3 已验证的三机整队偏移避障基础上，建立后续增加无人机数量和替换局部算法所需的稳定接口。

## 架构

- `collective_offset`：v0.3 稳定算法，继续作为默认闭环执行方案。
- `orca3d`：不依赖外部二进制的球形三维 ORCA 原型，输出逐机速度建议。
- `dynamic_avoidance_planner`：只依赖统一规划器工厂，通过 ROS 参数或请求字段选择算法。
- 风险监测、安全悬停和算法进程隔离保持不变。

统一规划结果包含 `algorithm`、`command_type`、`vehicle_count`、`viable` 和失败原因。整队算法输出 `selected_offset`；ORCA 输出 `commands[]`，每项包含无人机编号、期望速度、修正速度和约束数量。

## 机队规模

纯算法、风险监测、RViz、CSV 记录、安全监测和包裹管理接口支持 1～8 架无人机，并使用统一的居中生成点和颜色配置。任务生成器原本已经支持 `--vehicle-count` 和可扩展三维队形。

当前 PX4 launch 仍保留经过实飞验证的三机实例。4～8 架 PX4 的端口生成、资源压力测试和上位机数量选择将在 0.4.2 完成；0.4.1 不宣称已完成八机闭环飞行。

## ORCA 安全策略

- 对无人机和动态障碍建立三维球形速度约束。
- 完全对称的正面对撞使用确定性相反侧向速度破除对称。
- 速度始终限制在配置的最大速度内。
- 输出前再次采样预测机间距，约束不满足时返回 `viable=false`。
- v0.4.1 中 ORCA 为影子模式，不直接接管 PX4；现有减速和同步悬停仍为安全回退。

## 参数

```xml
<arg name="vehicle_count" default="3"/>
<arg name="local_avoidance_algorithm" default="collective_offset"/>
```

实验 ORCA：

```bash
roslaunch logistics_gazebo_sim_ros three_uav_mission.launch \
  dynamic_obstacles:=true local_avoidance_algorithm:=orca3d \
  dynamic_avoidance_execution:=false
```

## 验收

- 60 项 Python 测试通过。
- 3D ORCA 覆盖正面对撞、移动障碍、速度上限和八机输出。
- catkin 构建成功，无警告。
- ROS 请求/响应验证要求返回 `algorithm=orca3d`、`command_type=per_vehicle_velocity`。
- 中文 README 已从 reference 正确版本恢复为 UTF-8。
