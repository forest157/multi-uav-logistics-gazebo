# v0.3 动态障碍与在线风险预测

## 本批目标

第一批先建立可复用的动态环境基础，不直接把未经验证的绕行动作接入飞控：

1. Gazebo 中生成具有真实碰撞体的移动障碍物；
2. 通过 ROS 话题发布统一的三维位置、速度、半径和高度；
3. 对三机短时目标航段做时间索引净空预测；
4. 在 rqt 上位机提供实验开关和 SAFE/WARNING/CRITICAL 状态；
5. 默认 launch 参数关闭，保持 v0.2.2 的任务行为兼容。

## 数据接口

- `/dynamic_obstacles/state`：JSON，40 Hz，包含时间戳、坐标系和障碍物状态；
- `/dynamic_obstacles/markers`：RViz MarkerArray；
- `/fleet/dynamic_risk`：三机风险摘要，5 Hz；
- `/fleet/dynamic_diagnostics`：标准 ROS diagnostics。

当前预测采用障碍物短时匀速模型，无人机采用“当前位置到当前目标点”的短时线性航段。
它适合作为 v0.3 的风险检测基线；下一批将在该接口上接入局部重规划和速度避让。

## 运行方法

只启动动态障碍（已有 Gazebo 时）：

```bash
roslaunch logistics_gazebo_sim_ros dynamic_obstacles.launch
```

三机任务启用动态实验：

```bash
roslaunch logistics_gazebo_sim_ros three_uav_mission.launch dynamic_obstacles:=true
```

rqt 上位机默认勾选“启用交叉移动障碍物与在线风险预测”，可以取消勾选以复现
v0.2.2 的静态环境行为。

## 2026-08-08 验收记录

- 新增纯算法测试 6 项，工程全部 28 项单元测试通过；
- catkin build 成功，无警告；
- 动态状态实测约 40 Hz；
- 在线风险摘要实测约 5 Hz；
- Gazebo `get_model_state` 可查询动态模型，模型位置随时间变化；
- 动态鸟形模型使用保守球形碰撞体，停止节点后自动删除；
- 上位机可显示最近无人机、预测最小净空与冲突倒计时；
- 默认 `dynamic_obstacles:=false`，原三机任务启动参数不变。

## 第二批：编队级安全响应

- WARNING：任务虚拟时钟降为 35%，三架无人机同步减速，队形相位不被打乱；
- CRITICAL：锁存三机当时的本地位置作为目标，执行同步悬停；
- 恢复：风险连续 SAFE 2 秒后才解除悬停，避免边界抖动反复启停；
- STALE：不会凭空触发新悬停；已经锁存时则维持悬停；
- 手动重置任务会同时清空风险锁存和虚拟时钟状态；
- `/fleet/mission_state` 增加 `dynamic_action`、`dynamic_risk` 和
  `speed_scale` 字段，供 rqt 显示与日志分析。

在线注入验收中，CRITICAL 后三机目标被固定为各自当前落地点附近位置，
`dynamic_action=HOLD`、`speed_scale=0.0`；持续 SAFE 超过 2 秒后恢复为
`dynamic_action=NORMAL`、`speed_scale=1.0`。全工程 31 项测试通过。

## 下一批

1. 将动态风险接入局部三维轨迹候选生成；
2. 评估 ROS MoveIt/OMPL 重规划与速度障碍法的组合；
3. 先用减速/悬停作为安全回退，再启用局部绕行；
4. 编队原则保持，窄通道时允许受控三维重构；
5. 增加鸟类横穿、迎面、遮挡后出现及多障碍压力场景。
