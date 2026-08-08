# 测试计划

1. 校验七个 world 均为合法 SDF 1.6，障碍物高度和地图变换合法。
2. 使用 `gzserver --verbose` 无界面加载每个 world，检查解析错误。
3. Catkin 编译并运行 Python 单元测试。
4. 启动 PX4/MAVROS 单机，验证连接、setpoint 预发送、OFFBOARD 和解锁。
5. 验证 ENU 的 x/y/z/yaw 方向。
6. 完成单机起飞、悬停、航点跟踪和降落。
7. 扩展三机命名空间、端口和 system-id，检查无串线。
8. 接入三维编队轨迹，记录跟踪误差、最小机间距和障碍物净空。

单元测试通过不代表可飞；PX4 集成必须以实际 `/mavros/state`、local pose 和 Gazebo
模型状态作为验收依据。
