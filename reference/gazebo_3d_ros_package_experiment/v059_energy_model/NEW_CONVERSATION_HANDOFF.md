# 新对话交接：v0.5.9 已发布，继续 v0.5.10

仓库位于 Docker 容器 `gazebo-dev` 的 `/home/devuser`，ROS 包位于 `/home/devuser/catkin_ws/src/logistics_gazebo_sim`。开始前核对容器、分支、工作区、SSH 和远端；电脑重启后尤其要重新确认 SSH 配置仍通过 `ssh.github.com:443`，使用 `~/.ssh/id_ed25519_multi_uav_github`。

## 已完成

v0.5.9 已完成可重复的能量估计基线：悬停、水平速度、升降、加速度、转弯、载荷、容量差异和安全储备均进入模型；ROS 输出逐机剩余量、任务/返航/等待/降落需求和最终余量；MAVROS 电池数据被记录为标定参考。该版本不接管飞控决策。

验收包括 127 项测试、catkin 构建、能量专项 ROS 试验以及三机完整 Gazebo 任务。完整任务 `COMPLETE`、三机解除武装、每机估算约 18.56 Wh、最小机间距 3.211 m。完成态的 `required_to_land_wh` 已专项验证为 0。

## 下一步：v0.5.10

按 `ROADMAP.md` 实现电量感知返航和槽位调整：

1. 先以影子模式计算返航可行性和低能量告警，不直接改飞控；
2. 使用 v0.5.9 的 `usable_margin_wh` 与 `required_to_land_wh`，加入滞回、数据时效和故障安全语义；
3. 在开阔区为低余量无人机选择距离更短、爬升和转弯更少的返航槽位；
4. 安全联锁、真实位姿间距、空域上下界和感知降级始终优先于能量优化；
5. 再实现降落顺序、备用降落点和临界脱队策略，并通过不开桨、专项 ROS 和完整 Gazebo 验收。

不要重新实现或改写 v0.5.0～v0.5.9。当前能量系数是 `simulation_baseline_unfitted`，SITL 电池百分比接近 1，不能据此宣称完成 PX4/实机标定。
