# 新对话交接：multi-uav-logistics-gazebo v0.5.2

更新时间：2026-09-01

## 新对话首条提示词

请读取 `reference/gazebo_3d_ros_package_experiment/v052_motion_consistency/NEW_CONVERSATION_HANDOFF.md`，进入 Docker 容器 `gazebo-dev`，在 `/home/devuser` Git 仓库继续开发。先核对分支、工作区、SSH、`main` 与 `v0.5.2` 标签，再按“下一步”继续；不要重做 v0.5.2。

## 环境与仓库

- 容器：`docker exec -it -u devuser gazebo-dev bash`
- Git 根目录：`/home/devuser`
- ROS 主包：`~/catkin_ws/src/logistics_gazebo_sim`
- GitHub：`git@github.com:forest157/multi-uav-logistics-gazebo.git`
- SSH 走 `ssh.github.com:443`，密钥 `~/.ssh/id_ed25519_multi_uav_github`。
- 非交互启动仿真必须显式加载：`source ~/PX4_Firmware/Tools/setup_gazebo.bash ~/PX4_Firmware ~/PX4_Firmware/build/px4_sitl_default`。

## 版本状态

- `main` 已包含并发布 v0.5.2。
- `v0.5.2` 注释标签指向合并提交 `c41a88a`。
- 功能分支：`feature/v052-motion-consistency`，实现提交 `8c40594`。
- v0.5.1 补充矩阵分支：`test/v051-perception-matrix`，提交 `fbcc44b`。

## v0.5.2 已完成

- 四帧速度、方向、加速度运动一致性门控。
- 无鸟静态碎片伪检压制：完整任务 WARNING/CRITICAL 为 0。
- 集体避障优先横向，横向全部不可行时才使用垂直候选。
- 修复空域无目标时自行上偏；无鸟与单鸟回归的最大 Z 避障偏移均为 0。
- 避障恢复到 IDLE 后清除过期 planner failure。
- 102 项 Python 测试、catkin 构建和 ROS launch 展开通过。

## 完整回归证据

- 无鸟：`~/.ros/logistics_runs/mission_20260901_180105.csv`，303.50 秒 COMPLETE，SAFE 1677，启动 STALE 3，XYZ 最大偏移均为 0，最小机间距 3.224 m。
- 单鸟：`~/.ros/logistics_runs/mission_20260901_180700.csv`，298.91 秒 COMPLETE，WARNING 70、CRITICAL 36，最大偏移 1.681/1.895/0 m，最小机间距 3.137 m。
- 两轮三机最终均解除武装，模式 AUTO.LAND。

## 安全语义与边界

- 新鲜空检测必须 SAFE；超过 1 秒无消息才是 STALE。
- 不允许降低机间距、动态净空或超时门槛掩盖感知问题。
- `lidar_visualization:=false` 当前会同时关闭检测所依赖的点云聚合器，不应用它关闭物理感知运行时的可视化开销。
- 分布式 MPC 仍为 shadow；ORCA limited 才允许受限接管。

## 下一步

1. 新对话先核对 `main`、工作区和 `v0.5.2` 标签，确认 GitHub 已同步。
2. 后续版本再做遮挡后的轨迹维持和检测置信度标定；不要重做运动门控。
3. 可拆分 `lidar_visualization` 与点云聚合器启停参数，消除当前启动参数耦合。
4. 不要直接扩大到电量返航或新世界地图。
