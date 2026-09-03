# 新对话交接：v0.5.10 已发布，继续 v0.5.11

仓库位于 Docker 容器 `gazebo-dev` 的 `/home/devuser`。开始前核对容器、`main`、工作区、SSH 和远端；重启后确认 GitHub SSH 仍经 `ssh.github.com:443` 使用 `~/.ssh/id_ed25519_multi_uav_github`。

v0.5.10 已完成能量风险分级、时效与滞回、返航槽位安全建议、能量优先错峰降落，以及单架 CRITICAL 的受保护备用降落。多架临界或无安全落点时不会盲目脱队。安全联锁和动态 HOLD 的优先级高于能量控制。

验收：138 项测试、catkin 无警告、两组不开桨 ROS 试验，以及三机完整 Gazebo 回归。完整任务 `COMPLETE`、全部解除武装、最小机间距 3.210 m、安全 ERROR/HOLD 为 0。

下一步按 `ROADMAP.md` 开发 v0.5.11：调研许可清晰且兼容 Gazebo Classic/PX4/ROS Noetic 的真实尺度世界与模型；统一米制尺度、坐标、碰撞体和性能预算；先导入静态资产并进行净空与出生点校验，再跑现有规划、安全和完整任务回归。不要重做 v0.5.0～v0.5.10。
