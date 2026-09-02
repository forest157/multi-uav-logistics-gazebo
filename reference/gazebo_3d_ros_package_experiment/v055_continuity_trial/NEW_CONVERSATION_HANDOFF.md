# 新对话交接：multi-uav-logistics-gazebo v0.5.5

更新时间：2026-09-02

## 新对话首条提示词

请读取 `reference/gazebo_3d_ros_package_experiment/v055_continuity_trial/NEW_CONVERSATION_HANDOFF.md`，进入 Docker 容器 `gazebo-dev`，在 `/home/devuser` Git 仓库继续开发。电脑重启后先核对容器、分支、工作区、SSH 解析与实际认证，再核对 `main` 和 `v0.5.5` 标签；不要重做已发布版本。

## 环境与 SSH

- Git 根目录：`/home/devuser`
- GitHub：`git@github.com:forest157/multi-uav-logistics-gazebo.git`
- SSH 必须解析到 `ssh.github.com:443`，密钥 `~/.ssh/id_ed25519_multi_uav_github`。
- 重启后运行 `ssh -G github.com | grep -E "^(hostname|port|identityfile) "`，并用 `ssh -T git@github.com` 实际认证。

## 版本状态

- `main` 已发布 v0.5.5。
- v0.5.5 增加端到端 ROS 遮挡连续性验收与按可见周期统计的 ID 连续率。
- v0.5.4 的上游预测重关联、1.5 秒遮挡维持、CSV 轨迹指标继续保留。
- v0.5.2 的空域自发上偏修复继续保留。

## 验证

- 112 项 Python 测试全部通过，catkin 构建无警告。
- 自动 ROS 验收：0.6 秒遮挡、3 个遮挡样本，重现后 ID 保持 `lidar_target_0`。
- v0.5.4 完整飞行 CSV：12 个可见周期、12 个连续周期、周期内 ID 跳变 0、ID 连续率 1.0。
- 完整飞行原始记录：`~/.ros/logistics_runs/mission_20260902_133005.csv`。

## 下一步

1. 收集多轮不同目标速度、遮挡时长和传感噪声的 CSV。
2. 生成置信度与重捕获成功率的可靠性曲线。
3. 在数据充分后，再决定是否以及如何让轨迹置信度参与风险门控。
4. 不要直接扩大到电量返航或新世界地图。
