# AWS 小型仓库导入记录

2026-09-08，v0.5.11 功能分支。此记录只确认资产下载、固定版本与 Gazebo Classic 运行时加载；三机任务和规划占据模型尚未适配。

- 来源：https://github.com/aws-robotics/aws-robomaker-small-warehouse-world
- ROS1 提交：`3c23a698bf0b4e366ddf8b084af507c519bd3483`
- 归档 SHA-256：`213417f3b243cc6259a17b1eeafba28edd44f7d129ec5904f88793f1bc75d0e6`
- 许可证：MIT-0；版权归 Amazon.com, Inc. 或其关联公司，完整 LICENSE 保留在下载目录中。
- 世界：`worlds/no_roof_small_warehouse.world`，原文件和模型未修改。
- 下载量约 9.7 MiB；资产不纳入 Git，使用缓存目录。

## 复现

在容器中执行：

```bash
bash /home/devuser/catkin_ws/src/logistics_gazebo_sim/scripts/fetch_warehouse_assets
```

脚本先验证固定归档摘要，再解压到新目录，输出 ASSET_ROOT 和启动命令。保留旧缓存，不覆盖已有下载。脚本源码可直接运行，无需 catkin 安装。

加载 ROS 集成时先 source `/opt/ros/noetic/setup.bash` 与 `/home/devuser/catkin_ws/devel/setup.bash`，将输出目录的 models 加入 GAZEBO_MODEL_PATH，然后以 gazebo_ros/empty_world.launch 的 world_name 参数指向该世界。

## 运行验证

Gazebo Classic 加载后 `/gazebo/get_world_properties` 返回 success=true、仿真时间 6.824 秒及 25 个模型实例，包括仓库墙体、地面、货架和杂物。存在上游旧 SDF version 属性及 Collada COLOR 兼容提示。测试结束已正常关闭。

这不是碰撞净空或三机飞行验收。下一步需从模型几何建立规划占据体、确认米制包围盒与可容纳的三机出生/投递区域，并完成传感器、实时率和资源占用回归。原有七场景的编号和规划几何不能直接套用仓库世界。
