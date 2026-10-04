# Baylands 室外开源世界导入（2026-10-04）

这是 v0.5.11 的第三方世界导入兼容性验收，不是三机航线或动态避障验收。三机完整任务在本项目自建园区、住宅、街区世界单独验证。

- 来源：Open Robotics Fuel 的 [Baylands 模型](https://fuel.gazebosim.org/1.0/OpenRobotics/models/baylands)，元数据版本 3，作者 Cole Biesemeyer；Fuel 元数据标注 [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/)。世界入口使用已安装 PX4 Gazebo Classic 的 `Tools/sitl_gazebo/worlds/baylands.world`，不拷贝第三方网格进 Git。
- 归档下载地址：`https://fuel.gazebosim.org/1.0/OpenRobotics/models/baylands.zip`；SHA-256：`a6576c36823b61566b408a8cee2df97e9c926c3a004f1be6855680ace67f8f9b`。`scripts/fetch_baylands_assets` 下载后先核摘要，再解压到容器缓存；上游若改动“latest”文件会拒绝继续。
- 模型 SDF 1.5，三块 Collada 网格各有视觉与碰撞几何；解析碰撞网格顶点后的总体范围约 X −410.701～371.480 m、Y −394.172～171.839 m，即 782.181×566.011 m。网格顶点范围不是地表可飞空间，也不能用包围盒代替复杂地形占据。
- Gazebo Classic 11 无 GUI 加载后，`/gazebo/get_world_properties` 返回 `success: true`、模型 `baylands_01`；`get_model_properties` 返回三个静态 body 和三个 collision。12 秒空载采样实时率 0.999，`gzserver` CPU 0.109 核、RSS 846.5 MiB。此资源结果不包括 PX4、雷达和任务节点，不能代替全栈预算。

复现：执行 `scripts/fetch_baylands_assets`，将输出的 `GAZEBO_MODEL_PATH` 设为环境变量，然后用 `roslaunch gazebo_ros empty_world.launch world_name:=/home/devuser/PX4_Firmware/Tools/sitl_gazebo/worlds/baylands.world gui:=false` 加载。不要在未建立与复杂地形碰撞网格一致的规划占据、出生点及高度剖面前将三机任务直接切到该世界。原始网格和 ZIP 留在容器缓存，不随仓库提交。
