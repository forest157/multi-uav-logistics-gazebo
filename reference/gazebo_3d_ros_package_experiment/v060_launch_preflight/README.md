# v0.6.0 启动预检阶段（开发中，未发布）

稳定版仍是 `v0.5.12`。当前阶段只为既有三机启动增加失败关闭的预检，不声明五机或八机能力。`three_uav_mission.launch` 以前允许命令行把 `vehicle_count` 改成 5/8，但 `three_uav_sitl.launch` 实际只有 `uav0`～`uav2`；现已在 roslaunch 解析阶段拒绝该配置，避免任务侧与飞控侧数量不一致。

`preflight_three_uav_sitl` 检查：

- 车辆数必须是 3；自建室外世界通过已有 SDF/布局/元数据一致性验证，且至少有三个出生位。导入的 Baylands 没有飞行剖面，会拒绝。
- 当前 CPU 配额/亲和核数至少 4 核，可用内存至少 10240 MiB（含 cgroup v2 限额）；这是启动保守门槛，不是性能测量。
- Gazebo master、三套 MAVROS/PX4 的 13 个默认 TCP/UDP 端口当前可绑定且互不冲突。检查后立即释放，不能防止随后被其它程序占用。

```bash
source /home/devuser/catkin_ws/devel/setup.bash
/home/devuser/catkin_ws/src/logistics_gazebo_sim/scripts/preflight_three_uav_sitl --world outdoor_campus
```

预检从选中的 launch 文件读取 `gazebo_master_uri`：通用三机任务默认 `11450`，三个室外 lidar 试验入口默认 `11470`。前一阶段固定检查 `11450` 会漏掉实际室外入口端口；现已修正并加回归测试。可用 `launch_outdoor_checked --world outdoor_campus --dry-run` 只检查、不启动；去掉 `--dry-run` 才会执行相应的室外 lidar 试验 launch。该入口不接受任意 launch 参数或 Baylands 世界，且沿用原启动文件的 `auto_start=false`，不会自动起飞。

只针对当前固定的三机端口矩阵和本地 `gazebo_master_uri`；修改 PX4/MAVROS 端口后要同步扩展预检。当前 `scene_0` 只检查旧世界文件存在与固定三机容量，自建室外世界才执行几何一致性验证。直接调用原始 `roslaunch` 仍可绕过资源/端口检查；推荐使用检查后启动入口。下一阶段要把 PX4 实例、端口、命名空间和出生位置统一参数化，再按一机、三机、五机、八机逐级仿真验证。

2026-10-05 本机验证：园区、住宅、街区的预检均 `pass: true`，各识别 3 个出生位并检查 13 个端口；`--vehicle-count 5` 返回非零，`roslaunch --nodes ... vehicle_count:=5` 在解析阶段返回非零，`vehicle_count:=3` 正常列出 3 个 MAVROS 节点。修正端口后，三个室外 `launch_outdoor_checked --dry-run` 均通过并指向 `11470` 与对应 lidar launch；旧 `scene_0` 正确指向 `11450`。262 项 Python 回归测试及四包 catkin 构建通过。尚未为本分支重新运行完整 Gazebo 飞行，不能把本阶段标为五至八机验收。
