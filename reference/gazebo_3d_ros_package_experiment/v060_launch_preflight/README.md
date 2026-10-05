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

只针对默认端口矩阵和本地 `gazebo_master_uri`；修改启动端口后要同步扩展预检。当前 `scene_0` 只检查旧世界文件存在与固定三机容量，自建室外世界才执行几何一致性验证。预检是独立 CLI，直接调用原始 `roslaunch` 可以绕过资源/端口检查；真正启动前应先运行它。下一阶段要把 PX4 实例、端口、命名空间和出生位置统一参数化，建立不可绕过的安全入口，再按一机、三机、五机、八机逐级仿真验证。

2026-10-05 本机验证：园区、住宅、街区的预检均 `pass: true`，各识别 3 个出生位并检查 13 个端口；`--vehicle-count 5` 返回非零，`roslaunch --nodes ... vehicle_count:=5` 在解析阶段返回非零，`vehicle_count:=3` 正常列出 3 个 MAVROS 节点。260 项 Python 回归测试及四包 catkin 构建通过。尚未为本分支重新运行完整 Gazebo 飞行，不能把本阶段标为五至八机验收。
