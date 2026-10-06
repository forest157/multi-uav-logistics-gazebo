# v0.6.0 开发与安全记录

以下首节保留 2026-10-05 启动预检阶段的过程记录；最终 0.6.0 结果见[发布说明](RELEASE.md)。当时只为既有三机启动增加失败关闭的预检，不声明五机或八机能力。`three_uav_mission.launch` 以前允许命令行把 `vehicle_count` 改成 5/8，但 `three_uav_sitl.launch` 实际只有 `uav0`～`uav2`；现已在 roslaunch 解析阶段拒绝该配置，避免任务侧与飞控侧数量不一致。

`preflight_three_uav_sitl` 检查：

- 车辆数必须是 3；自建室外世界通过已有 SDF/布局/元数据一致性验证，且至少有三个出生位。导入的 Baylands 没有飞行剖面，会拒绝。
- 当前 CPU 配额/亲和核数至少 4 核，可用内存至少 10240 MiB（含 cgroup v2 限额）；这是启动保守门槛，不是性能测量。
- Gazebo master、三套 MAVROS/PX4 的 13 个默认 TCP/UDP 端口当前可绑定且互不冲突。检查后立即释放，不能防止随后被其它程序占用。

```bash
source /home/devuser/catkin_ws/devel/setup.bash
/home/devuser/catkin_ws/src/logistics_gazebo_sim/scripts/preflight_three_uav_sitl --world outdoor_campus
```

预检从选中的 launch 文件读取 `gazebo_master_uri`：通用三机任务默认 `11450`，三个室外 lidar 试验入口默认 `11470`。前一阶段固定检查 `11450` 会漏掉实际室外入口端口；现已修正并加回归测试。可用 `launch_outdoor_checked --world outdoor_campus --dry-run` 只检查、不启动；去掉 `--dry-run` 才会执行相应的室外 lidar 试验 launch。该入口不接受任意 launch 参数或 Baylands 世界，且沿用原启动文件的 `auto_start=false`，不会自动起飞。

只针对当前固定的三机端口矩阵和本地 `gazebo_master_uri`；修改 PX4/MAVROS 端口后要同步扩展预检。当前 `scene_0` 只检查旧世界文件存在与固定三机容量，自建室外世界才执行几何一致性验证。直接调用原始 `roslaunch` 仍可绕过资源/端口检查；推荐使用检查后启动入口。现已抽取 `sitl_instance.launch`，由实例 ID 推导命名空间、MAVROS 系统号、PX4/Gazebo 端口，三机父入口仍按原位置传入 `0/1/2`。该可复用单实例入口不等于已实现五至八机任务；还需扩建世界出生位和任务容量，再按单机、三机、五机、八机逐级验证。

2026-10-05 本机验证：园区、住宅、街区的预检均 `pass: true`，各识别 3 个出生位并检查 13 个端口；`--vehicle-count 5` 返回非零，`roslaunch --nodes ... vehicle_count:=5` 在解析阶段返回非零，`vehicle_count:=3` 正常列出 3 个 MAVROS 节点。修正端口后，三个室外 `launch_outdoor_checked --dry-run` 均通过并指向 `11470` 与对应 lidar launch；旧 `scene_0` 正确指向 `11450`。单实例 ID `7` 静态解析为 `/uav7`、MAVROS `14547/14587`、Gazebo `14567/4567` 和系统号 `8`；ID `8` 被解析门拒绝。三机展开后的端口与已发布版一致。园区 headless 无自动起飞烟测中 `iris0/1/2` 均进入 Gazebo，三套 MAVROS 均连接且解除武装，随后已关闭进程。关闭日志出现 ROS shutdown 异常和一次 `Segmentation fault (core dumped)`，也有 PX4 拒绝若干降落参数；尚未定位是否为既有环境问题，不将此次烟测当作完整飞行验收。262 项 Python 回归测试及四包 catkin 构建通过。尚未为本分支重新运行完整 Gazebo 飞行，不能把本阶段标为五至八机验收。

## 规模场景与分组通行（开发中）

新增 `outdoor_scale_yard` 真实尺度室外场景：八个相隔 12 m 的起飞位、对应终点位，两栋带碰撞体的长建筑构成约 12 m 的狭廊。18 m 巡航高度低于 22 m 屋顶，不能靠爬高或把整个编队压进通道。世界文件、布局和 JSON 元数据三方一致性验证后，按每机 1.2 m 水平包络、0.6 m 垂直包络和 0.5 m 采样步长计算容量为三机。`phase_plan` 将 1/3/5/8 架分别分成 1、1、2、3 组，按实际到位状态依次完成入口集结、横向收拢、穿廊、出口展开及反向返航；任何一组未清空通道，下一组不能进入。全部路线经静态几何和同步运动下的机间距抽样检查。20/35/50 架的模式只做轻量运动学排程压力测试，不加载 PX4 或 Gazebo。

`fleet_scale_sitl.launch` 根据实例 ID 推导 MAVROS/PX4 系统号、命名空间、UDP/TCP/视频端口和出生位置，并选用轻量 `iris`；原三机 lidar 启动路径保持不变。推荐先执行 `preflight_scale_sitl --count 8` 或 `launch_scale_checked --count 8 --dry-run`；检查 CPU、可用内存、世界八个出生位与相应端口。实际启动命令：

```bash
source /opt/ros/noetic/setup.bash
source /home/devuser/catkin_ws/devel/setup.bash
cd /home/devuser/catkin_ws/src/logistics_gazebo_sim
scripts/launch_scale_checked --count 3 --gui false
# 独立终端，在起飞前先启动 Gazebo 真值审计：
scripts/audit_scale_flight --count 3 --output /tmp/v060_flight_3.json
# 确认状态 ready、实体和起飞区域安全后，才由操作员显式启动：
rosservice call /scale_group_controller/start
rosservice call /scale_group_controller/status
```

启动入口不会自动解锁。显式 `start` 在解锁前逐机设置并读回 `COM_RCL_EXCEPT` 的 Offboard 位（4），失败则拒绝启动。首次五机试验揭示旧三机 EEPROM 带有该位而新实例没有，导致后两架进入 `AUTO.RTL`；修复不能依赖持久参数。控制器在遥测缺失/超过 2 s、MAVROS 断连、机间实测距离小于 2.7 m、进入建筑包络、意外退出已解锁的 `OFFBOARD` 或阶段超时后停止推进并保持位置；故障不会自动重试或在狭廊里盲目降落。此模式是无雷达、无动态障碍的规模验证，不替代已发布三机 lidar 避障任务；实飞结果和版本结论以逐级真值报告为准。

2026-10-06 最终验收：1/3/5/8 架完整往返分别通过 12/12、12/12、22/22、32/32 阶段独立 Gazebo 真值审计，最小机间距（后三组）3.182/2.869/3.102 m，最小静态包络净空分别 4.626/1.014/0.984/1.001 m，零危险事件且全部解除武装。八机启动预检核查 73 个端口，20/35/50 架轻量排程压测通过，273 项 Python 回归及七包 catkin 构建无警告。结构化结果见 [`V060_SCALE_ACCEPTANCE_OCT06.json`](V060_SCALE_ACCEPTANCE_OCT06.json)。
