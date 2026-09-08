# 室外场景

在原 outdoor_logistics.world 基础上新增三个自建场景，所有坐标为 ENU 米制。

| 世界 | 布局范围 | 建筑 | 高度 |
|---|---|---|---|
| outdoor_campus.world | 180×160 m | 6 栋园区楼宇 | 10–16 m |
| outdoor_residential.world | 200×180 m | 12 栋住宅 | 6–12 m |
| outdoor_urban.world | 240×220 m | 12 栋街区建筑 | 14–30 m |

每个世界旁的同名 JSON 保存建筑包围盒、道路、三机候选出生点与配送点。范围表示布局范围，地面仍为 Gazebo ground_plane。道路为视觉层，建筑的视觉与碰撞几何相同。候选起降点按 3.5 m 水平余量检查建筑净空，尚未接入任务规划或完成飞行验收。

生成：`python3 scripts/generate_worlds worlds`（先 source catkin 工作区）。

查看：`roslaunch gazebo_ros empty_world.launch gui:=true world_name:=/home/devuser/catkin_ws/src/logistics_gazebo_sim/worlds/outdoor_campus.world`

原室外场景标记已修复旧参考坐标转换错误，起终点现在对应配置中的 (-18,-4) 和 (0,42) 米。新场景是本项目程序化自建资产，不能计为第三方室外世界导入。

验证：150 项 unittest 和 catkin 构建通过；三个新增世界逐一无 GUI 加载，get_world_properties 均 success=true，仿真时间均推进超过 5 秒。探测由 timeout 正常收尾，未启动无人机任务。
