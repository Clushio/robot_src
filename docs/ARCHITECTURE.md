# 系统架构

本文描述当前 Ranger AutoNAV 的运行关系。它解释“哪个节点拥有哪项职责”，不替代
各包源码或参数文件。

## 分层

| 层 | 主要组件 | 职责 |
|---|---|---|
| 操作层 | `script/aNAV_ranger.py`、RViz | 启停进程、点位编辑、目标下发、状态显示、软件急停 |
| 任务层 | `x2bot_teleop/runnav` | 读取点位/拓扑、选路、阻塞边管理、循环任务支撑 |
| 规划层 | `mxb_move_base`、`myglobal_planner`、`jgl_dwa_local_planner` | 全局规划、B 样条参考路径、局部跟踪、代价地图 |
| 精调层 | `mm3v_serial_reader.py`、`TagCtl` | 读取 Tag，相对 X/Y/Yaw 工位精调 |
| 定位层 | Livox 驱动、`lio_lite` | 点云/IMU、建图、重定位和 `/Odometry` |
| 速度安全层 | `cmd_vel_arbiter`、`collision_monitor` | 来源仲裁、限速、运动预测、减速和停车 |
| 执行层 | `ranger_base_node`、`ugv_sdk` | CAN 通信、底盘状态、轮组模式和回正 |

## 启动所有权

- `ranger_bringup/ranger_mini_v2.launch` 启动底盘。
- 底盘的 include launch 同时启动 `cmd_vel_arbiter` 和 `collision_monitor`。
- `robot_r/3startlocation.launch` 启动 MID360s、地图服务器、LIO 定位和可选 RViz。
- `robot_r/5nav.launch` 启动定制 MoveBase 并加载规划/代价地图参数。
- `robot_r/3navlocations.launch` 启动 `runnav` 拓扑任务服务。
- GUI 只是编排和交互入口，不替代上述 ROS 节点的职责。

详细 launch 对照见 [`robot_r/README.md`](../robot_r/README.md)。

## 定位与地图数据流

```text
/livox/lidar + /livox/imu
            │
            v
       LIO-Lite (laserMapping)
            ├── /Odometry          全局定位位姿，GUI 和导航使用
            ├── /cloud_registered  配准点云
            ├── GlobalMap.pcd      三维静态地图
            ├── FeatureMap.pcd     定位特征地图
            └── split_map/         分块加载数据

GlobalMap.pcd ──> pcd2pgm ──> /map ──> map_saver ──> map.pgm + map.yaml
```

Ranger 底盘另行发布 `/odom`，它表示底盘测得的速度/里程计；GUI 用 `/Odometry`
显示全局位置，用 `/odom` 和 `/motion_state` 显示实测速度及轮组姿态。两者不要混用。

## 拓扑任务数据流

```text
map.yaml + map.pgm + robot_positions.txt
                    │
                    v
           build_topology.py
                    │
             topology.yaml
                    │
                    v
 /plan_path_and_go ─> runnav ─> Dijkstra 拓扑路径 ─> move_base action
                                         │
                                         ├── /topology_plan
                                         ├── /topology_markers
                                         └── /anav/task_status
```

点位文件中的行号只是稳定节点 ID，不代表默认路线顺序。拓扑构建器会根据二维地图、
机器人尺寸、padding、最小净空和节点度数生成安全边，并将输入指纹写入
`topology.yaml`。`runnav` 会验证指纹、节点和边；验证失败时拒绝任务。

`/plan_path_and_go` 的 `run` 字段：

| 值 | 行为 |
|---|---|
| `0` | 只规划和发布拓扑路径，不执行 |
| `1` | 自动绕路；边堵塞后可暂时封锁并重新规划 |
| `2` 或更大 | 固定路线；堵塞时停车等待，不换路 |

## B 样条与 MoveBase

`myglobal_planner` 生成栅格全局路径，定制 `jgl_dwa_local_planner` 从路径构造参考
曲线。当前默认 `reference_curve_type: bspline`，优化目标包括：

- 曲线平滑度；
- 与障碍物的安全距离；
- 对拓扑/全局路径的偏离；
- 最大曲率和最小转弯半径；
- 局部失败段的回退与重新生成。

跟踪器还对曲率进行低通、死区和变化率限制，以减少双阿克曼轮组抖动。主要参数
在 `nav/jgl_dwa_local_planner/param/dwa_local_planner_params.yaml`。

## 速度与安全链路

```text
/cmd_vel/safety  (100，软件急停)
/cmd_vel/teleop  ( 80，人工遥控)
/cmd_vel/tag     ( 60，Tag 精调)
/cmd_vel/nav     ( 20，导航)
         │
         v
 cmd_vel_arbiter -- 来源超时、速度上限、切换零速度周期
         │
 /cmd_vel/candidate  (ArbitratedCommand，包含来源名和 Twist)
         │
         v
 collision_monitor -- footprint + 地图 + 实测运动 + 制动预测
         │
         v
      /cmd_vel
         │
         v
 ranger_base_node
```

仲裁优先级和上限来自
`cmd_vel_arbiter/config/cmd_vel_arbiter.yaml`。碰撞监控读取：

- `/cmd_vel/candidate`：带来源的候选速度；
- `/odom`：实测底盘运动；
- `/map`：静态占据地图；
- `/mxb_move_base/local_costmap/costmap`：局部障碍地图；
- `map -> base_link` TF；
- `collision_monitor/config/robot_footprint.yaml`：物理 footprint。

监控可能输出原速、75%/50%/25% 减速或零速度，并通过
`/collision_monitor/status` 给出原因。地图、TF、odom 或命令超时等关键数据不可信
时采取失败即停车。完整状态机见
[`collision_monitor/README.md`](../collision_monitor/README.md)。

遥控来源属于人工操作语义，可能显示 `MANUAL_BYPASS`，此时没有 footprint 预测
保护。软件急停也依赖 ROS、仲裁器和底盘进程正常工作，因此物理急停始终是最后保障。

## 任务结束与轮组回正

导航、Tag 或取消任务完成后，通过
`/cmd_vel_arbiter/finish_motion` 请求停止。仲裁器暂时抑制旧来源命令，并调用底盘
`/stop_and_center` action。底盘执行停车、等待车轮静止、切换模式和轮组居中。

普通零速度命令不等价于“停车并回正”。调试任务结束问题时，应同时检查：

- `/cmd_vel_arbiter/finish_motion` 服务；
- `/stop_and_center` action；
- `/motion_state`；
- 底盘驱动日志中的车轮速度和转角误差。

## Tag 精调数据流

```text
MM3V 串口 ─> mm3v_serial_reader.py ─> /tag_position
                                           │
/set_target_y ----------------------------> TagCtl
                                           │
                                      /cmd_vel/tag
```

读头节点负责协议解析、坐标方向/偏置、短时丢 Tag 保持、滤波和可选 UDP 回传。
`TagCtl` 按阶段修正角度、X、Y/Yaw，并在成功、失败或超时后请求停车回正。

## 主要接口速查

| 接口 | 类型/方向 | 用途 |
|---|---|---|
| `/Odometry` | `nav_msgs/Odometry` | LIO 全局位姿 |
| `/odom` | `nav_msgs/Odometry` | Ranger 实测速度/里程计 |
| `/motion_state` | `ranger_msgs/MotionState` | 轮组姿态 |
| `/move_base` | `move_base_msgs/MoveBaseAction` | 单段导航执行 |
| `/plan_path_and_go` | `x2bot_teleop/SetInt` | 拓扑规划/执行 |
| `/anav/cancel_navigation` | `std_srvs/Trigger` | 取消当前拓扑任务 |
| `/anav/reload_topology` | `std_srvs/Trigger` | 重新加载拓扑 |
| `/anav/nav_config` | `x2bot_teleop/NavConfig` | 读取/应用 AutoNAV 参数 |
| `/anav/task_status` | `std_msgs/String` | GUI 任务状态 |
| `/set_target_y` | `x2bot_teleop/SetTagY` | Tag 精调目标 |
| `/cmd_vel_arbiter/finish_motion` | `cmd_vel_arbiter/FinishMotion` | 结束速度来源并请求回正 |
| `/stop_and_center` | `ranger_msgs/StopAndCenterAction` | 底盘停车与轮组回正 |
| `/collision_monitor/status` | `diagnostic_msgs/DiagnosticArray` | 安全状态和原因码 |

## 配置所有权

- 物理 footprint：`collision_monitor/config/robot_footprint.yaml`，代价地图和安全门共用。
- 速度来源上限：`cmd_vel_arbiter/config/cmd_vel_arbiter.yaml`。
- 最终运动/制动上限：`collision_monitor/config/collision_monitor.yaml`。
- 路径生成和跟踪：`nav/jgl_dwa_local_planner/param/dwa_local_planner_params.yaml`。
- LIO 和 MID360s：`robot_r/lio/config/mid360.yaml`。
- AutoNAV 运行策略：`~/maps/autonav_params.yaml`。

同一个物理量在多层可能各有上限，最终有效值通常取最保守者。修改时必须沿整条链路核对，不能只看单个 YAML。
