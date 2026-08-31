# ROS 1 Noetic → ROS 2 Humble 迁移状态

验收日期：2026-08-31。ROS 1 源目录在迁移期间保持只读；所有修改均位于
`src-ros2`。

## 1. 迁移结论

- 已完成并可构建的 ROS 2 package：18 个。
- 尚未完成的 ROS 1 package：无。
- ROS 1 `livox_ros_driver` 没有自行移植；按要求由官方最新
  `livox_ros_driver2` + 固定版本 SDK2 替代。
- `script/` 中原本散落的 GUI/拓扑脚本已组成新的 `anav_ranger` ROS 2 package。
- 源码树中保留少量未编译的 ROS 1 源文件、XML launch 和旧式 YAML 作为算法对照；
  它们不进入 ROS 2 安装空间，也不是运行入口。

完整干净构建通过，`rosdep check` 通过，18/18 package 可被 `ros2 pkg` 发现。
完整测试结果为 `116 tests, 0 errors, 0 failures, 5 skipped`。工作区 42 个
`*.launch.py` 与官方 Livox 的 9 个 Python launch 均能解析。

## 2. 已迁移 package

| package | ROS 2 迁移内容 | 验证重点 |
|---|---|---|
| `ugv_sdk` | 独立 CMake/catkin 分支改为 `ament_cmake` 注册，保留 SDK 协议实现 | SDK 库、ament discovery、`ranger_base` 链接 |
| `ranger_msgs` | msg/srv/action 由 rosidl 生成 | 8 msg、1 srv、1 action 可发现 |
| `ranger_base` | rclcpp、参数、TF2、底盘 service/action、Python launch | 4 种车型 launch、`odom/base_link` |
| `ranger_bringup` | ament launch 编排 | 底盘 + 仲裁 + 碰撞监控依赖解析 |
| `cmd_vel_arbiter` | rclcpp、多源 Twist 仲裁、新 ROS 2 msg/srv、action client | 优先级、超时、限幅、任务结束停车 |
| `collision_monitor` | rclcpp、TF2、ROS 2 参数和代价地图 topic | 最终 `/cmd_vel`、预测停车、安全状态 |
| `livox_ros_driver2` | 官方 ROS 2 driver 1.2.7、SDK2 vendoring、rosidl | 官方 9 个 launch、CustomMsg、SDK 链接 |
| `livx2pcl` | CustomMsg/PointCloud2/LaserScan ROS 2 API | `/livox/lidar` 转换与 QoS |
| `obstacle_detection` | rclcpp/PCL publisher/subscriber 和参数 | 点云分类输出 |
| `lio_lite` | rclcpp、TF2、rosidl、ROS 2 YAML/Python launch | 建图/定位目标、静态地图过滤测试 |
| `pcd2pgm` | rclcpp、ament、Python launch | PCD 到 OccupancyGrid/PGM |
| `myglobal_planner` | `nav2_core::GlobalPlanner` plugin | Nav2 实际加载、规划 service/topic |
| `jgl_dwa_local_planner` | `nav2_core::Controller` 和两个 costmap layer plugin | B-spline/DWA 测试、Nav2 实际加载 |
| `mxb_move_base` | Nav2 servers + ROS 1 行为兼容节点/BT | action、规划/清图兼容服务、关闭竞态 |
| `x2bot_teleop` | rclcpp、rclcpp_action、msg/srv、TF2、launch | 固定路线、Tag、手柄、任务取消 |
| `robot_r` | ROS 2 launch、RViz2 panel/tool plugin、串口/TCP 节点 | plugin 注册、TF、启动编排 |
| `anav_ranger` | rclpy、PyQt5 GUI、拓扑/点位工具 package 化 | GUI 无头启动、服务和 TF 接入 |
| `nav_benchmark` | rclpy 导航运行记录与离线汇总 | 订阅、参数、结果文件和干净退出 |

## 3. 主要 API 替换

| ROS 1 | ROS 2 Humble | 行为处理 |
|---|---|---|
| catkin / `catkin_package` | `ament_cmake`、`ament_cmake_python`、rosidl | 全部活动 package 使用 ament build type |
| `roscpp` / `rospy` | `rclcpp` / `rclpy` | callback、timer、QoS 和 shutdown 按 ROS 2 生命周期改写 |
| `message_generation` | `rosidl_default_generators` | msg/srv/action 均由 ROS 2 interface 生成 |
| actionlib `MoveBaseAction` | `nav2_msgs/action/NavigateToPose` | AutoNAV 和兼容节点转发到 `/navigate_to_pose` |
| move_base 单体 | Nav2 planner/controller/BT/lifecycle servers | 保留旧 topic/service 行为兼容端点 |
| `nav_core::BaseGlobalPlanner` | `nav2_core::GlobalPlanner` | 原全局规划算法包为 plugin，不主动重构算法 |
| `nav_core::BaseLocalPlanner` | `nav2_core::Controller` | DWA/B-spline 逻辑保留，适配 Nav2 生命周期接口 |
| `costmap_2d::Layer` | `nav2_costmap_2d::Layer` | decay/blind-clear layer 作为 ROS 2 plugin 导出 |
| `tf` / `tf::TransformListener` | TF2 buffer/listener/broadcaster | frame 名和静态外参保持不变 |
| ROS parameter server | node-scoped declare/get/set parameters | YAML 全部使用 `ros__parameters` |
| dynamic_reconfigure | ROS 2 parameters/参数回调 | ROS 2 无直接等价包，采用节点参数的最小等价方案 |
| XML roslaunch | Python launch / 少量 ROS 2 XML include | package share 路径由 ament index 解析 |
| ROS 1 pluginlib export | `pluginlib_export_plugin_description_file` | Nav2/RViz plugin 均注册到 ament index |
| rosbag | rosbag2 | benchmark 和工具依赖改为 rosbag2 系列 |

### 无法一对一映射的处理

ROS 2 没有 ROS 1 `move_base` 单体及 `move_base_msgs/MoveBaseAction` 的完全等价物。
因此采用 Nav2 标准 action 和生命周期节点，同时由 `mxb_move_base` 提供
`/move_base_simple/goal`、`/mxb_move_base/make_plan` 和
`/mxb_move_base/clear_costmaps` 等兼容端点。固定路线通过独立 BT XML 选择，未删除
原拓扑和 B-spline 算法。

ROS 1 driver1 也没有作为本工程维护的 Humble 等价版本；使用官方 driver2 是最小
且可维护的替代，业务 topic 和 LIO CustomMsg 接口保持兼容。

## 4. 自定义接口

已由 `ros2 interface list` 验证：

- `ranger_msgs`：`ActuatorState`、`ActuatorStateArray`、`DriverState`、
  `MotionState`、`MotorState`、`RangerLightCmd`、`RsStatus`、`SystemState`；
  `TriggerParkMode`；`StopAndCenter` action。
- `cmd_vel_arbiter`：`ArbitratedCommand`；`FinishMotion` service。
- `x2bot_teleop`：`NavConfig`、`SetInt`、`SetTagY` services。
- `lio_lite`：`Pose6D`。
- `livox_ros_driver2`：`CustomMsg`、`CustomPoint`。

## 5. 运行 API 审计

### 导航

Nav2 冒烟启动时，planner/controller/costmap plugin 全部实际加载：

- `myglobal_planner/MyGlobalPlanner`
- `jgl_dwa_local_planner/DWAPlannerROS`
- `jgl_dwa_local_planner/DecayObstacleLayer`
- `jgl_dwa_local_planner/BlindClearLayer`

生命周期节点在临时 TF 下均到达 `active`。发现的标准 action：

- `/compute_path_to_pose`
- `/compute_path_through_poses`
- `/follow_path`
- `/navigate_to_pose`
- `/navigate_through_poses`

兼容服务：

- `/mxb_move_base/make_plan`：`nav_msgs/srv/GetPlan`
- `/mxb_move_base/clear_costmaps`：`std_srvs/srv/Empty`
- `/planner_server/GridBased/make_plan`：`nav_msgs/srv/GetPlan`

### AutoNAV 和 Tag

- `/anav/nav_config`：`x2bot_teleop/srv/NavConfig`
- `/anav/cancel_navigation`：`std_srvs/srv/Trigger`
- `/anav/reload_topology`：`std_srvs/srv/Trigger`
- `/plan_path_and_go`：`x2bot_teleop/srv/SetInt`
- `/set_target_y`：`x2bot_teleop/srv/SetTagY`
- `/anav/task_status`、`/diagnostics`、`/anav/topology_safety_phase`

### 速度安全链

运行时已验证以下接口存在且参数实载：

```text
/cmd_vel/safety   priority 100 --+
/cmd_vel/teleop   priority  80 --+
/cmd_vel/tag      priority  60 --+--> /cmd_vel/candidate
/cmd_vel/nav      priority  20 --+          |
                                             v
                                      collision_monitor --> /cmd_vel
```

- `/cmd_vel/candidate`：`cmd_vel_arbiter/msg/ArbitratedCommand`
- `/cmd_vel_arbiter/finish_motion`：`cmd_vel_arbiter/srv/FinishMotion`
- `/stop_and_center`：`ranger_msgs/action/StopAndCenter`
- `/collision_monitor/markers`：`visualization_msgs/msg/MarkerArray`
- `/diagnostics`：`diagnostic_msgs/msg/DiagnosticArray`

## 6. TF 和参数兼容

静态 TF 与 ROS 1 数值完全相同：

```text
map -> body -> base_link -> livox_frame
             ^
odom --------+  # ranger_base 的 publish_odom_tf 可选
```

- `body -> base_link`：x = `-0.33 m`
- `base_link -> livox_frame`：x = `+0.33 m`
- 全局规划/碰撞监控：`map`
- 机器人基座：`base_link`
- 底盘里程计：`odom -> base_link`

TF2 运行查询已返回上述两个静态变换。安装空间中的 12 份参数 YAML 均通过 YAML
解析和 `ros__parameters` 结构检查；关键节点运行时确认读取到
`output_topic=/cmd_vel/candidate`、`output_topic=/cmd_vel`、
`global_frame=map`、`base_frame=base_link`。

## 7. 新增依赖

系统层实际新增/确认：

- `libaprutil1-dev`
- `python3-serial`
- `python3-pyqt5`、`python3-pyqt5.qtsvg`
- `pyqt5-dev`、`python3-sip-dev`、`sip-dev`
- `gnome-terminal`
- `iproute2`

ROS 2 侧主要新增 Nav2、`rclcpp_action`、TF2、pluginlib、RViz2、joy、rosbag2、
rosidl 和 ament lint/test 依赖。PCL、Eigen、Boost、ASIO、gflags、glog、TBB 和
yaml-cpp 由各 package manifest/rosdep 管理。详细新机命令见
[ROS2_HUMBLE_INSTALL.md](ROS2_HUMBLE_INSTALL.md)。

## 8. 仍需真机验证的风险

以下项目无法由容器编译和无硬件冒烟消除：

1. Ranger CAN 时序、车型切换、停车回正和物理急停联动。
2. 实车速度方向、Ackermann 曲率、B-spline 跟踪误差、终点姿态和轮组回正。
3. 碰撞监控 footprint、盲区、制动加速度、地图未知区策略和最坏制动距离。
4. MID360s/MID360 UDP 网络、时间同步、IMU 外参、丢包率和 QoS。
5. LIO 的 `map -> body` 漂移、重定位、动态点过滤和大地图内存/实时性。
6. `map -> body -> base_link -> livox_frame` 与实物安装尺寸的一致性。
7. `~/maps` 中 PCD、PGM/YAML、点位和 topology 指纹的现场一致性。
8. MM3V `/dev/ttyUSB0` 权限、Tag 坐标符号/偏置、UDP 地址和精调停止距离。
9. PyQt5/RViz2 在真实桌面、显卡和多终端环境下的交互；RViz plugin 点击行为。
10. 可选 MAVROS/PX4 入口（当前环境未安装，且默认关闭）。
11. 导航 benchmark 的长时间运行、磁盘空间和 rosbag2 数据完整性。

在这些项目完成前，只能认定“软件迁移与容器级验证完成”，不能认定实车安全验收
完成。
