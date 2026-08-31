# Ranger AutoNAV ROS 2 Humble 工作区

本分支是 Ranger 移动机器人完整的 ROS 2 Humble 工作区，保留了 ROS 1
工程中的 Livox/LIO、建图定位、拓扑任务、B-spline 路径跟踪、Tag 精调、
`cmd_vel` 仲裁、碰撞监控、RViz 面板和 PyQt5 操作界面。

源码目录在宿主机 `/home/nav/lyf/version/src-ros2`，映射到
`nav-humble` 容器的 `/workspace/src`。ROS 1 原工程
`/home/nav/lyf/version/src` 只作为行为对照，不参与 ROS 2 构建。

> 这是实车控制软件。编译和仿真检查不能代替断开动力测试、封闭场地低速测试、
> 物理急停检查以及制动距离验证。软件急停不能替代底盘物理急停。

## 首次使用

先阅读：

1. [ROS 2 Humble 新机安装与构建](docs/ROS2_HUMBLE_INSTALL.md)
2. [ROS 2 迁移状态、API 映射与风险](docs/ROS2_MIGRATION_STATUS.md)
3. [系统架构](docs/ARCHITECTURE.md)
4. [操作手册](docs/OPERATIONS.md)

容器内最短构建流程：

```bash
source /opt/ros/humble/setup.bash
cd /workspace
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
colcon build --symlink-install --executor sequential
source /workspace/install/setup.bash
colcon test --executor sequential
colcon test-result --verbose
```

当前工作区应发现 18 个 package。验证方式：

```bash
colcon list
for package in $(colcon list --names-only); do
  ros2 pkg prefix "$package"
done
```

## 系统数据流

```text
MID360s -> livox_ros_driver2 -> LIO-Lite -> /Odometry、点云、map->body
                                              |
点位 + topology.yaml -> AutoNAV -> Nav2/B-spline -> /cmd_vel/nav
MM3V Tag -------------------------------------> /cmd_vel/tag
手柄 -----------------------------------------> /cmd_vel/teleop
GUI 软件急停 ---------------------------------> /cmd_vel/safety
                                                   |
                                            cmd_vel_arbiter
                                                   |
                                            /cmd_vel/candidate
                                                   |
             /map + /local_costmap/costmap + /odom -> collision_monitor
                                                   |
                                                /cmd_vel
                                                   |
                                             Ranger 底盘驱动
```

安全约束不变：业务节点不得直接发布最终 `/cmd_vel`；仲裁器负责优先级、超时
和来源限幅，碰撞监控是唯一的最终速度出口。

## 真机启动顺序

每个终端先执行：

```bash
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
```

然后按需启动：

```bash
# 1. CAN
sudo ip link set can0 up type can bitrate 500000

# 2. Ranger 底盘、安全仲裁和碰撞监控
ros2 launch ranger_bringup ranger_mini_v2.launch.py

# 3. MID360s、地图、LIO 定位和 RViz
ros2 launch robot_r 3startlocation.launch.py

# 4. Nav2、定制全局/局部规划器和 B-spline 跟踪
ros2 launch robot_r 5nav.launch.py

# 5. 拓扑任务与导航基准记录
ros2 launch robot_r 3navlocations.launch.py
```

PyQt5 GUI 入口：

```bash
ros2 run anav_ranger aNAV_ranger.py
```

## 关键兼容接口

- TF：`map -> body -> base_link -> livox_frame`；底盘可选发布
  `odom -> base_link`。
- 导航 action：`/navigate_to_pose`、`/navigate_through_poses`。
- ROS 1 行为兼容端点：`/move_base_simple/goal`、
  `/mxb_move_base/make_plan`、`/mxb_move_base/clear_costmaps`。
- AutoNAV 服务：`/anav/nav_config`、`/anav/cancel_navigation`、
  `/anav/reload_topology`、`/plan_path_and_go`。
- 底盘安全 action：`/stop_and_center`；任务结束服务：
  `/cmd_vel_arbiter/finish_motion`。

## 运行数据

业务文件默认位于 `~/maps`：

```text
GlobalMap.pcd
FeatureMap.pcd
split_map/
map.pgm
map.yaml
robot_positions.txt
topology.yaml
autonav_params.yaml
nav_benchmark_results/
```

新机器必须按实际网卡、雷达、CAN 和串口修改配置。当前真机配置使用 `can0`
500000 bit/s、Tag `/dev/ttyUSB0` 38400 baud；Livox 地址见
`robot_r/config/MID360_config.json`。

## 主要目录

| 目录 | 作用 |
|---|---|
| `script/` | `anav_ranger`：GUI、拓扑构建和辅助工具 |
| `robot_r/` | 真机 launch、RViz plugin、雷达/LIO 集成 |
| `nav/x2bot_teleop/` | AutoNAV、点位记录、Tag 和手柄控制 |
| `nav/jgl_dwa_local_planner/` | Nav2 Controller、B-spline 和 costmap layer |
| `nav/myglobal_planner/` | Nav2 全局规划器 plugin |
| `nav/mxb_move_base/` | Nav2 启动及 ROS 1 行为兼容节点 |
| `cmd_vel_arbiter/` | 多来源速度仲裁与停车回正 |
| `collision_monitor/` | footprint 预测、减速和最终停车 |
| `slam/LIO-Lite-eskf-base/` | LIO 建图/定位和静态地图过滤 |
| `livox_ros_driver2/` | 官方 driver 1.2.7 与固定版本 SDK2 |
| `ranger_ros/`、`ugv_sdk/` | Ranger ROS 2 驱动和底盘 SDK |

ROS 1 XML launch、旧式 YAML 和未参与构建的旧源文件仅保留在源码树中作为算法
对照；安装空间只暴露 ROS 2 入口。不要从这些参考文件启动系统。
