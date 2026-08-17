# Ranger AutoNAV 导航工作区

这是面向 Ranger 移动底盘的 ROS 1 导航工作区，包含 MID360s
雷达接入、LIO 建图/定位、拓扑任务调度、B 样条路径跟踪、Tag 工位精调、
速度仲裁、碰撞监控和 PyQt5 操作界面。

新机器统一使用标准工作区 `~/catkin_ws`，把本仓库内容放在
`~/catkin_ws/src`。`build/`、`devel/` 和 CMake 缓存应在目标机器重新生成，不属于
需要发布或迁移的源码。

> **安全提示**
>
> 这是实车控制软件。首次编译或修改导航、速度、安全参数后，应先断开动力、
> 在封闭场地低速测试。GUI 中的“紧急停止”是软件安全通道，不能替代底盘物理急停。
> 遥控模式可能进入碰撞监控的人工旁路，操作人员必须负责现场安全。
> 

## 从哪里开始

第一次接触仓库，建议按顺序阅读：

1. 本文：了解目录和系统入口。
2. [新机器最小执行方案](docs/最小执行方案.md)：从已有 Noetic 到干净编译和首次启动。
3. [系统架构](docs/ARCHITECTURE.md)：了解节点、Topic、Service 和速度安全链路。
4. [操作手册](docs/OPERATIONS.md)：完成建图、点位、拓扑、定位和导航。
5. [启动编排](robot_r/README.md)：确认每个 launch 文件实际会启动什么。
6. [AutoNAV 与 Tag](nav/x2bot_teleop/README.md)：理解拓扑任务和工位精调。
7. [速度仲裁](cmd_vel_arbiter/README.md)与
   [碰撞监控](collision_monitor/README.md)：理解最终速度为何被放行、减速或停车。

## 系统概览

```text
MID360s ──> LIO-Lite ──> /Odometry、点云、静态地图
                              │
点位 + topology.yaml ──> AutoNAV ──> move_base / B样条 ──> /cmd_vel/nav
MM3V Tag ───────────────> TagCtl ─────────────────────────> /cmd_vel/tag
手柄 ───────────────────> teleop ─────────────────────────> /cmd_vel/teleop
GUI 软件急停 ─────────────────────────────────────────────> /cmd_vel/safety
                                                              │
                                                              v
                                                    cmd_vel_arbiter
                                                              │
                                                     /cmd_vel/candidate
                                                              │
                         /map + 局部代价地图 + /odom ─> collision_monitor
                                                              │
                                                          /cmd_vel
                                                              │
                                                     Ranger 底盘驱动
```

关键约束：

- 业务节点不得直接发布最终 `/cmd_vel`。
- `cmd_vel_arbiter` 负责来源优先级、超时和速度上限。
- `collision_monitor` 是唯一的最终 `/cmd_vel` 发布者。
- 底盘启动文件会同时启动速度仲裁和碰撞监控。

## 目录说明

| 目录 | 作用 | 性质 |
|---|---|---|
| `script/` | PyQt5 GUI、拓扑构建、地图点位迁移、TCP 服务 | 项目业务代码 |
| `robot_r/` | 真机 launch 编排、RViz 配置、MID360s/LIO 参数 | 项目集成层 |
| `nav/x2bot_teleop/` | AutoNAV、点位记录、Tag 精调、手柄转换 | 项目业务代码 |
| `nav/jgl_dwa_local_planner/` | DWA、本项目的 B 样条生成和跟踪、代价地图插件 | 深度定制 |
| `nav/mxb_move_base/` | 定制 move_base | 深度定制 |
| `nav/myglobal_planner/` | 定制全局规划器 | 深度定制 |
| `cmd_vel_arbiter/` | 多来源速度命令仲裁、任务结束停车回正 | 项目安全链路 |
| `collision_monitor/` | footprint 运动预测、减速和最终停车 | 项目安全链路 |
| `slam/LIO-Lite-eskf-base/` | LIO 建图/定位及动态点过滤 | 上游基础上的定制 |
| `livox_ros_driver2/` | Livox MID360/MID360s 驱动 | 引入的驱动代码 |
| `hardware/livx2pcl/` | Livox 点云转换 | 项目硬件适配 |
| `ranger_ros/`、`ugv_sdk/` | Ranger ROS 驱动和 AgileX SDK | 上游基础上的定制 |
| `slam/pcd2pgm/` | 三维点云转二维占据地图 | 地图工具 |
| `obstacle_detection/` | 独立障碍物检测实验包 | 非主导航入口 |

`script/stpnav_B.py`、`stpnav_C.py`、`stpnav_D.py`、`stpbuild.py`、
`pathRecord.py` 等脚本保留了旧路径或实验入口。新功能以
`script/aNAV_ranger.py`、`build_topology.py`、`transfer_robot_positions.py`
以及 `robot_r/launch/` 当前入口为准；使用旧脚本前必须先核对其中引用的 launch
文件和绝对路径。

## 环境与硬件

当前工作区按以下环境组织：

- Ubuntu 20.04 / ROS Noetic。
- Python 3、PyQt5、OpenCV、NumPy、pyserial。
- Ranger Mini V2，经 `can0`、500000 bit/s 通信。
- Livox MID360s 雷达。
- MM3V Tag 读头（可选，默认 `/dev/ttyUSB0`、38400）。
- 图形桌面、RViz 和 `gnome-terminal`（GUI 会打开独立日志标签页）。

新机器已有 Noetic 时，依赖安装、Livox-SDK2 1.3.1 和完整 apt 清单直接按
[新机器最小执行方案](docs/最小执行方案.md)操作。rosdep 的核心命令是：

```bash
source /opt/ros/noetic/setup.bash
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src --rosdistro noetic \
  --skip-keys='livox_ros_driver GTSAM' -r -y
```

Livox SDK/驱动的额外要求见
[`livox_ros_driver2/README.md`](livox_ros_driver2/README.md)。不同机器的用户目录、
网卡名、Tag 串口和 IP 不应直接照抄，见[操作前检查](docs/OPERATIONS.md#操作前检查)。

## 编译

```bash
source /opt/ros/noetic/setup.bash
cd ~/catkin_ws
catkin_make -j2 -l2 -DROS_EDITION=ROS1 -DCMAKE_BUILD_TYPE=Release
source ~/catkin_ws/devel/setup.bash
```

该命令已在全新的标准工作区完成无缓存全量预编译并返回 0。当前版本的消息生成依赖
已经完整，不需要额外手工修改 `CMakeLists.txt`。

确认工作区被 ROS 找到：

```bash
rospack find robot_r
rospack find x2bot_teleop
rospack find collision_monitor
```

修改 `.msg`、`.srv`、`.action`、CMake 或 package 依赖后必须重新编译。GUI 会尝试
自动定位 `~/catkin_ws/devel/env.sh`，但首次构建和命令行调试仍建议显式 `source`。

## 启动 GUI

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
cd ~/catkin_ws
python3 src/script/aNAV_ranger.py
```

GUI 是推荐的操作入口，它能够启动 CAN、底盘、Tag、定位、MoveBase、AutoNAV、
建图和 RViz，并显示各进程状态。第一次运行时不要直接点击导航目标，应先完成
[操作前检查](docs/OPERATIONS.md#操作前检查)并确认物理急停可用。

## 命令行最短启动链

下面用于理解系统和排查问题。每条命令建议放在独立终端中，并先执行：

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
```

真机导航的基本顺序：

```bash
# 1. 配置 CAN；按实际权限执行，不要依赖 GUI 中的测试密码
sudo ip link set can0 up type can bitrate 500000

# 2. 底盘，同时启动 cmd_vel_arbiter 和 collision_monitor
roslaunch ranger_bringup ranger_mini_v2.launch

# 3. MID360s + map_server + LIO 定位 + RViz
roslaunch robot_r 3startlocation.launch

# 4. 定制 move_base、全局/局部代价地图和 B 样条局部规划
roslaunch robot_r 5nav.launch

# 5. 拓扑任务服务
roslaunch robot_r 3navlocations.launch
```

启动后至少检查：

```bash
rostopic hz /Odometry
rostopic hz /odom
rostopic echo -n 1 /collision_monitor/status
rosservice list | grep anav
rosservice list | grep plan_path_and_go
```

完整建图、点位和导航流程见[操作手册](docs/OPERATIONS.md)。

## 地图与任务数据

业务数据统一放在 `~/maps`。部署前应检查源码和 launch 中的固定绝对路径，并将其
参数化或调整为当前账号的目录；编译成功不代表运行时的数据路径一定正确。

```text
~/maps/
├── GlobalMap.pcd                 # LIO 输出的三维静态地图
├── GlobalMap_raw.pcd             # 可选，动态过滤前的原始点云
├── FeatureMap.pcd                # 定位使用的特征地图
├── split_map/                    # 分块地图及索引
├── map.pgm                       # 二维占据地图
├── map.yaml                      # map_server 配置
├── robot_positions.txt           # 导航点和工位点
├── topology.yaml                 # 由 build_topology.py 生成
└── autonav_params.yaml           # GUI 保存的 AutoNAV 参数，可选
```

`topology.yaml` 带有输入文件指纹。修改 `map.yaml`、PGM 或
`robot_positions.txt` 后应重新构建拓扑；AutoNAV 会拒绝明显过期或不安全的拓扑，
不会静默退回“按文件顺序连线”。

## 常用配置位置

| 配置 | 文件 |
|---|---|
| MID360s/LIO、重定位和动态点过滤 | `robot_r/lio/config/mid360.yaml` |
| MoveBase 与 B 样条跟踪 | `nav/jgl_dwa_local_planner/param/` |
| 机器人 footprint | `collision_monitor/config/robot_footprint.yaml` |
| 速度来源、优先级、超时和上限 | `cmd_vel_arbiter/config/cmd_vel_arbiter.yaml` |
| 碰撞预测、padding、制动参数 | `collision_monitor/config/collision_monitor.yaml` |
| Tag 串口、坐标符号、偏置和 UDP | `robot_r/launch/6tagReadAndCtl_mm3v.launch` |
| AutoNAV 阻塞/超时/到达阈值 | `~/maps/autonav_params.yaml` 或 GUI“导航参数”页 |

不要只修改一处 footprint：代价地图和最终碰撞门共用
`collision_monitor/config/robot_footprint.yaml`，应保持唯一真实来源。

## 测试与只读检查

```bash
# 拓扑生成器单元测试
python3 src/script/test_build_topology.py

# 只审计现有点位和地图，不替换 topology.yaml
python3 src/script/build_topology.py --audit

# catkin 测试
catkin_make run_tests_collision_monitor
catkin_make run_tests_jgl_dwa_local_planner
catkin_make run_tests_lio_lite
catkin_test_results build
```

涉及实车速度的测试不能只以“编译通过”为验收标准，还需要低速现场验证制动距离、
footprint、传感器盲区、Tag 坐标方向和轮组回正。

## 常见问题

### GUI 提示 ROS 未连接

确认 `roscore` 可用、工作区已编译，并检查 `ROS_MASTER_URI`。GUI 会自动加载
`devel/env.sh`，但不会自动安装缺失的 Python/ROS 依赖。

### 定位启动后没有数据

检查 `/livox/lidar`、`/livox/imu`、TF 和 `/Odometry`，并确认
`~/maps/GlobalMap.pcd`、`FeatureMap.pcd` 与当前环境匹配。

### AutoNAV 拒绝启动或提示拓扑过期

检查 `~/maps/map.yaml`、对应 PGM、`robot_positions.txt` 和 `topology.yaml`，然后
重新运行 `python3 ~/catkin_ws/src/script/build_topology.py`。

### 有规划但底盘不动

按速度链路逐段检查 `/cmd_vel/nav`、`/cmd_vel/candidate`、`/cmd_vel` 和
`/collision_monitor/status`。安全数据不完整时碰撞监控会按失败即停车处理。

### CAN 启动失败

确认接口名、USB-CAN 设备、权限和 bitrate。代码中仍存在用于调试设备的固定 sudo
密码，不应把它作为部署方案；应改用正确的系统权限或受控脚本。

## 维护约定

- 新增主流程 launch、Topic、Service、Action 或 `~/maps` 文件时，同步更新文档。
- 标明实验/兼容代码，不要让新人通过文件名猜测生产入口。
- 硬件地址、串口、IP、用户目录和密码应通过参数或部署配置提供。
- 修改速度、安全、footprint 或制动参数时，记录测试条件和实车验证结果。
- 提交生成物前检查 `.gitignore`；地图、PCD、bag、`build/`、`devel/` 不应进入 Git。
