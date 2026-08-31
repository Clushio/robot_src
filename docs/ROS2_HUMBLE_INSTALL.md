# ROS 2 Humble 新机安装与构建

本文用于在一台新的 Ubuntu 22.04 / ROS 2 Humble 机器或容器中复现当前工作区。
本文记录的是迁移和验证期间实际使用的依赖，不依赖 ROS 1 Noetic 环境。

## 1. 目录和容器约定

当前部署约定：

- 宿主机源码：`/home/nav/lyf/version/src-ros2`
- Docker 容器：`nav-humble`
- 容器工作区：`/workspace`
- 容器源码：`/workspace/src`
- ROS 2：Humble，Ubuntu 22.04

进入已有容器：

```bash
sudo docker exec -it nav-humble bash
source /opt/ros/humble/setup.bash
cd /workspace
```

GUI、CAN、串口和雷达需要容器具备相应能力。建议使用 host network，并按安全
策略映射 X11/Wayland、`/dev/ttyUSB*`、CAN 设备及必要的设备权限。仅编译时不需要
连接实物设备。

## 2. 系统依赖

先安装构建和 rosdep 工具：

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake git \
  python3-colcon-common-extensions python3-rosdep
```

迁移过程中实际新增或确认安装的非 ROS apt 包如下：

```bash
sudo apt install -y \
  libaprutil1-dev \
  python3-serial \
  python3-pyqt5 python3-pyqt5.qtsvg \
  pyqt5-dev python3-sip-dev sip-dev \
  gnome-terminal iproute2
```

用途和分类：

| 依赖 | 分类 | 用途 |
|---|---|---|
| `libaprutil1-dev` | Docker 系统依赖 | 官方 Livox driver 的 APR 日志依赖 |
| `python3-serial` | Docker 系统依赖 | MM3V/Tag 串口读取 |
| `python3-pyqt5`、`python3-pyqt5.qtsvg` | Docker 系统依赖 | aNAV GUI |
| `pyqt5-dev`、`python3-sip-dev`、`sip-dev` | Docker 系统/构建依赖 | Qt/Python 绑定开发文件；其中部分由 apt 传递安装 |
| `gnome-terminal` | Docker 系统依赖 | GUI/RViz 面板打开独立终端 |
| `iproute2` | Docker 系统依赖 | 配置 `can0` 和雷达网卡 |

rosdep 还会解析 PCL、Eigen、Boost、ASIO、gflags、glog、TBB、yaml-cpp、Qt、
Nav2、RViz2、TF2、pluginlib、joy、rosbag2 及消息包。不要手工维护一份与
`package.xml` 脱节的 ROS deb 清单，以 rosdep 结果为准：

```bash
sudo rosdep init  # 仅首次、且 /etc/ros/rosdep/sources.list.d 尚不存在时执行
rosdep update
cd /workspace
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
rosdep check --from-paths src --ignore-src
```

当前验证结果应为 `All system dependencies have been satisfied`。

### 可选 MAVROS

`robot_r/testlocation.launch.py` 的 `mavros_enable` 默认是 `false`。核心导航不依赖
MAVROS；若确实要启用 PX4/MAVROS 测试入口，再安装 Humble 对应的 MAVROS 包和
geographiclib 数据。未安装 MAVROS 属于“可选 ROS 2 package 缺失”，不是核心迁移
缺陷。

## 3. Livox 官方驱动和 SDK2

工作区已经包含可复现的官方源码，不要再安装或移植 ROS 1
`livox_ros_driver`：

- `livox_ros_driver2` 1.2.7：官方 commit
  `4a1def929e5b59c7a8122d19fce6efba581ce9f7`
- `Livox-SDK2`：官方 commit
  `08f523c930b2f0ba1e98a6afaa8d7476bf479908`

SDK2 已 vendored 到 driver 源码中，普通 `colcon build` 会一起编译；不需要在
`/usr/local` 另装一份 SDK。准确来源和本地集成范围见
`livox_ros_driver2/UPSTREAM_VERSIONS.md`。

这是对“原依赖没有合适 ROS 2 版本”的处理：旧 driver1 不迁移，使用官方 ROS 2
driver2，并保持业务使用的 `/livox/lidar`、`/livox/imu` 和 `CustomMsg` 接口。

## 4. 构建

推荐先从干净的构建产物开始：

```bash
source /opt/ros/humble/setup.bash
cd /workspace
rm -rf build install log
colcon build --symlink-install --executor sequential
source /workspace/install/setup.bash
```

LIO/PCL 编译占用内存较大，因此默认使用顺序执行器。需要定位单包问题时，按依赖
从底层向上构建：

```bash
colcon build --packages-select ugv_sdk --symlink-install
colcon build --packages-select ranger_msgs --symlink-install
colcon build --packages-select livox_ros_driver2 --symlink-install
colcon build --packages-up-to ranger_base --symlink-install
colcon build --packages-up-to lio_lite --symlink-install
colcon build --packages-up-to mxb_move_base --symlink-install
colcon build --packages-up-to robot_r anav_ranger --symlink-install
```

每次构建新终端都要重新 source：

```bash
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
```

## 5. 编译后验证

### package 和依赖

```bash
cd /workspace
colcon list

for package in $(colcon list --names-only); do
  ros2 pkg prefix "$package" >/dev/null || echo "missing: $package"
done

rosdep check --from-paths src --ignore-src
```

应发现 18 个 package，且 18 个都能由 `ros2 pkg prefix` 找到。

### 测试

```bash
colcon test --executor sequential
colcon test-result --verbose
```

迁移验收时的完整结果为：

```text
Summary: 116 tests, 0 errors, 0 failures, 5 skipped
```

跳过项来自 cppcheck 2.7 性能保护及为保持上游 Ranger driver 逻辑不变而禁用的
纯样式检查，不是功能测试失败。

### launch 语法和依赖解析

关键入口可先只解析参数，不连接硬件：

```bash
ros2 launch ranger_bringup ranger_mini_v2.launch.py --show-args
ros2 launch robot_r 3startlocation.launch.py --show-args
ros2 launch robot_r 5nav.launch.py --show-args
ros2 launch robot_r 3navlocations.launch.py --show-args
ros2 launch lio_lite location_360.launch.py --show-args
ros2 launch livox_ros_driver2 msg_MID360_launch.py --show-args
```

验收时共解析了 42 个工作区 `*.launch.py` 和 9 个官方 Livox Python launch，
失败数为 0。

## 6. 真机资源和网络

### 地图数据

启动定位/导航前准备 `~/maps`：

```text
~/maps/
├── GlobalMap.pcd
├── FeatureMap.pcd
├── split_map/
├── map.pgm
├── map.yaml
├── robot_positions.txt
├── topology.yaml
├── autonav_params.yaml          # 可选
└── nav_benchmark_results/       # 自动创建
```

`map.yaml` 内的 image 路径必须在容器中可访问。修改地图或点位后重新生成
`topology.yaml`，不要沿用指纹已过期的拓扑。

### CAN

```bash
sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can0 up type can bitrate 500000
ip -details link show can0
```

容器内看不到 `can0` 时，先处理 Docker device/network 权限，而不是修改
`ranger_base` 算法。

### Livox

真机入口 `robot_r/launch/livox_mid360.launch.py` 默认读取
`robot_r/config/MID360_config.json`。当前配置包含主机地址 `192.168.2.111` 和雷达
地址 `192.168.2.142`；新机器必须按实际网卡和雷达地址修改，并保证容器使用 host
network 或等价 UDP 路由。没有对应网卡地址时，driver 的 UDP bind 失败属于网络
配置问题。

### Tag/MM3V

默认串口是 `/dev/ttyUSB0`、38400 baud。确认容器设备映射和用户串口权限，并按
现场设置 `udp_feedback_host`、目标 IP 和坐标符号。

## 7. 常用运行检查

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic hz /Odometry
ros2 topic hz /odom
ros2 action list -t
ros2 service list -t | grep -E '(anav|plan_path_and_go|finish_motion)'
ros2 topic list -t | grep '^/cmd_vel'
ros2 run tf2_ros tf2_echo map base_link
```

只有上述数据链、TF、服务和 action 都正常，并完成低速实车测试后，才可认为部署
完成。
