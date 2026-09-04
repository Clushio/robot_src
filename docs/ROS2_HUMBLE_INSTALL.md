# Ubuntu 22.04 实体机安装、编译与运行

本文面向一台已经安装 ROS 2 Humble、除此之外没有准备过开发环境的 Ubuntu
22.04 实体机。以下示例使用部署账号 `nav`，工作区为
`/home/nav/anav_ros2_ws`。源码目录可以换位置，但当前工程仍有少量地图路径固定为
`/home/nav/maps`，因此实车部署强烈建议使用 `nav` 账号。

> 这是实车控制软件。首次运行必须架空驱动轮或断开动力测试，并准备物理急停；然后
> 才能在封闭场地低速测试。软件急停、速度仲裁和碰撞监控不能代替物理急停。

## 1. 确认系统和 ROS 版本

打开终端执行：

```bash
lsb_release -a
source /opt/ros/humble/setup.bash
echo "$ROS_DISTRO"
ros2 --help >/dev/null && echo "ROS 2 CLI 正常"
```

预期系统代号是 `jammy`，`ROS_DISTRO` 输出 `humble`。如果
`/opt/ros/humble/setup.bash` 不存在，说明 Humble 没有正确安装，应先修复 ROS 2
安装；不要在同一个终端同时 source ROS 1 Noetic。

如果机器安装的只是 `ros-humble-ros-base` 而不是 Desktop，也没有关系，后续
`rosdep` 会补齐 Nav2、RViz2、消息包和构建依赖。

## 2. 创建工作区并复制源码

先安装复制、构建和依赖管理工具：

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake git ripgrep rsync \
  python3-colcon-common-extensions python3-rosdep python3-vcstool
```

创建标准 ROS 2 工作区：

```bash
mkdir -p ~/anav_ros2_ws/src
```

假设源码盘上的目录叫 `src-ros2`，把下面第一条命令中的源路径换成实际路径。命令末尾
的 `/` 不能漏，它表示复制目录内容到工作区的 `src`：

```bash
rsync -av \
  --exclude='.git' --exclude='build' --exclude='install' --exclude='log' \
  /实际路径/src-ros2/ ~/anav_ros2_ws/src/
cd ~/anav_ros2_ws
```

例如源码位于 U 盘 `/media/nav/ROBOT/src-ros2` 时：

```bash
rsync -av \
  --exclude='.git' --exclude='build' --exclude='install' --exclude='log' \
  /media/nav/ROBOT/src-ros2/ ~/anav_ros2_ws/src/
```

检查源码是否完整：

```bash
test -f src/robot_r/package.xml && echo "robot_r: OK"
test -f src/nav/anav_interfaces/package.xml && echo "anav_interfaces: OK"
test -f src/livox_ros_driver2/package.xml && echo "Livox driver: OK"
```

源码根目录有一个历史遗留的 ROS 1 `CMakeLists.txt` 符号链接也不影响构建。不要在
`src` 中运行 `cmake` 或 `catkin_make`；始终在 `~/anav_ros2_ws` 运行 `colcon`。

## 3. 安装全部依赖

### 3.1 初始化 rosdep

每台机器只需要执行一次 `rosdep init`：

```bash
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update --rosdistro humble
```

如果提示 `default sources list file already exists`，说明已经初始化过，直接执行
`rosdep update --rosdistro humble`。

### 3.2 安装 package.xml 声明的依赖

```bash
source /opt/ros/humble/setup.bash
cd ~/anav_ros2_ws
rosdep install \
  --from-paths src \
  --ignore-src \
  --rosdistro humble \
  -r -y
```

这一步会安装工程需要的 Nav2、RViz2、TF2、pluginlib、PCL、Eigen、Boost、ASIO、
gflags、glog、TBB、yaml-cpp、Qt5、joy、rosbag2 和相关 ROS 2 消息包。后续增加包时也
应该重新运行此命令，不要只维护一份手写的 deb 列表。

### 3.3 安装运行工具和 Python 补充依赖

部分运维工具和离线地图脚本不是 C++ 编译依赖，单独安装：

```bash
sudo apt install -y \
  can-utils iproute2 procps gnome-terminal \
  libapr1-dev libaprutil1-dev libasio-dev \
  python3-numpy python3-opencv python3-serial \
  python3-pyqt5 python3-pyqt5.qtsvg \
  pyqt5-dev python3-sip-dev sip-dev
```

其中：

- `libapr1-dev`、`libaprutil1-dev` 用于工作区内的 Livox ROS 2 driver；
- `python3-opencv`、`python3-numpy` 用于拓扑生成和地图点位迁移工具；
- `python3-serial` 用于 MM3V/Tag 串口；
- PyQt5、QtSvg 和 `gnome-terminal` 用于 aNAV 操作界面；
- `can-utils` 提供 `candump`，`iproute2` 用于配置 `can0`。

工程已经带有固定版本的 `livox_ros_driver2` 和 Livox-SDK2 源码，不需要再向
`/usr/local` 安装另一份 Livox SDK，也不要安装 ROS 1 的 `livox_ros_driver`。

检查依赖：

```bash
cd ~/anav_ros2_ws
source /opt/ros/humble/setup.bash
rosdep check --from-paths src --ignore-src --rosdistro humble
```

预期最后显示 `All system dependencies have been satisfied`。如果 `rosdep install`
因为单个可选包失败，可以先保留完整报错再使用 `-r` 继续，不能用删除功能包的方式
绕过依赖。

## 4. 准备地图目录和部署数据

创建运行目录：

```bash
mkdir -p ~/maps/split_map ~/maps/nav_benchmark_results
```

定位和导航需要从旧机器复制同一套地图数据，典型内容如下：

```text
/home/nav/maps/
├── GlobalMap.pcd
├── FeatureMap.pcd
├── split_map/
│   └── map_index.txt 等分块文件
├── map.pgm
├── map.yaml
├── robot_positions.txt
├── topology.yaml
├── autonav_params.yaml          # 可选，AutoNAV 启动参数
└── nav_benchmark_results/       # 可为空，会自动写入结果
```

复制后执行：

```bash
ls -lh ~/maps/GlobalMap.pcd ~/maps/FeatureMap.pcd
ls -lh ~/maps/map.pgm ~/maps/map.yaml
ls -lh ~/maps/robot_positions.txt ~/maps/topology.yaml
grep '^image:' ~/maps/map.yaml
```

`map.yaml` 的 `image:` 可以写同目录相对路径（推荐 `map.pgm`），也可以写存在的绝对
路径。地图、点位和拓扑必须来自同一坐标系；地图或点位改变后要重新生成拓扑，不能
继续使用旧拓扑指纹。

### 非 nav 用户注意

当前以下运行入口仍以 `/home/nav/maps` 为默认值，LIO 源码中也有一个固定
`/home/nav/` 路径：

```bash
rg -n '/home/nav' ~/anav_ros2_ws/src \
  --glob '!docs/**' --glob '!**/3rdparty/**'
```

因此最稳妥的部署方式是使用用户名 `nav`。如果必须使用其他账号，应在编译前统一
修改这些路径并重新编译，或由管理员确认 `/home/nav/maps` 指向实际地图目录。不能只
覆盖 launch 的 `map_file`：那只解决二维地图，LIO 的 PCD 路径仍会找错。

## 5. 编译

首次构建：

```bash
source /opt/ros/humble/setup.bash
cd ~/anav_ros2_ws
colcon build \
  --symlink-install \
  --executor sequential \
  --cmake-args \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_TESTING=OFF \
  -DBUILD_TESTS=OFF
```

>注意这个  -DBUILD_TESTING=OFF -DBUILD_TESTS=OFF 很重要没这个可能编译失败

LIO、PCL 和 Livox SDK 编译时内存占用较大，所以首次部署建议顺序构建。内存充足时
可以去掉 `--executor sequential`。成功后执行：

```bash
source ~/anav_ros2_ws/install/setup.bash
```

每个新终端都必须先 source ROS 2，再 source 本工作区：

```bash
source /opt/ros/humble/setup.bash
source ~/anav_ros2_ws/install/setup.bash
```

确认完整工作区能被识别：

```bash
cd ~/anav_ros2_ws
colcon list
colcon list --names-only | wc -l
```

当前源码应发现 **19 个 package**。如果数量不对，先检查源码复制是否完整以及
`package.xml` 是否存在。

### 只重编一个修改过的包

日常修改后可以让 colcon 自动带上依赖：

```bash
cd ~/anav_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-up-to x2bot_teleop --symlink-install --executor sequential
```

把 `x2bot_teleop` 换成目标包。接口包或公共库变更后，推荐重新完整构建，避免下游
仍链接旧接口。

## 6. 编译后验证

### 6.1 package、接口和插件

```bash
cd ~/anav_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

for package in $(colcon list --names-only); do
  ros2 pkg prefix "$package" >/dev/null || echo "missing: $package"
done

ros2 interface show anav_interfaces/msg/FrozenTopologyPlan
ros2 interface show cmd_vel_arbiter/srv/FinishMotion
ros2 interface show ranger_msgs/action/StopAndCenter
ros2 pkg plugins --package nav2_core | grep -E 'myglobal|jgl'
```

循环没有输出 `missing:` 才正常。插件命令应能看到本工程的全局/局部规划插件。

### 6.2 测试

```bash
cd ~/anav_ros2_ws
colcon test --executor sequential
colcon test-result --verbose
```

当前迁移基线是 **127 tests、0 errors、0 failures、5 skipped**。跳过的是受版本或
上游代码约束的静态检查，不是运行测试失败。不同系统安装的 lint 工具版本可能让
测试总数略有差异，但 errors 和 failures 必须为 0。

### 6.3 不连接硬件的 launch 解析

以下命令只解析入口和参数，不会真正启动节点：

```bash
ros2 launch ranger_bringup ranger_mini_v2.launch.py --show-args
ros2 launch robot_r 3startlocation.launch.py --show-args
ros2 launch robot_r 5nav.launch.py --show-args
ros2 launch robot_r 3navlocations.launch.py --show-args
ros2 launch lio_lite location_360.launch.py --show-args
ros2 launch livox_ros_driver2 msg_MID360_launch.py --show-args
```

任何 `package not found` 或 `launch file not found` 都应在上车前解决。

## 7. 实体机设备配置

### 7.1 CAN 和 Ranger 底盘

插好 USB-CAN 后执行：

```bash
sudo modprobe gs_usb
sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can0 up type can bitrate 500000
ip -details link show can0
candump can0
```

`ip` 应显示 `UP`、`bitrate 500000`，底盘上电后 `candump` 应持续收到帧。按
`Ctrl+C` 停止 `candump`。如果适配器没有使用 `gs_usb`，应按该硬件驱动说明配置，
不要修改 Ranger 控制算法来掩盖接口问题。

### 7.2 Livox MID360s 网络

标准定位入口 `3startlocation.launch.py` 当前使用：

- 主机网卡：`192.168.1.5`
- 雷达：`192.168.1.167`
- 配置文件：`robot_r/config/MID360s_config.json`

仓库另有单雷达默认配置 `MID360_config.json`，其主机/雷达地址是
`192.168.2.111` / `192.168.2.142`。必须确认实际启动入口使用哪个配置。

先找出连接雷达的网卡名：

```bash
ip -br link
ip -br address
```

假设网卡叫 `enp3s0`，按当前 MID360s 配置临时设置地址：

```bash
sudo ip address add 192.168.1.5/24 dev enp3s0
sudo ip link set enp3s0 up
ping -c 3 192.168.1.167
```

`enp3s0` 必须换成真实网卡。地址要永久生效时使用 Ubuntu 的网络设置/Netplan，避免
同时给两块网卡设置冲突网段。若现场 IP 不同，应修改 JSON 中所有 host IP 和雷达 IP
后重新编译，或把外部 JSON 通过 launch 的 `config_file:=...` 参数传入。

### 7.3 MM3V/Tag 串口

```bash
sudo usermod -aG dialout "$USER"
```

执行后注销并重新登录一次，组权限才会生效。重新插入设备并检查：

```bash
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
groups
```

默认配置是 `/dev/ttyUSB0`、38400 baud。Tag 网络还包含现场相关地址：TCP 服务默认
绑定 `192.168.3.216:12345`，MM3V UDP 默认发送到 `192.168.3.17:22222`。部署前要按
实体机和接收端实际地址修改或传参。

### 7.4 手柄（可选）

```bash
ls -l /dev/input/js* 2>/dev/null
ros2 run joy joy_enumerate_devices
```

若无权限，可把用户加入设备所属组后重新登录。首次使用必须架空轮子核对轴和按键
映射，不能假设不同型号手柄的编号一致。

## 8. 启动和运行

### 8.1 GUI 方式

每个终端先执行：

```bash
source /opt/ros/humble/setup.bash
source ~/anav_ros2_ws/install/setup.bash
```

启动操作界面：

```bash
ros2 run anav_ranger aNAV_ranger.py
```

日常操作建议使用 GUI，详细按钮顺序见 [真机操作手册](OPERATIONS.md)。GUI 会使用
`gnome-terminal` 打开各启动进程；如果桌面不是 GNOME，仍需保证
`gnome-terminal` 命令可用。

### 8.2 命令行方式

每条 launch 建议放在独立终端，且每个终端都 source 两个 setup 文件：

```bash
# 终端 1：底盘、速度仲裁和碰撞监控
ros2 launch ranger_bringup ranger_mini_v2.launch.py

# 终端 2：MID360s、二维地图、LIO 定位、TF 和 RViz2
ros2 launch robot_r 3startlocation.launch.py

# 终端 3：Nav2、定制规划器和 B-spline 跟踪
ros2 launch robot_r 5nav.launch.py

# 终端 4：AutoNAV 拓扑任务和导航基准记录
ros2 launch robot_r 3navlocations.launch.py
```

第一次不要四条命令连续启动。应等待当前层数据和状态正常后再启动下一层。

可选入口：

```bash
# 三维建图
ros2 launch robot_r s2lam.launch.py

# PCD 生成二维地图
ros2 launch robot_r 4genmap.launch.py

# MM3V/Tag
ros2 launch robot_r 6tagReadAndCtl_mm3v.launch.py \
  port:=/dev/ttyUSB0 baudrate:=38400 \
  udp_feedback_host:=192.168.3.17
```

### 8.3 启动后的基本检查

```bash
ros2 node list
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic hz /Odometry
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo map base_link
ros2 action list -t
ros2 service list -t | grep -E '(anav|plan_path_and_go|finish_motion)'
ros2 topic list -t | grep '^/cmd_vel'
```

重点确认 TF 保持 `map -> body -> base_link -> livox_frame` 兼容关系，以及最终速度链路
是 `/cmd_vel/nav|tag|teleop|safety -> /cmd_vel/candidate -> /cmd_vel`。业务节点不应绕过
仲裁器和碰撞监控直接控制底盘。

### 8.4 可选：每次开终端自动加载环境

先确认工作区构建成功，再在 `~/.bashrc` 末尾加入：

```bash
source /opt/ros/humble/setup.bash
source ~/anav_ros2_ws/install/setup.bash
```

不要同时自动 source Noetic 或另一个覆盖工作区。修改后新开终端验证
`echo "$AMENT_PREFIX_PATH"`。

## 9. 常见问题

### `ros2: command not found` 或 package 找不到

```bash
source /opt/ros/humble/setup.bash
source ~/anav_ros2_ws/install/setup.bash
ros2 pkg prefix robot_r
```

若 `install/setup.bash` 不存在，说明工作区还没成功构建。若只构建了单包，它的工作区
内依赖可能没安装，应回到工作区根目录完整构建。

### `colcon: command not found`

```bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions
```

### `rosdep init`、`rosdep update` 失败

- `already exists`：不是故障，跳过 `init`；
- DNS、超时、证书错误：先修复系统时间、DNS、代理或软件源网络；
- `No definition of [某键]`：确认 `--rosdistro humble` 和 Ubuntu 22.04，记录完整键名；
- apt 被占用：等系统的软件更新进程结束后重试，不要直接删除 apt 锁文件。

### CMake 找不到 PCL、Eigen、glog、gflags、TBB、APR 或 ASIO

先重跑 rosdep：

```bash
cd ~/anav_ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
```

仍缺失时可核对以下 Ubuntu 包：`libpcl-dev`、`libeigen3-dev`、
`libgoogle-glog-dev`、`libgflags-dev`、`libtbb-dev`、`libapr1-dev`、
`libaprutil1-dev`、`libasio-dev`、`libyaml-cpp-dev`。不要混用自行编译到
`/usr/local` 的另一套 PCL/Livox SDK。

### 编译出现 `Killed`、`cc1plus` 被终止或机器卡死

通常是内存不足。关闭 RViz/浏览器后顺序构建：

```bash
cd ~/anav_ros2_ws
colcon build --symlink-install --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

若仍失败，应增加交换空间或物理内存，并查看 `dmesg -T | tail` 是否有 OOM 记录。

### 移动源码后出现旧绝对路径或奇怪的链接错误

CMake 缓存保存了旧工作区路径。确认当前目录确实是目标工作区后，仅清理该工作区的
构建产物并重建：

```bash
cd ~/anav_ros2_ws
pwd
rm -rf build install log
source /opt/ros/humble/setup.bash
colcon build --symlink-install --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

这不会删除 `src`，但仍应先检查 `pwd`，绝不能在错误目录照抄 `rm -rf`。

### launch 能解析，但定位报地图不存在

依次检查：

```bash
ls -lh /home/nav/maps/GlobalMap.pcd /home/nav/maps/FeatureMap.pcd
ls -lh /home/nav/maps/map.yaml /home/nav/maps/map.pgm
grep '^image:' /home/nav/maps/map.yaml
```

如果登录用户不是 `nav`，参见“非 nav 用户注意”。如果二维地图正常但 LIO 报 PCD
不存在，通常就是只改了 `map_file`、没有处理 LIO 固定路径。

### Livox 启动后没有 `/livox/lidar` 或 UDP bind 失败

- 检查实际 launch 使用的是 `MID360s_config.json` 还是 `MID360_config.json`；
- `ip -br address` 必须包含 JSON 中配置的主机 IP；
- `ping` 雷达 IP，并检查网线、防火墙、网段和端口冲突；
- 同一雷达不能同时被两个 driver 进程占用；
- 修改 `src` 下 JSON 后需要重新 build，`--symlink-install` 下通常即时可见，但仍应
  检查 `install/robot_r/share/robot_r/config/` 中的实际文件。

### `can0` 不存在、不能 UP 或底盘无数据

```bash
lsusb
ip link show
sudo modprobe gs_usb
sudo ip link set can0 up type can bitrate 500000
ip -details link show can0
candump can0
```

`Cannot find device` 是 USB-CAN 驱动/命名问题；`Device or resource busy` 时先执行
`sudo ip link set can0 down` 再重新配置；有接口但无帧时检查底盘电源、CAN-H/CAN-L、
终端电阻和波特率。

### 串口 `Permission denied` 或设备号变化

```bash
sudo usermod -aG dialout "$USER"
```

注销登录后再试。用 `ls -l /dev/serial/by-id/` 查稳定设备名，必要时通过 launch 的
`port:=...` 覆盖 `/dev/ttyUSB0`。不建议长期用 `sudo ros2 ...`，否则环境、地图归属和
日志权限都会混乱。

### PyQt5/QtSvg 缺失、GUI 不显示或 gnome-terminal 找不到

```bash
sudo apt install -y \
  python3-pyqt5 python3-pyqt5.qtsvg gnome-terminal
python3 -c 'from PyQt5 import QtWidgets, QtSvg; print("PyQt5 OK")'
echo "$DISPLAY"
```

实体机需要登录图形桌面且 `DISPLAY` 有值。通过纯 SSH 启动 GUI 时还需要正确的 X11
转发；仅做后台导航时可直接使用命令行 launch。

### Nav2 报插件类不存在

```bash
source /opt/ros/humble/setup.bash
source ~/anav_ros2_ws/install/setup.bash
ros2 pkg plugins --package nav2_core | grep -E 'myglobal|jgl'
```

没有输出时重新完整构建并 source 新的 `install/setup.bash`。还要确认没有 source
另一个同名旧工作区覆盖当前 overlay。

### Nav2 节点存在，但 action 不可用

```bash
ros2 lifecycle nodes
ros2 action list -t | grep navigate
```

检查 map、TF 和 Nav2 lifecycle manager 日志。节点进程存在不代表已进入 active；
不要在定位和 TF 未稳定时强行下发目标。

### 有路径但机器人不动

依次观察：

```bash
ros2 topic echo /cmd_vel/nav
ros2 topic echo /cmd_vel/candidate
ros2 topic echo /cmd_vel
ros2 topic echo /diagnostics
```

若 `/cmd_vel/nav` 有值但最终 `/cmd_vel` 为零，检查软件急停、仲裁超时、odom/TF/局部
地图新鲜度和碰撞监控原因。不能通过停掉碰撞监控来换取运动。

### 多机之间看不到 topic

两台机器要处于可达网络，ROS 2 发行版和 `ROS_DOMAIN_ID` 要一致：

```bash
echo "${ROS_DOMAIN_ID:-0}"
echo "${RMW_IMPLEMENTATION:-default}"
```

检查防火墙、DDS 组播/发现和网卡路由。单机进程互相可见而跨机不可见，一般是网络或
DDS 配置问题，不是 topic 名称问题。

## 10. 更新源码后的标准重编流程

把新源码同步到 `~/anav_ros2_ws/src` 后执行：

```bash
cd ~/anav_ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
colcon build --symlink-install --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
colcon test --executor sequential
colcon test-result --verbose
```

如果接口定义、CMake 安装规则、插件 XML 或 package 名称变化，使用第 9 节的干净重建
流程。仅 Python 脚本改动且使用 `--symlink-install` 时通常不需要全量编译，但正式部署
前仍建议完整构建和测试一次。

## 11. 真机部署验收清单

- `colcon list` 发现 19 个 package，所有包都能被 `ros2 pkg prefix` 找到；
- `colcon test-result --verbose` 为 0 errors、0 failures；
- 六个关键 launch 的 `--show-args` 都能解析；
- Livox 点云、IMU、LIO odometry 和底盘 odom 频率稳定；
- TF、二维地图、三维地图和机器人姿态在 RViz 中对齐；
- Nav2 lifecycle 全部 active，定制 planner/controller 插件成功加载；
- `/cmd_vel` 始终经过仲裁器和碰撞监控，软件/物理急停均实测有效；
- 串口、Tag 坐标符号、CAN、雷达 IP 与现场硬件一致；
- 从架空轮测试到封闭场地低速测试逐级通过；
- 终点锁定、停车回正、固定路线、自动绕路、局部障碍、footprint 碰撞和 Hybrid A*
  调试开关仍需结合现场逐项验收。

更完整的启动、建图、点位、拓扑、循环任务和故障处置流程见
[真机操作手册](OPERATIONS.md)。
