# 真机操作手册

本文给出当前默认配置下的标准流程。所有涉及运动的步骤都需要现场人员、物理急停和
足够安全空间。

## 操作前检查

### 软件和数据

- 工作区已经成功编译，当前终端已 source `devel/setup.bash`。
- `~/maps` 存在，且当前任务所需文件齐全。
- `map.yaml` 中的 `image` 能解析到正确的 PGM。
- `robot_positions.txt` 与当前地图属于同一坐标系。
- `topology.yaml` 由当前地图和点位生成，不是旧环境文件。
- MID360s、底盘、USB-CAN 和 MM3V 串口权限正确。
- `ROS_MASTER_URI`、Tag TCP/UDP IP 与本机网络一致。

### 机械和现场

- 机器人 footprint、载荷外形和参数一致。
- 轮组可以正常转向、回正，底盘无故障码。
- 物理急停有效且操作人员能立即触达。
- 第一次运行使用低速、封闭区域，并清空机器人周围人员和移动物体。

### 当前需要特别留意的固定配置

仓库仍包含部署机相关值：

- 多处 `/home/nav/maps`；
- GUI 中用于调试设备的固定 sudo 密码；
- Tag TCP 默认绑定 `192.168.3.216:12345`；
- MM3V UDP 默认发送到 `192.168.3.17:22222`；
- MM3V 默认串口 `/dev/ttyUSB0`。

换机器前应参数化或修改部署配置。不要为了“先跑起来”复制密码或放宽系统权限。

## 启动 GUI

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
cd ~/catkin_ws
python3 src/script/aNAV_ranger.py
```

推荐通过 GUI 完成日常作业。GUI 会把大多数 roslaunch 放到独立终端标签页，关闭
主窗口时会尝试停止自己管理的子进程。地图保存依赖正常退出，不要用 `kill -9`。

## 标准导航流程

1. 接通设备并确认物理急停。
2. 在“设备与工具”中启动 CAN。
3. 启动底盘，确认 CAN、底盘、速度仲裁状态正常。
4. 启动定位，等待 GUI 显示持续有效的定位数据。
5. 在 RViz 检查地图、机器人姿态、点云与 TF 是否重合。
6. 确认 `map.yaml`、点位和拓扑来自当前环境。
7. 启动自动导航；GUI 会先启动/等待 MoveBase，再启动 AutoNAV。
8. 选择普通导航策略：
   - 自动绕路：堵塞边临时封锁，等待后重新选路；
   - 固定路线：保持原拓扑路线，堵塞时停车等待。
9. 下发一个近距离目标进行首次验证。
10. 观察 `/collision_monitor/status`、局部代价地图和现场运动。

任务结束、取消或失败后，系统会请求停车和轮组回正。如果 GUI 提示回正未确认，
不要立即下发新任务，应先检查底盘状态。

## 命令行启动与停止

每个终端先执行：

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
```

按顺序启动：

```bash
sudo ip link set can0 up type can bitrate 500000
roslaunch ranger_bringup ranger_mini_v2.launch
roslaunch robot_r 3startlocation.launch
roslaunch robot_r 5nav.launch
roslaunch robot_r 3navlocations.launch
```

停止时先取消 AutoNAV，再停止 MoveBase，最后停止定位和底盘。不要先杀掉仲裁器或
碰撞监控后继续保留底盘运动节点。必要时先使用物理急停。

## 三维建图

GUI“地图构建”页的“开始构建地图”对应：

```bash
roslaunch robot_r s2lam.launch
```

操作要点：

1. 缓慢覆盖作业区域，尽量形成闭环。
2. 避免大量人员或车辆长期遮挡固定结构。
3. 观察 `/cloud_registered` 以及静态/动态调试点云。
4. 结束时通过 GUI 正常停止 roslaunch，让 LIO 执行保存逻辑。
5. 检查 `~/maps/GlobalMap.pcd`、`FeatureMap.pcd`、可选的
   `GlobalMap_raw.pcd` 和 `split_map/`。

动态点过滤参数在 `robot_r/lio/config/mid360.yaml` 的 `dynamic_filter` 下。改变体素、
命中次数或自由空间删除阈值后，需要重新建图并对静态结构保留率进行验证。

## 生成二维地图

GUI“生成并保存二维地图”会启动点云转二维地图，并调用 `map_saver`。命令行入口：

```bash
roslaunch robot_r 4genmap.launch
```

确认 `/map` 正常后：

```bash
rosrun map_server map_saver -f ~/maps/map
```

输出应包含 `~/maps/map.pgm` 和 `map.yaml`。二维过滤、高度范围、分辨率等在
`robot_r/launch/pcd2pgm.launch` 中。生成后在 RViz 检查墙体、通道宽度、未知区域和
低矮障碍物是否符合实际。

## 记录和维护点位

GUI 支持两种模式：

- 实车采点：启动 `robot_r/3settinglocation.launch`，把机器人移动到目标姿态后记录。
- RViz 采点：无需动车，在地图上使用“记录点位”工具（快捷键 `R`）拖出位置和朝向。

点位保存在 `~/maps/robot_positions.txt`。每行格式为：

```text
x y z roll pitch yaw [label]
```

- 普通点没有 label。
- 工位使用 `W1`、`W2` 等唯一 label。
- 文件中的有效行顺序决定节点 ID，但不自动表示拓扑连线顺序。
- GUI 修改、删除、排序或导入点位后，应重新构建拓扑。

GUI 会在覆盖点位前建立时间戳备份。提交或复制地图时，应把 PGM、YAML、点位和拓扑
作为同一组数据管理。

## 构建与审计拓扑

只审计，不覆盖现有文件：

```bash
cd ~/catkin_ws
python3 src/script/build_topology.py --audit
```

生成 `~/maps/topology.yaml`：

```bash
python3 src/script/build_topology.py
```

同时保存 JSON 审计报告：

```bash
python3 src/script/build_topology.py --report /tmp/topology-report.json
```

重点查看：

- 节点数和候选/最终边数；
- 孤立节点或度数不足节点；
- articulation points（割点）和 bridges（桥）；
- 被占用、净空不足、距离过长而拒绝的边；
- 地图/点位指纹。

生成后启动或调用 `/anav/reload_topology`，再在 RViz 检查
`/topology_markers`。修改 `--robot-length`、`--robot-width`、
`--footprint-padding` 或 `--minimum-clearance` 前，应和物理 footprint、安全 padding
保持一致。

## 地图重建后的点位迁移（⚠️ 尚未验证！！！）

> ⚠️ **尚未完成实机验证！！！**
>
> 当前流程和工具输出只能用于测试，不能直接替换生产点位。使用前必须备份原始点位，
> 并在 RViz 和现场逐点复核迁移结果。

工具默认使用：

```text
~/maps/transfer/
├── map_old/map.yaml
├── map_new/map.yaml
└── robot_positions.txt
```

运行：

```bash
python3 ~/catkin_ws/src/script/transfer_robot_positions.py
```

输出默认为 `robot_positions_transformed.txt`。工具通过新旧 PGM 特征匹配估计刚体
变换，并检查内点、误差、尺度和点位是否在地图内。不要直接覆盖生产点位；先在 RViz
逐点核对，再替换并重新构建拓扑。仅在明确理解风险时使用 `--force` 或
`--allow-outside-map`。

## 循环任务

GUI“循环任务”页从当前 `topology.yaml` 中选择两个不同点，持续往返直到人工停止。

- 开始前确认 AutoNAV 和取消服务可用。
- 固定路线模式适合必须沿指定通道行驶的任务。
- 自动绕路模式允许堵塞后尝试其他拓扑边。
- 停止循环会取消当前目标并请求停车回正。
- 循环任务失败后不会无条件继续下一段，应先查看失败原因。

## Tag 精调

启动：

```bash
roslaunch robot_r 6tagReadAndCtl_mm3v.launch
python3 ~/catkin_ws/src/script/tcpserver.py
```

常见覆盖参数：

```bash
roslaunch robot_r 6tagReadAndCtl_mm3v.launch \
  port:=/dev/ttyUSB0 \
  udp_feedback_host:=192.168.3.17 \
  x_sign:=1.0 y_sign:=1.0 yaw_sign:=-1.0
```

精调前验证：

- `/tag_position` 的单位和正负方向；
- Tag 丢失时是否在设定超时后安全退出；
- `xp_bias`、`yp_bias`、`angle_bias`；
- `/cmd_vel/tag` 速度是否落在仲裁器和碰撞监控上限内；
- 从导航 profile 切换到 Tag profile 时机器人已稳定停止。

不要在贴近工位时通过缩小 padding 来掩盖坐标标定错误。

## 导航参数

GUI“导航参数”页可以读取、应用和保存 AutoNAV 参数。保存位置：
`~/maps/autonav_params.yaml`。

主要参数：

| 参数 | 含义 |
|---|---|
| `blocked_timeout` | 判定路径持续堵塞的时间 |
| `blocked_cooldown_initial/max` | 堵塞边初始/最大封锁时间 |
| `blocked_backoff_factor` | 重复堵塞时退避倍率 |
| `blocked_wait_timeout` | 等待临时路径恢复的总时限 |
| `goal_timeout` | 单个目标执行时限 |
| `block_bidirectional` | 堵塞一条边时是否同时封锁反向边 |
| `waypoint_reached_distance` | 中间拓扑点到达距离 |
| `fixed_route_final_xy_tolerance` | 固定路线最终点位置容差 |

默认堵塞冷却时间按 `60 → 120 → 180` 秒退避，无替代路线最长等待 240 秒。临时
封锁到期后，曾失败的边仍以失败次数作为首要选路代价；存在未失败绕行路线时不会立即
切回短边，只有没有干净替代路线时才会重试。若 `~/maps/autonav_params.yaml` 已存在，
文件中的值优先于程序默认值，应在 GUI 中应用并保存新参数。

正在执行任务时不要随意修改关键阈值。GUI 会显示参数是否立即应用以及服务返回信息。

## 软件急停和故障处置

GUI 软件急停持续发布 `/cmd_vel/safety` 零速度；仲裁器将其作为最高优先级来源。

遇到异常运动：

1. 优先按物理急停。
2. 确认现场安全后再检查软件。
3. 查看 `/cmd_vel/safety`、`/cmd_vel/candidate`、`/cmd_vel`。
4. 查看 `/collision_monitor/status` 的 state、reason、source 和 collision time。
5. 检查 `/odom`、TF、静态地图和局部代价地图是否新鲜。
6. 不要在原因未明时通过关闭碰撞监控继续运行。

解除 GUI 软件急停后系统保持暂停，不会自动恢复旧任务；需要人工确认后重新继续。

## 排障命令

```bash
# 节点和接口
rosnode list
rosservice list
rostopic list

# 传感器和定位
rostopic hz /livox/lidar
rostopic hz /livox/imu
rostopic hz /Odometry
rostopic hz /odom

# 速度链路
rostopic echo /cmd_vel/nav
rostopic echo /cmd_vel/candidate
rostopic echo /cmd_vel
rostopic echo /collision_monitor/status

# TF
rosrun tf tf_echo map base_link

# CAN
ip -details link show can0
candump can0
```

### 有规划、无运动

依次确认 `/cmd_vel/nav` 是否有输出、仲裁器是否选择 nav、碰撞监控是否因数据未就绪
或碰撞输出零速度、底盘是否收到 `/cmd_vel`。

### 碰撞监控一直停车

查看原因码，不要直接调低阈值。常见原因是 TF/odom/局部地图超时、机器人初始姿态已
与障碍重叠、footprint/padding 错误或定位跳变。

### 定位和底盘速度看起来矛盾

`/Odometry` 是 LIO 全局定位，`/odom` 是底盘实测运动。检查 GUI 使用的是哪个字段，
并确认 `/motion_state` 对应当前双阿克曼、蟹行、原地旋转或驻车模式。

### 地图能加载但 AutoNAV 拒绝拓扑

运行 `build_topology.py --audit`。如果地图、PGM 或点位指纹变化，重新生成拓扑并调用
reload 服务。
