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
- `ROS_MASTER_URI`、Tag TCP/UDP IP 与目标机器网络一致。

### 机械和现场

- 机器人 footprint、载荷外形和参数一致。
- 轮组可以正常转向、回正，底盘无故障码。
- 物理急停有效且操作人员能立即触达。
- 第一次运行使用低速、封闭区域，并清空机器人周围人员和移动物体。

### 当前需要特别留意的固定配置

仓库仍包含需要按部署环境确认的固定配置：

- 源码或 launch 中的固定绝对地图路径，应统一参数化为 `~/maps`；
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

## GUI 界面与使用方法

### 先看状态，再点按钮

窗口顶部显示 ROS、定位和导航状态，底部状态栏显示最近一次操作结果。右上角
“状态中心”用于查看按时间记录的告警和异常；启动失败时还应同时查看 GUI 打开的
终端标签页，因为 roslaunch、驱动和规划器的完整日志都在那里。

GUI 共包含六个页面：

| 页面 | 主要用途 |
|---|---|
| 导航作业 | 启停定位、MoveBase、AutoNAV，下发目标、暂停、取消、急停和 Tag 微调 |
| 循环任务 | 选择两个拓扑点并持续往返 |
| 导航参数 | 设置路线策略、堵塞重规划和到达判定参数 |
| 点位管理 | 实车/RViz 采点，编辑、排序、导入和导出点位 |
| 地图构建 | 三维建图以及生成二维占据地图 |
| 设备与工具 | 启动 CAN、底盘、手柄、Tag，查看运行状态并构建拓扑 |

状态显示“命令已执行”只代表进程或命令已经发起，不代表硬件已经可用。继续下一步前，
应等待相应状态变为正常，并检查实时数据是否持续刷新。

### “设备与工具”页面

推荐按以下顺序使用：

1. 点击“启动 CAN”，等待 CAN 状态正常。该按钮当前依赖目标机器本地配置的 sudo
   密码，部署方法见[新机器最小执行方案](最小执行方案.md)第 12 节。
2. 点击“启动底盘”。它会启动 Ranger 驱动，同时带起速度仲裁器和碰撞监控。
3. 如需 Tag 工位精调，点击“启动标签读取”；GUI 会同时启动 MM3V ROS 节点和 TCP
   服务。
4. 只有确实需要 PXN 手柄遥控时才点击“启动手柄”。手柄和 GUI 会复用 `/joy`，启用
   前必须确认按键映射，避免 GUI 控制消息被解释为遥控命令。

“实时运行状态”会显示 CAN、底盘、速度仲裁、定位、MoveBase、AutoNAV 和标签读取。
“实时数据”用于核对机器人位姿与最新 Tag 数据。“构建导航拓扑”根据当前
`robot_positions.txt` 和 `map.yaml` 生成 `~/maps/topology.yaml`；点位或地图变化后
必须重新执行。

### “导航作业”页面

#### 启动定位和导航

1. 点击“启动定位”。GUI 启动 `3startlocation.launch`，并在 RViz 尚未运行时自动打开
   RViz。
2. 等待定位数据持续更新，在 RViz 确认地图、点云、机器人姿态和 TF 对齐。
3. 点击“启动自动导航”。如果 MoveBase 尚未运行，GUI 会先自动启动 MoveBase，等待
   就绪后再启动 AutoNAV；“启动 MoveBase”按钮主要用于需要单独调试规划器的情况。
4. AutoNAV 就绪后，工位快捷按钮和“点位管理”中的“导航到选中点”才可用。

“停止定位”只停止定位。“取消当前任务”会取消正在执行的 AutoNAV 目标并请求停车
回正，但保留 MoveBase 待命；“停止导航”会停止 AutoNAV、MoveBase 和相关日志进程。
日常取消单个目标使用“取消当前任务”，结束整套导航时才使用“停止导航”。

#### 下发目标和任务控制

- “导航参数”页的“普通导航”策略用于下一次目标下发：自动绕路对应 `run=1`，固定
  路线对应 `run=2`。
- `W1`、`W2` 等工位来自点位文件的 label，AutoNAV 启动后会动态显示为快捷按钮。
- 普通 `P0`、`P1` 等导航点从“点位管理”页选中后下发，也可以双击对应行。
- 点击目标按钮后会立即调用 `/plan_path_and_go` 并执行，不需要再点击“开始任务”。
- “开始任务”保留给顺序点位任务的兼容入口；“暂停任务”会停止当前目标推进，
  “继续任务”只恢复已经暂停的任务，不会自动创建新目标。
- “取消当前任务”是结束当前任务的推荐方式；任务结束后如果提示轮组回正未确认，
  不要立即下发下一目标。

#### 软件急停

“紧急停止”会持续发布最高优先级的 safety 零速度。按下后先处理现场风险；确认安全
后再点击“解除急停”。解除只移除软件急停状态，旧任务不会自动恢复，需要操作员重新
点击“继续任务”或重新下发目标。软件急停不能替代底盘物理急停。

#### Tag 精细调整

使用前先在“设备与工具”启动标签读取，并确认“最新标签定位”持续有效。机器人应先
通过 AutoNAV 到达工位附近并稳定停止，然后在“精细调整”中填写目标 X、Y（米）和
角度（度），点击“发送目标”。

Y 快捷值按钮会把该值写入 Y，并**立即发送**当前 X、Y、角度组合，不只是填充输入框。
首次使用必须先确认坐标正负方向、单位和当前 X/角度输入，建议从小偏移开始验证。

### “循环任务”页面

1. 点击“刷新 topo 点位”，确认至少有两个有效点。
2. 选择不同的循环点 A、B。
3. 选择“固定路线”或“自动绕路”。
4. 点击“开始循环”。如果导航服务尚未运行，GUI 会尝试启动 MoveBase 和 AutoNAV，
   最长等待约 40 秒。
5. 观察“正在前往”和“已完成单程”计数；一次 A→B 或 B→A 计为一个单程。
6. 点击“停止循环”会取消当前目标并结束往返。

任意一段失败后循环会终止，不会无条件继续下一段。应先处理状态栏或状态中心显示的
失败原因，再重新开始。

### “导航参数”页面

- “普通导航”策略立即用于下一次目标，不改变正在运行的目标。
- “读取当前节点”从正在运行的 AutoNAV 读取实际参数；AutoNAV 未启动时该服务不可用。
- “应用到当前节点”修改本次正在运行的 AutoNAV 参数，但不会写入默认配置文件。
- “保存为启动默认值”写入 `~/maps/autonav_params.yaml`，供下次启动 AutoNAV 时加载，
  不等同于立即应用到当前节点。

修改参数前先读取当前节点；需要立即生效并在重启后保留时，应依次“应用到当前节点”
和“保存为启动默认值”。任务执行中不要修改判堵时间、终点容差等关键参数。

### “点位管理”页面

点位采集有两种方式：

- 实车采点：移动机器人到目标姿态，再点击“添加导航点”或“添加工位”。
- RViz 采点：在 RViz 中选择“记录点位”工具（快捷键 `R`），在地图上拖出位置和
  朝向，再回到 GUI 保存为导航点或工位。RViz 的 Fixed Frame 必须是 `map`。

开始采集前必须区分两个按钮：

- “进入点位设置”：备份现有 `robot_positions.txt` 后开始一份全新的点位文件；
- “继续添加点位”：备份并保留已有点位，在文件末尾继续添加。

采集完成后点击“结束点位设置”。RViz 采点时如果 MoveBase 正在运行，GUI 会拒绝进入
设置模式，避免记录箭头被导航插件误当成运动目标。

列表支持刷新、导航、修改、删除、上下移动、撤销、恢复最近备份、导入和导出。修改
点位顺序会改变普通导航点的节点 ID；工位必须保持唯一的 `W1`、`W2` 等名称。任何
新增、删除、修改、排序或导入完成后，都要在“设备与工具”重新构建导航拓扑。

### “地图构建”页面

建图前先停止定位、MoveBase 和 AutoNAV，避免同时启动多套雷达/LIO 处理链：

1. 点击“开始构建地图”，缓慢控制机器人覆盖场地并形成闭环。
2. 点击“结束地图构建”正常停止，让 LIO 执行 PCD 和特征地图保存逻辑。
3. 确认三维地图文件生成后，点击“生成并保存二维地图”。
4. 等待 `~/maps/map.yaml` 和 `map.pgm` 生成，再点击停止或按页面提示结束。
5. 在 RViz 核对二维地图后重新采集/迁移点位，并重新构建拓扑。

不要通过关闭终端、`kill -9` 或直接断电结束建图，否则地图保存可能不完整。

### 正常退出 GUI

完成作业后先取消当前任务，停止导航和定位，再点击“退出控制台”。关闭窗口也会尝试
发布软件停车、请求轮组回正，并停止由 GUI 启动的子进程；手工从其他终端启动的进程
不一定属于 GUI 管理范围，退出后仍需用 `rosnode list` 确认没有残留运动节点。

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
| `controller_handoff_distance` | 首尾控制模式交接时的 base_link 到点距离 |
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
