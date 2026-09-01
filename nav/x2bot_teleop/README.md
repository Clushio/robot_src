# x2bot_teleop：AutoNAV、点位与 Tag 控制

这个包虽然沿用了 `teleop` 名称，但当前同时承载四类项目业务：

- 拓扑自动导航 `runnav`；
- 点位记录 `saveLocation`；
- MM3V Tag 工位精调 `TagCtl`；
- PXN 手柄到速度命令的转换 `x2bot_joy_PXN`。

系统总览见[架构文档](../../docs/ARCHITECTURE.md)。

## 可执行程序

| 程序 | 源码 | 作用 |
|---|---|---|
| `runnav` | `src/run_nav.cpp` | 读取点位/拓扑、拓扑选路、调用 MoveBase、堵塞恢复和任务状态 |
| `saveLocation` | `src/save_Location.cpp` | 根据 `/joy` 指令把当前位姿写入点位文件 |
| `TagCtl` | `src/TagCtl_srv.cpp` | 根据 `/tag_position` 和目标偏差发布 `/cmd_vel/tag` |
| `x2bot_joy_PXN` | `src/x2bot_joy_PXN.cpp` | 把 PXN 手柄转换为 `/cmd_vel/teleop` |

## AutoNAV 输入文件

默认数据目录是 `~/maps`。部署前应确认相关 ROS 参数和代码路径都解析到当前账号的
该目录。

```text
map.yaml + 对应 map.pgm
robot_positions.txt
topology.yaml
autonav_params.yaml（可选）
```

`robot_positions.txt` 每个有效行：

```text
x y z roll pitch yaw [label]
```

- 节点 ID 从 0 开始，按有效行顺序分配。
- 普通导航点通常没有 label。
- 工位用唯一的 `W1`、`W2` 等 label；服务中用负数请求，例如 `-1` 表示 W1。
- 行顺序不等价于路线顺序，边必须来自经过安全检查的 `topology.yaml`。

## AutoNAV 接口

| 接口 | 类型 | 说明 |
|---|---|---|
| `/plan_path_and_go` | `x2bot_teleop/SetInt` | 规划或执行到目标点/工位的拓扑路径 |
| `/anav/cancel_navigation` | `std_srvs/Trigger` | 取消当前任务 |
| `/anav/reload_topology` | `std_srvs/Trigger` | 重新加载并验证拓扑 |
| `/anav/nav_config` | `x2bot_teleop/NavConfig` | 读取或应用运行参数 |
| `/anav/task_status` | `std_msgs/String` | GUI 使用的任务状态 |
| `/anav/fixed_route_mode` | `std_msgs/Bool` | 通知局部规划器当前是否固定路线 |
| `/anav/topology_safety_phase` | `std_msgs/UInt8` | 10 Hz 发布拓扑首段、中间段、末段安全阶段 |
| `/anav/terminal_motion_state` | `std_msgs/UInt8` | 局部控制器发布直线跟踪、终点位置捕获、最终旋转和完成状态 |
| `/anav/path_control_mode` | `std_msgs/UInt8` | 当前路径控制模式：0 未确定、1 B 样条参考跟踪、2 老控制器/回退模式 |
| `/topology_plan` | `nav_msgs/Path` | 当前拓扑路径 |
| `/topology_markers` | `visualization_msgs/MarkerArray` | RViz 拓扑显示 |
| `/cmd_vel/nav` | `geometry_msgs/Twist` | AutoNAV 直接控制阶段的导航速度输入 |

### `/plan_path_and_go`

请求字段：

- `data`：目标节点 ID；负数表示工位 label。
- `currentID`：调用方认为的当前节点；定位无法确定最近节点时作为回退。
- `run`：`0` 只规划，`1` 自动绕路执行，`2+` 固定路线执行。

示例：

```bash
# 只规划到 P3
rosservice call /plan_path_and_go '{data: 3, currentID: 0, run: 0}'

# 自动绕路执行到 P3
rosservice call /plan_path_and_go '{data: 3, currentID: 0, run: 1}'

# 固定路线执行到工位 W1
rosservice call /plan_path_and_go '{data: -1, currentID: 0, run: 2}'
```

`runnav` 会优先通过定位选择最近的有效起点。服务返回成功不应只理解为“MoveBase
action 返回 SUCCEEDED”；代码还会检查真实到点距离，并在任务结束请求底盘停车回正。

安全阶段取值为 `0=NORMAL`、`1=START_SEGMENT`、`2=FINAL_SEGMENT`。
当前位置到首点以及首点到第二点保持 `START_SEGMENT`；中间拓扑边使用
`NORMAL`；向最终目标发送最后一段时使用 `FINAL_SEGMENT`。首段完成状态会
跨重规划锁存，避免中途生成的新路径因索引重新从零开始而误用小 padding。

## 拓扑验证和堵塞策略

启动时 `runnav` 验证：

- 点位和 topology 节点是否匹配；
- 地图/PGM/点位指纹是否仍有效；
- 边引用和元数据是否完整；
- 静态地图加载是否成功。

验证失败时拒绝隐式顺序连线。应重新运行：

```bash
python3 ~/catkin_ws/src/script/build_topology.py --audit
python3 ~/catkin_ws/src/script/build_topology.py
rosservice call /anav/reload_topology
```

自动绕路模式会在持续无进展后临时封锁问题边，按指数退避设置冷却时间，并尝试其他
路径。默认封锁序列为 60、120、180 秒，无替代路线时最长等待 240 秒。封锁到期的
失败边不会立即恢复为普通短边：选路会先最小化历史失败边数量，再比较路径长度；只有
没有干净替代路线时才会重试失败边。堵塞后的重规划锚定在最后确认到达的拓扑点，避免
最近点判断绕过刚刚封锁的边。固定路线模式不会换边，而是在原路线停车等待。参数通过
GUI 或 `~/maps/autonav_params.yaml` 管理。

## 点位记录

命令行入口：

```bash
roslaunch robot_r 3settinglocation.launch
```

节点订阅 `/joy` 并把点位写入 `~/maps/robot_positions.txt`。日常更推荐 GUI：它支持
实车采点、RViz 采点、手工修改、排序、删除、导入、备份和 marker 刷新。

修改点位后必须重建拓扑。不要在 MoveBase 正在运行时使用 RViz 采点工具，以免目标
事件被其他导航插件误解。

## Tag 精调

`TagCtl` 订阅 `/tag_position`，提供 `/set_target_y` 服务并发布 `/cmd_vel/tag`。

```bash
rosservice call /set_target_y \
  '{target_x: 0.0, target_y: 0.0, target_angle: 0.0}'
```

当前控制流程分阶段修正角度、X、Y/Yaw，并要求连续稳定帧后才切换阶段或报告成功。
Tag 数据连续失效、任务超时、成功或失败后都会停止命令并请求
`/cmd_vel_arbiter/finish_motion`。

主要私有参数在 `TagCtl_srv.cpp` 中读取，可由 launch 覆盖，包括速度上限、到达阈值、
滤波、yaw PD、稳定帧数和 Tag 丢失超时。读头协议、坐标符号与偏置位于
`robot_r/launch/6tagReadAndCtl_mm3v.launch`。

## 手柄与 `/joy` 复用

GUI 也使用 `/joy` 发送“记录点位、开始、暂停、继续”等兼容指令。定位启动文件不会
自动包含 `x2bot_joy_PXN`，因为同时启动手柄转换可能把 GUI 按键误解释成遥控速度。

启用手柄前确认：

- 当前是否仍需要 GUI 的 `/joy` 控制；
- `x2bot_joy_PXN.launch` 的设备和轴映射；
- 输出是 `/cmd_vel/teleop`，不是直接 `/cmd_vel`；
- 遥控来源可能处于人工碰撞旁路。

## 编译和测试

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make -DCATKIN_WHITELIST_PACKAGES=x2bot_teleop
source devel/setup.bash
```

恢复全工作区编译时不要遗留 whitelist：

```bash
catkin_make -DCATKIN_WHITELIST_PACKAGES=""
```

修改 `SetInt.srv`、`NavConfig.srv` 或 `SetTagY.srv` 后，需要重新编译所有直接依赖
消息生成结果的目标。
