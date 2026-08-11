# Collision Monitor

Ranger 底盘的统一碰撞监控节点。

它位于速度仲裁器和底盘之间，统一保护导航、Tag 精调和遥控命令，
不依赖上层当前正在执行 B 样条跟踪还是原地旋转。

```text
导航 / Tag / 遥控
        │
        ▼
cmd_vel_arbiter
        │  /cmd_vel/candidate
        ▼
collision_monitor
        │  /cmd_vel
        ▼
Ranger 底盘
```

节点以 `map -> base_link` 的真实位姿和 `/odom` 实测速度为预测起点，
将带安全裕度的 footprint 沿未来轨迹逐点放置，同时检查静态地图和
local costmap。

运行频率为 50 Hz。最终可以输出：

- 100% 原始候选速度；
- 75%、50% 或 25% 的候选速度；
- 零速度。

## 状态写在哪里

状态目前没有定义成单独的 C++ `enum`，而是字符串，主要集中在：

```text
src/collision_monitor_node.cpp
```

具体位置：

- `TimerCallback()`：状态决策及主要原因码；
- `DataReady()`、`GetRobotPose()`：数据和 TF 原因码；
- `SupportedCommand()`：候选命令检查；
- `SupportedMeasuredMotion()`：实测运动检查；
- `PublishStatus()`：诊断等级和字段；
- `PublishMarkers()`：RViz 显示。

`src/collision_checker.cpp` 负责 footprint 栅格碰撞检查和运动仿真，
不生成对外的状态字符串。

## 状态速查表

| 状态 | 等级 | 速度输出 | 作用 |
| --- | --- | --- | --- |
| `OK` | OK | 100% | 完整轨迹安全 |
| `STOP_COMMAND` | OK | 零速度 | 上层主动停车 |
| `SLOWDOWN` | WARN | 25%～75% | 遇障减速或逐级恢复 |
| `MANUAL_BYPASS` | WARN | 100% | 数据缺失时遥控旁路 |
| `PROFILE_TRANSITION` | WARN | 零速度 | 等待来源安全切换 |
| `STOPPED` | ERROR | 零速度 | 候选命令异常 |
| `DATA_NOT_READY` | ERROR | 零速度 | 监控数据不完整 |
| `UNSUPPORTED_MOTION` | ERROR | 零速度 | 命令或实测运动不支持 |
| `COLLISION_STOP` | ERROR | 零速度 | 没有安全的非零速度 |
| `EMERGENCY_STOP` | ERROR | 零速度 | 停车轨迹也会碰撞 |

诊断等级对应：

```text
OK = 0    WARN = 1    ERROR = 2
```

## 状态详细说明

### `OK`

- 输出：100% 候选速度；
- 原因：`FULL_COMMAND_SAFE`；
- 含义：从当前真实状态开始，完整预测轨迹安全。

### `STOP_COMMAND`

- 输出：零速度；
- 原因：`ZERO_COMMAND_PASSED`；
- 含义：上层本来就要求停车。

零速度命令直接放行，不要求地图、定位或 local costmap 先有效。

### `SLOWDOWN`

- 输出：25%、50% 或 75% 候选速度；
- `PATH_CLEARANCE`：原速度不安全，但较低速度安全；
- `RECOVERY_RAMP`：环境恢复安全，正在逐级升速。

线速度和角速度使用同一个缩放比例，以保持运动曲率。

降速立即生效。恢复时每经过连续 0.50 s 安全，最多上升一档。

### `MANUAL_BYPASS`

- 输出：100% 遥控命令；
- 只允许来源为 `teleop`；
- 触发原因是 footprint、odom、地图、local costmap 或 TF 不可用。

旁路期间没有 footprint 碰撞保护，需要操作人员负责安全。

`nav` 和 `tag` 不允许进入该状态，数据缺失时必须停车。

### `PROFILE_TRANSITION`

- 输出：零速度；
- `WAITING_FOR_ROBOT_TO_STOP`：底盘还没有停稳；
- `WAITING_FOR_STATIONARY_HOLD`：已经停稳，正在等待保持时间。

例如从 `nav` 切换到 `tag` 时，不能在车辆仍运动时直接把 padding
从 0.15 m 缩小到 0.08 m。

节点会先使用导航的保守 footprint 检查停车轨迹。实测速度连续低于
阈值 0.20 s 后，才启用新的 Tag profile。

### `STOPPED`

- 输出：零速度；
- `CANDIDATE_STALE`：候选命令未收到或超过 0.25 s；
- `CANDIDATE_NON_FINITE`：候选 Twist 中含有 NaN 或 Inf。

### `DATA_NOT_READY`

- 输出：零速度；
- 适用于 `nav` 和 `tag`；
- 表示碰撞监控所需数据不完整或不可信。

具体原因见“数据原因码”。

### `UNSUPPORTED_MOTION`

- 输出：零速度；
- 命令来源未知；
- 命令使用了不支持的自由度；
- 命令超过当前 profile 上限；
- odom 显示了当前 profile 不允许的运动。

### `COLLISION_STOP`

- 输出：零速度；
- 当前立即停车轨迹尚未构成紧急碰撞；
- 但是 100%、75%、50%、25% 都没有可安全执行的速度；
- 或障碍刚清除，仍处于 0.50 s 安全恢复保持期。

### `EMERGENCY_STOP`

- 输出：零速度；
- 表示即使现在立刻发送零速度；
- 按反应时间和制动能力预测的停车轨迹仍然会发生碰撞。

这是 Collision Monitor 的最高优先级停车状态。

## 全部原因码

### 原因码速查表

| 类别 | 原因码 | 简要含义 |
| --- | --- | --- |
| 候选命令 | `CANDIDATE_STALE` | 候选命令缺失或超时 |
| 候选命令 | `CANDIDATE_NON_FINITE` | 候选命令含 NaN/Inf |
| 候选命令 | `ZERO_COMMAND_PASSED` | 上层零速度直接放行 |
| 正常运行 | `FULL_COMMAND_SAFE` | 100% 速度预测安全 |
| 减速恢复 | `PATH_CLEARANCE` | 需要降低速度才能安全 |
| 减速恢复 | `RECOVERY_RAMP` | 正在逐档恢复速度 |
| 碰撞停车 | `STATIC_PATH_BLOCKED` | 静态地图阻塞预测路径 |
| 碰撞停车 | `LOCAL_PATH_BLOCKED` | 局部障碍阻塞预测路径 |
| 碰撞停车 | `NO_SAFE_SPEED_LEVEL` | 没有安全的非零速度档 |
| 碰撞停车 | `CLEAR_HOLD_OR_NO_SAFE_LEVEL` | 等待恢复或仍无安全档 |
| 数据检查 | `FOOTPRINT_NOT_READY` | Footprint 没有准备好 |
| 数据检查 | `ODOM_STALE` | Odom 缺失或超时 |
| 数据检查 | `ODOM_NON_FINITE` | Odom 速度含 NaN/Inf |
| 数据检查 | `STATIC_MAP_NOT_READY` | 静态地图未准备好 |
| 数据检查 | `LOCAL_MAP_NOT_READY` | Local costmap 未准备好 |
| TF 检查 | `TF_UNAVAILABLE: ...` | 无法取得当前机器人位姿 |
| TF 检查 | `TF_HAS_ZERO_STAMP` | TF 时间戳为零 |
| TF 检查 | `TF_STALE` | TF 已过期或时间异常 |
| TF 检查 | `TF_NON_FINITE` | TF 位姿数值无效 |
| TF 检查 | `TF_JUMP` | 定位位姿发生较大跳变 |
| 运动检查 | `UNKNOWN_COMMAND_SOURCE` | 命令来源不是已知 profile |
| 运动检查 | `NON_FINITE_COMMAND` | Profile 检查到命令数值无效 |
| 运动检查 | `UNSUPPORTED_COMMAND_DOF` | 命令使用了不支持的自由度 |
| 运动检查 | `COMMAND_LIMIT_EXCEEDED` | 命令超过当前 profile 上限 |
| 运动检查 | `NON_FINITE_MEASURED_MOTION` | 实测速度数值无效 |
| 运动检查 | `UNSUPPORTED_MEASURED_DOF` | 实测运动不符合当前 profile |
| 来源切换 | `WAITING_FOR_ROBOT_TO_STOP` | 等待底盘停止 |
| 来源切换 | `WAITING_FOR_STATIONARY_HOLD` | 等待连续静止保持时间 |
| 紧急停车 | `STATIC_COLLISION_DURING_PROFILE_TRANSITION` | 切换停车轨迹碰静态地图 |
| 紧急停车 | `LOCAL_COLLISION_DURING_PROFILE_TRANSITION` | 切换停车轨迹碰局部障碍 |
| 紧急停车 | `STATIC_COLLISION_IN_STOPPING_PATH` | 立即停车仍会碰静态地图 |
| 紧急停车 | `LOCAL_COLLISION_IN_STOPPING_PATH` | 立即停车仍会碰局部障碍 |

下面按照类别具体说明各原因码的触发条件。

### 候选命令

- `CANDIDATE_STALE`
  
  未收到候选命令，或者接收时间超过 0.25 s。

- `CANDIDATE_NON_FINITE`
  
  候选 Twist 包含 NaN 或 Inf。

- `ZERO_COMMAND_PASSED`
  
  候选速度所有分量均小于 `zero_epsilon`，直接输出零速度。

- `FULL_COMMAND_SAFE`
  
  100% 候选速度的完整 swept volume 安全。

### 减速和停车

- `PATH_CLEARANCE`
  
  100% 速度不安全，但至少有一个较低速度等级安全。

- `RECOVERY_RAMP`
  
  环境允许更高速度，但节点正在按 0.25 一档逐级恢复。

- `STATIC_PATH_BLOCKED`
  
  完整轨迹会碰到静态 `/map`，并且没有安全的非零速度等级。

- `LOCAL_PATH_BLOCKED`
  
  完整轨迹会碰到 local costmap，并且没有安全的非零速度等级。

- `NO_SAFE_SPEED_LEVEL`
  
  1.0、0.75、0.50、0.25 四档都不安全，但没有得到可归类的
  完整轨迹首碰撞原因。正常情况下较少出现。

- `CLEAR_HOLD_OR_NO_SAFE_LEVEL`
  
  当前仍保持零比例。可能还没有连续安全 0.50 s，也可能仍然没有
  安全的非零档位。

### 数据原因码

- `FOOTPRINT_NOT_READY`
  
  三个 profile 的 footprint 没有正确加载。

- `ODOM_STALE`
  
  没有收到 `/odom`，或接收时间超过 0.20 s。

- `ODOM_NON_FINITE`
  
  `/odom.twist.twist` 包含 NaN 或 Inf。

- `STATIC_MAP_NOT_READY`
  
  `/map` 没有收到或格式无效。静态地图收到后不做周期超时。

- `LOCAL_MAP_NOT_READY`
  
  local costmap 没有收到、超过 0.30 s 或格式无效。

- `TF_UNAVAILABLE: ...`
  
  无法查询 `map -> base_link`。冒号后面是 TF2 的具体错误。

- `TF_HAS_ZERO_STAMP`
  
  TF 时间戳为零，无法判断定位数据的新鲜度。

- `TF_STALE`
  
  TF 超过 0.30 s、时间差无效，或时间戳明显来自未来。

- `TF_NON_FINITE`
  
  TF 位置、yaw 或四元数无效。

- `TF_JUMP`
  
  相邻 TF 跳变超过 0.50 m 或 0.80 rad，默认停车保持 0.50 s。

### 命令和实测运动

- `UNKNOWN_COMMAND_SOURCE`
  
  `source` 不是配置的 `nav`、`tag` 或 `teleop`。

- `NON_FINITE_COMMAND`
  
  profile 内部检查到命令包含 NaN 或 Inf。这是第二层防御，通常会先被
  `CANDIDATE_NON_FINITE` 截住。

- `UNSUPPORTED_COMMAND_DOF`
  
  命令使用了当前 profile 不允许的自由度，例如导航倒车、横移，或者
  `linear.z`、`angular.x`、`angular.y` 非零。

- `COMMAND_LIMIT_EXCEEDED`
  
  候选命令超过当前来源的线速度或角速度上限。

- `NON_FINITE_MEASURED_MOTION`
  
  odom 实测速度含有 NaN 或 Inf。这也是第二层防御。

- `UNSUPPORTED_MEASURED_DOF`
  
  odom 显示当前底盘仍有该 profile 不允许的运动。例如 `nav` 状态下
  实测倒车或横移超过阈值。

### 来源切换和紧急停车

- `WAITING_FOR_ROBOT_TO_STOP`
  
  命令来源已经变化，但 odom 实测速度仍高于静止阈值。

- `WAITING_FOR_STATIONARY_HOLD`
  
  实测速度已经低于阈值，正在等待连续静止 0.20 s。

- `STATIC_COLLISION_DURING_PROFILE_TRANSITION`
  
  来源切换的停车轨迹会碰到静态地图。

- `LOCAL_COLLISION_DURING_PROFILE_TRANSITION`
  
  来源切换的停车轨迹会碰到 local costmap。

- `STATIC_COLLISION_IN_STOPPING_PATH`
  
  当前 profile 下立即停车，swept volume 仍会碰到静态地图。

- `LOCAL_COLLISION_IN_STOPPING_PATH`
  
  当前 profile 下立即停车，swept volume 仍会碰到 local costmap。

## 输入

| 输入 | 类型 | 用途 |
| --- | --- | --- |
| `/cmd_vel/candidate` | `ArbitratedCommand` | 候选速度和来源 |
| `/odom` | `Odometry` | 实测初始速度 |
| `/map` | `OccupancyGrid` | 静态障碍 |
| local costmap | `OccupancyGrid` | 当前传感器障碍 |
| `map -> base_link` | TF2 | 当前真实位姿 |

默认 local costmap 话题：

```text
/mxb_move_base/local_costmap/costmap
```

静态 `/map` 的处理规则：

- 占用值达到 100：碰撞；
- 未知栅格：碰撞；
- 地图外：碰撞。

local costmap 的处理规则：

- 只把值达到 100 的致命栅格视为碰撞；
- 未知栅格忽略；
- 地图外忽略。

## 输出

### 底盘速度 `/cmd_vel`

消息类型为 `geometry_msgs/Twist`。

Collision Monitor 应是 `/cmd_vel` 的唯一发布者。

| 字段 | 单位 | 含义 |
| --- | --- | --- |
| `linear.x` | m/s | 前后速度 |
| `linear.y` | m/s | Tag 横向精调速度 |
| `angular.z` | rad/s | yaw 角速度 |
| 其他分量 | — | V1 始终输出零 |

### 诊断 `/collision_monitor/status`

消息类型为 `diagnostic_msgs/DiagnosticArray`。

其中包含一个名为 `collision_monitor` 的 `DiagnosticStatus`。

#### 标准字段

| 字段 | 内容 |
| --- | --- |
| `header.stamp` | 本次诊断时间 |
| `status.name` | `collision_monitor` |
| `status.hardware_id` | `ranger` |
| `status.level` | 0、1 或 2 |
| `status.message` | `state: reason` |

#### values 中的字段

| 键 | 含义 |
| --- | --- |
| `state` | 当前监控状态 |
| `reason` | 状态原因 |
| `source` | `nav`、`tag` 或 `teleop` |
| `profile` | 实际使用的运动 profile |
| `footprint_padding` | 当前安全裕度，单位 m |
| `scale` | 候选速度缩放比例 |
| `collision_time` | 预计首次碰撞时间，单位 s |
| `output_linear_x` | 实际输出 x 线速度 |
| `output_linear_y` | 实际输出 y 线速度 |
| `output_angular_z` | 实际输出 yaw 角速度 |
| `measured_linear_x` | odom 实测 x 线速度 |
| `measured_linear_y` | odom 实测 y 线速度 |
| `measured_angular_z` | odom 实测 yaw 角速度 |

特殊值说明：

- `collision_time = -1`：本周期没有预计碰撞时间；
- `collision_time = 0`：当前姿态或第一个采样点已经碰撞；
- `collision_time > 0`：预计经过该时间后首次碰撞；
- 没有 odom 时，实测速度字段输出字符串 `-1`；
- 没有有效 profile 时，`footprint_padding` 输出字符串 `-1`。

查看完整诊断：

```bash
rostopic echo /collision_monitor/status
```

### RViz `/collision_monitor/markers`

消息类型为 `visualization_msgs/MarkerArray`，坐标系为 `map`。

| Marker namespace | 显示内容 |
| --- | --- |
| `current_padded_footprint` | 当前黄色 footprint |
| `predicted_path` | 机器人中心预测轨迹 |
| `predicted_footprints` | 轨迹上的蓝色 footprint |
| `collision` | 首个碰撞位置红球 |

## 控制台日志

典型日志：

```text
collision_monitor: state=COLLISION_STOP source=nav \
reason=STATIC_PATH_BLOCKED scale=0.00 ttc=0.420
```

| 参数 | 含义 |
| --- | --- |
| `state` | 当前状态 |
| `source` | 候选速度来源 |
| `reason` | 状态原因 |
| `scale` | 速度缩放比例 |
| `ttc` | 预计首次碰撞时间，单位 s |

这些警告使用 1 s 节流输出，不会按 50 Hz 全部打印。

### 常见日志解释

`STATIC_COLLISION_DURING_PROFILE_TRANSITION, ttc=0.000`

: 来源切换时使用导航的保守 footprint，当前 footprint 已经与静态地图
  占用格、未知格或地图边界相交。

`STATIC_COLLISION_IN_STOPPING_PATH, ttc=0.000`

: 立即停车预测从第一个姿态就判定碰撞。应在 RViz 检查黄色 footprint、
  红球和 `/map` 是否重合。

`STATIC_PATH_BLOCKED, ttc=0.420`

: 100% 命令约 0.42 s 后碰撞，而且 75%、50%、25% 也没有安全的完整
  停车轨迹，因此输出零速度。

`CLEAR_HOLD_OR_NO_SAFE_LEVEL`

: 障碍可能刚清除但还没有连续安全 0.50 s，也可能所有非零档仍不安全。

`UNSUPPORTED_MEASURED_DOF`

: 应检查 `/odom.twist.twist`，不是底盘驱动单独打印的 steering angle。
  重点检查 `linear.x` 是否为负、`linear.y` 是否非零，以及
  `linear.z/angular.x/angular.y` 是否超过阈值。

## Footprint 与运动 Profile

物理 footprint 位于：

```text
config/robot_footprint.yaml
```

当前配置：

```yaml
footprint:
  - [ 0.36,  0.25]
  - [ 0.36, -0.25]
  - [-0.36, -0.25]
  - [-0.36,  0.25]
```

以 `base_link` 为中心，物理车体尺寸为：

```text
长 0.72 m × 宽 0.50 m
```

### Profile 速查

| Profile | Padding | 最大 vx | 最大 vy | 最大 wz |
| --- | ---: | ---: | ---: | ---: |
| `nav` | 0.15 m | 0.20 m/s | 0 | 0.50 rad/s |
| `tag` | 0.08 m | 0.05 m/s | 0.05 m/s | 0.15 rad/s |
| `teleop` | 0.15 m | 0.35 m/s | 0 | 1.10 rad/s |

加 padding 后的近似外包尺寸：

- `nav`：1.02 m × 0.80 m；
- `tag`：0.88 m × 0.66 m；
- `teleop`：1.02 m × 0.80 m。

运动限制：

- `nav`：允许前进、前进圆弧和原地旋转，不允许倒车和横移；
- `tag`：允许低速前后、横移和旋转精调；
- `teleop`：当前配置不允许倒车和横移；数据缺失时允许人工旁路。

Profile 参数位于：

```text
config/collision_monitor.yaml
```

## 构建与启动

```bash
catkin_make
source devel/setup.bash
roslaunch collision_monitor collision_monitor.launch
```

启动后确认 `/cmd_vel` 只有 Collision Monitor 一个发布者：

```bash
rostopic info /cmd_vel
```
