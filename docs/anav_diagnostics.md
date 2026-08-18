# ANAV 统一状态与异常诊断方案

## 1. 文档目的

本文定义移动机器人作业控制台、AutoNAV、MoveBase、标签靠站、速度仲裁、
碰撞安全层和 Ranger 底盘之间统一的异常上报、展示、恢复和留档方式。

目标不是把终端里的所有 ROS 日志搬进 GUI，而是让现场人员能够快速确认：

1. 当前有哪些尚未恢复的异常；
2. 异常何时发生、属于哪个模块、出现过几次；
3. 机器人为何不能定位、导航、靠站或运动；
4. 应先检查哪个节点、话题、文件或硬件；
5. 异常是否已恢复以及恢复时间；
6. 远程排查时如何导出结构化记录。

本文同时区分当前已经实现的范围和下一阶段仍需补充的范围。标为“待接入”的
条目，当前不能认为已经具备完整诊断能力。

## 2. 总体方案

### 2.1 数据流

```text
AutoNAV / TagCtl / cmd_vel_arbiter / collision_monitor / ranger_base
                              │
                              │ diagnostic_msgs/DiagnosticArray
                              ▼
                        /diagnostics
                              │
                              ▼
                    aNAV_ranger.py（GUI）
                              │
              ┌───────────────┴────────────────┐
              │                                │
        ROS 模块诊断                    GUI 本地监控
                                      - ROS Master
                                      - CAN 接口
                                      - 子进程退出
                                      - 节点失联
                                      - 定位数据超时
              │                                │
              └───────────────┬────────────────┘
                              ▼
                  FaultCenterDialog 状态中心
                    - 当前未恢复异常
                    - 历史与恢复记录
                    - 详细原因和处理建议
                    - JSONL 留档 / CSV 导出
```

模块内部异常必须由模块自己发布，因为模块最清楚失败原因；GUI 只补充它从外部
能够可靠判断的状态，例如进程退出、节点失联、CAN 未启用和话题长期没有更新。
不得通过匹配终端输出文字来判断模块内部故障。

### 2.2 涉及文件

| 文件 | 职责 |
| --- | --- |
| `script/fault_center.py` | 独立非模态状态窗口、事件去重、恢复、筛选、落盘和导出 |
| `script/aNAV_ranger.py` | 订阅 `/diagnostics`，执行 GUI 本地健康检查，维护主窗口状态中心入口 |
| `nav/x2bot_teleop/src/run_nav.cpp` | 地图、点位、拓扑校验以及导航任务失败诊断 |
| `nav/x2bot_teleop/src/TagCtl_srv.cpp` | 标签数据、控制占用和停车回正诊断 |
| `cmd_vel_arbiter/src/cmd_vel_arbiter_node.cpp` | 速度来源、非法速度、限幅、安全接管和回正诊断 |
| `collision_monitor/src/collision_monitor_node.cpp` | 地图、里程计、TF、障碍减速/停车和运动模式诊断 |
| `ranger_ros/ranger_base/src/ranger_messenger.cpp` | 底盘速度看门狗、底盘错误码和停车回正诊断 |

## 3. GUI 状态中心设计

### 3.1 主窗口入口

主窗口右上角保留“状态中心”按钮。

- 无未恢复异常时显示 `状态中心`，使用普通样式；
- 只有 WARN 时显示橙色，并显示 `状态中心 (N)`；
- 存在 ERROR 或 STALE 时显示红色，并显示 `状态中心 (N)`；
- `N` 是 WARN、ERROR、STALE 未恢复事件数量之和；
- 点击按钮打开状态窗口，不影响主控制窗口和机器人任务。

### 3.2 独立窗口

状态中心使用独立的非模态窗口。用户可以一边查看异常，一边操作主 GUI；关闭
按钮只隐藏窗口，不退出 GUI，也不会清除记录。

当前窗口包含：

- 级别筛选：恢复/OK、WARN、ERROR、STALE；
- 汇总：分别显示 WARN、ERROR、STALE 的未恢复数量；
- 事件表：时间、级别、错误码、模块、描述、状态、发生次数；
- 详情区：首次发生时间、最后更新时间、详细原因、处理建议和原始诊断字段；
- “清除已恢复记录”：只清除本次 GUI 内存中的已恢复记录；
- “导出记录”：把本次 GUI 内存中的记录导出为 CSV；
- “关闭”：隐藏窗口。

默认不显示 OK/恢复记录，默认显示 WARN、ERROR 和 STALE。新的 ERROR 或 STALE
会自动唤起状态中心；WARN 只更新按钮和列表，不自动抢占当前操作。

### 3.3 自动弹窗和显示规则

自动弹出必须满足“新激活的 ERROR/STALE”这一条件。相同异常的周期上报只更新
最后出现时间，不能每秒把窗口重新弹到前台。

- `WARN`：功能还能运行，但已经降级、限幅、等待或存在风险；
- `ERROR`：功能失败、输入非法或安全动作未成功；
- `STALE`：原本应该存在的节点或数据流失联；
- `OK/恢复`：此前同一模块的活动异常已经解除；
- 表格按最新事件在前显示；
- “次数”表示该模块与错误码从恢复状态重新进入异常状态的次数，不是同一异常
  每秒重复发布的报文数。

建议后续增加“本次运行不再自动弹出”，但它只能关闭弹窗动作，不能停止异常
采集、计数和留档。当前版本尚未实现静音或确认功能。

## 4. ROS 诊断消息协议

所有 ROS 模块统一向 `/diagnostics` 发布
`diagnostic_msgs/DiagnosticArray`。

### 4.1 DiagnosticStatus 字段

| 字段 | 必填 | 说明 | 示例 |
| --- | --- | --- | --- |
| `name` | 是 | 稳定且唯一的状态机名称 | `/anav/tag_control` |
| `level` | 是 | `OK(0)`、`WARN(1)`、`ERROR(2)`、`STALE(3)` | `2` |
| `message` | 是 | 给操作人员看的简短中文说明 | `标签数据持续超时，任务已终止` |
| `hardware_id` | 是 | 设备或底盘型号 | `ranger`、`ranger_mini_v2` |
| `values.code` | 是 | 稳定错误码 | `ANAV-TAG-007` |
| `values.active` | 是 | `true` 表示未恢复，`false` 表示正常或恢复 | `true` |
| `values.kind` | 是 | `FAULT` 或 `STATE` | `FAULT` |
| `values.detail` | 建议 | 具体原因、超时时长、退出码等 | `/tag_position 5.0 秒未更新` |
| `values.action` | 建议 | 面向现场人员的处理建议 | `检查标签和串口读取节点` |

模块可增加扩展字段。例如碰撞安全层还会发布 `state`、`reason`、`source`、
`profile`、`scale`、`collision_time` 和输出速度。扩展字段必须保持键名稳定。

### 4.2 名称与状态机约束

一个 `name` 代表一个当前状态机。GUI 收到该模块新的活动错误码时，会结束该模块
之前的活动错误，并把新错误设为当前错误。

- 一个节点任意时刻只表示一种主要状态时，可以固定使用一个 `name`；
- 同一节点可能同时存在多个独立故障时，必须拆成不同子项，例如
  `/anav/localization/odom` 和 `/anav/localization/tf`；
- 不允许用不断变化的 `name` 或错误码携带时间、计数和实测值；
- 时间、计数、超时秒数和硬件原始码应放在 `detail` 或扩展 `values` 中。

### 4.3 发布示例

```cpp
diagnostic_msgs::DiagnosticArray array;
array.header.stamp = ros::Time::now();

diagnostic_msgs::DiagnosticStatus status;
status.name = "/anav/example";
status.hardware_id = "ranger";
status.level = diagnostic_msgs::DiagnosticStatus::ERROR;
status.message = "示例数据超时";

// values 至少加入：
// code=ANAV-XXX-001
// active=true
// kind=FAULT
// detail=已连续 3.0 秒没有收到数据
// action=检查示例节点和输入话题

array.status.push_back(status);
diagnostics_pub.publish(array);
```

恢复消息应使用同一个 `name`：

```text
name=/anav/example
level=OK
message=示例数据已恢复
code=ANAV-XXX-000
active=false
kind=STATE
```

### 4.4 错误生命周期与去重

状态中心按 `(name, code)` 识别同一活动异常：

1. 第一次收到 `active=true`：创建事件、计数一次、写入 JSONL；
2. 再次收到相同活动异常：不新增行，只更新 `last_seen`、`detail` 和扩展字段；
3. 同一 `name` 改为另一个活动错误码：旧状态结束，新错误创建一行；
4. 收到同一 `name` 的 `OK` 或 `active=false`：结束该模块所有活动异常并追加一条
   恢复记录；
5. 该错误恢复后再次出现：创建新事件，“发生次数”加一；
6. 从未存在活动异常时收到普通 OK：不新增恢复记录，避免正常心跳刷屏。

恢复消息可以使用模块的 `*-000`，GUI 依据 `name` 结束该模块当前活动异常，而不
要求恢复码与故障码完全相同。

## 5. 错误码规范

格式统一为 `ANAV-<DOMAIN>-<NNN>`：

- `000` 保留给正常、完成或恢复状态；
- `001` 至 `899` 是已定义故障或警告；
- `900` 至 `998` 保留给临时兼容和厂商扩展；
- `999` 是 GUI 在消息缺少 `values.code` 时使用的兜底码，正式模块不得主动使用；
- 已发布错误码的含义不能改变；废弃后保留并标记“废弃”。

| 前缀 | 模块 |
| --- | --- |
| `ANAV-SYS-*` | GUI、ROS Master 与总体运行环境 |
| `ANAV-LOC-*` | 定位进程与定位数据 |
| `ANAV-MAP-*` | 静态地图、点位文件与导航拓扑 |
| `ANAV-NAV-*` | AutoNAV、MoveBase 与导航任务 |
| `ANAV-TAG-*` | 标签读取与靠站控制 |
| `ANAV-ARB-*` | 多来源速度仲裁与停车回正调用 |
| `ANAV-SAF-*` | 碰撞监控、地图/里程计/TF 安全检查 |
| `ANAV-BASE-*` | CAN、Ranger 底盘反馈与轮组回正 |

## 6. 当前错误码清单

下表以当前代码为准。级别写“动态”表示发布级别由具体状态决定。

### 6.1 系统与定位

| 错误码 | 级别 | 检测者 | 触发条件 | 恢复条件 |
| --- | --- | --- | --- | --- |
| `ANAV-SYS-001` | ERROR | GUI | ROS Master 启动检查失败或健康检查无法执行 `rosnode list` | ROS Master 可重新连接 |
| `ANAV-SYS-999` | 动态 | 状态中心兜底 | 收到的诊断消息没有 `values.code` | 修复发布者消息格式 |
| `ANAV-LOC-000` | OK | GUI | 定位未启动，作为正常状态 | 不适用 |
| `ANAV-LOC-001` | WARN | GUI | 定位已启动，但尚未收到 `/Odometry` | 收到定位里程计 |
| `ANAV-LOC-002` | STALE | GUI | 曾收到 `/Odometry`，之后超过 2 秒没有更新 | `/Odometry` 恢复更新 |
| `ANAV-LOC-003` | ERROR | GUI | GUI 启动的定位进程意外退出或启动失败 | 定位重新正常运行 |

### 6.2 地图和导航

| 错误码 | 级别 | 发布者/检测者 | 触发条件 | 建议检查 |
| --- | --- | --- | --- | --- |
| `ANAV-MAP-002` | ERROR | AutoNAV | `map.yaml` 或图像缺失、格式错误、无法读取 | `~/maps/map.yaml`、`image` 路径和权限 |
| `ANAV-MAP-004` | ERROR | AutoNAV | `robot_positions.txt` 缺失或内容无效 | 点位文件路径、格式和点位数量 |
| `ANAV-MAP-009` | ERROR | AutoNAV | `topology.yaml` 缺失、与点位/地图不匹配或校验失败 | 在 GUI 重新构建拓扑并查看输出 |
| `ANAV-NAV-000` | OK | AutoNAV | 配置检查通过，或任务进入运行/到达状态 | 不适用 |
| `ANAV-NAV-001` | STALE | GUI | AutoNAV 进程仍在，但 `/runnav` 连续 3 次检查缺失 | `/runnav` 恢复 |
| `ANAV-NAV-002` | ERROR | GUI | AutoNAV 启动失败或进程意外退出 | AutoNAV 重新正常运行 |
| `ANAV-NAV-003` | ERROR | GUI | MoveBase 启动失败、启动后退出或意外退出 | MoveBase 重新正常运行 |
| `ANAV-NAV-004` | ERROR | GUI | 启动 MoveBase 后 30 秒仍未发现 `/mxb_move_base` | 地图、TF、参数和启动日志 |
| `ANAV-NAV-012` | ERROR | AutoNAV | `/plan_path_and_go` 任务执行失败 | 下一任务进入 running/arrived 时恢复 |

### 6.3 标签靠站

| 错误码 | 级别 | 发布者/检测者 | 触发条件 | 建议检查 |
| --- | --- | --- | --- | --- |
| `ANAV-TAG-000` | OK | TagCtl | 标签数据恢复或靠站任务完成 | 不适用 |
| `ANAV-TAG-001` | STALE | GUI | 标签进程存在，但 `/TagCtl_service`、`/mm3v_serial_reader` 未全部运行 | 两个节点及启动日志 |
| `ANAV-TAG-002` | ERROR | GUI | 标签服务启动失败或相关进程意外退出 | MM3V、串口权限、TCP Server 和启动文件 |
| `ANAV-TAG-006` | WARN | TagCtl | 活动靠站任务未收到有效 `/tag_position` | 标签视野、识别和串口链路 |
| `ANAV-TAG-007` | ERROR | TagCtl | 标签持续无效达到 `tag_abort_timeout`，任务终止 | 排除链路问题后重新下发任务 |
| `ANAV-TAG-008` | WARN | TagCtl | 前一个靠站请求仍在执行，新请求被拒绝 | 等待当前任务完成 |
| `ANAV-TAG-010` | ERROR | TagCtl | 无法调用停车回正服务，或任务结束后未确认回正 | 仲裁、底盘和转向机构 |

### 6.4 速度仲裁

| 错误码 | 级别 | 触发条件 | 建议检查 |
| --- | --- | --- | --- |
| `ANAV-ARB-000` | OK | 仲裁器没有活动故障 | 不适用 |
| `ANAV-ARB-001` | STALE | 底盘栈已启动，但 `/cmd_vel_arbiter` 连续 3 次检查缺失 | 节点与启动日志 |
| `ANAV-ARB-002` | WARN | 导航速度来源超时，已触发停车回正 | 导航节点和速度话题 |
| `ANAV-ARB-003` | WARN | 标签速度来源超时，已触发停车回正 | TagCtl 和标签速度话题 |
| `ANAV-ARB-005` | ERROR | 某速度输入包含 NaN 或无穷值 | 详情中的速度来源话题 |
| `ANAV-ARB-006` | ERROR | `/cmd_vel/safety` 收到非零速度并被强制清零 | 安全通道发布者 |
| `ANAV-ARB-007` | WARN | 输入超过该来源速度上限并被限幅 | 速度参数和底盘能力 |
| `ANAV-ARB-008` | WARN | 安全速度通道正在接管 | 现场安全情况和急停状态 |
| `ANAV-ARB-011` | ERROR | Ranger `/stop_and_center` action 不可用 | `ranger_base` 和 action 服务 |
| `ANAV-ARB-012` | ERROR | 等待停车回正超过配置时限 | 底盘反馈、运动模式和转向机构 |
| `ANAV-ARB-013` | ERROR | 停车回正 action 返回失败 | action 返回详情和底盘日志 |

仲裁器周期发布当前状态。短时 WARN 会保持配置的诊断窗口，结束后发布
`ANAV-ARB-000` 恢复。

### 6.5 碰撞安全层

| 错误码 | 常见级别 | 状态或原因 | 建议检查 |
| --- | --- | --- | --- |
| `ANAV-SAF-000` | OK | 正常、停止命令或机器人已停止 | 不适用 |
| `ANAV-SAF-001` | STALE | 底盘栈已启动，但 `/collision_monitor` 连续 3 次检查缺失 | 节点和启动日志 |
| `ANAV-SAF-002` | ERROR | `STATIC_MAP_NOT_READY` | `/map` 和静态地图加载节点 |
| `ANAV-SAF-003` | ERROR | `LOCAL_MAP_NOT_READY` | 局部代价地图和 MoveBase |
| `ANAV-SAF-004` | ERROR | `ODOM_STALE` 或 `ODOM_NON_FINITE` | `/odom` 更新及数值 |
| `ANAV-SAF-005` | ERROR | TF 不可用、零时间戳、过期或非有限值 | `map` 到 `base_link` 定位 TF |
| `ANAV-SAF-006` | ERROR | `TF_JUMP` | 定位跳变和 TF 发布者 |
| `ANAV-SAF-008` | ERROR | 候选轨迹、指令或实测运动包含非有限值 | 当前速度来源和运动模式 |
| `ANAV-SAF-009` | ERROR | `COMMAND_LIMIT_EXCEEDED` | 速度上限与运动模式参数 |
| `ANAV-SAF-010` | WARN | `SLOWDOWN` | 前后障碍物、地图膨胀和预测时间 |
| `ANAV-SAF-011` | WARN | 静态地图预测碰撞并停车 | 静态障碍物和地图 |
| `ANAV-SAF-012` | WARN | 局部地图预测碰撞并停车 | 动态障碍物和局部代价地图 |
| `ANAV-SAF-013` | ERROR | `EMERGENCY_STOP` | 急停来源和现场安全状态 |
| `ANAV-SAF-014` | WARN | `PROFILE_TRANSITION` | 运动轮廓正在切换，通常短时出现 |
| `ANAV-SAF-015` | ERROR | `UNSUPPORTED_MOTION` | 当前速度来源与轮组模式配置 |

碰撞安全层的 `message` 当前保留英文状态和原因，例如
`COLLISION_STOP: STATIC_COLLISION`；详情区保留原始
`state/reason/source/profile/scale/collision_time`，便于研发排查。

### 6.6 CAN 和 Ranger 底盘

| 错误码 | 级别 | 触发条件 | 建议检查 |
| --- | --- | --- | --- |
| `ANAV-BASE-000` | OK | 速度恢复，或底盘已停车且轮组回正 | 不适用 |
| `ANAV-BASE-001` | ERROR | 启动 CAN 后未检测到 `can0`，或启动命令异常 | CAN 设备、驱动和 sudo 权限 |
| `ANAV-BASE-002` | WARN | 检测到 `can0`，但接口没有 UP | 接线、bitrate=500000 和接口状态 |
| `ANAV-BASE-003` | ERROR/STALE | 底盘启动失败、进程退出或 `/ranger_base_node` 失联 | CAN 和底盘日志 |
| `ANAV-BASE-005` | ERROR | Ranger `error_code` 从 0 变为非 0，运动被锁定 | 详情中的十六进制底盘错误码 |
| `ANAV-BASE-007` | ERROR | `/cmd_vel` 含 NaN 或无穷值 | 仲裁输出和上游速度节点 |
| `ANAV-BASE-008` | WARN | `/cmd_vel` 超时，底盘停车并请求回正 | 仲裁节点和 `/cmd_vel` |
| `ANAV-BASE-009` | ERROR | 停车阶段未在时限内确认静止 | 轮速反馈和机械状态 |
| `ANAV-BASE-010` | ERROR | 回正时运动模式切换失败或超时 | 运动模式反馈和控制器 |
| `ANAV-BASE-011` | ERROR | 其他停车回正失败或回正超时 | 底盘反馈与转向机构 |
| `ANAV-BASE-015` | WARN | 原始故障已清除，但正在回正，运动锁尚未解除 | 等待自动回正完成 |

## 7. GUI 本地健康检查

GUI 每 1 秒发起一次异步健康检查，避免阻塞界面。

### 7.1 ROS Master

通过带 2 秒超时的 `rosnode list` 判断 ROS Master 是否可达。失败时更新顶部 ROS
状态并激活 `ANAV-SYS-001`，恢复后关闭异常。

### 7.2 CAN

读取 `/sys/class/net/can0` 和接口 flags：

- 用户未点击启动 CAN 时，只显示状态，不把“未检测”作为活动故障；
- 点击启动 CAN 3 秒后仍未 UP，才产生异常；
- 接口不存在使用 `ANAV-BASE-001`；
- 接口存在但未 UP 使用 `ANAV-BASE-002`。

### 7.3 进程与节点

- 子进程有退出码：立即记录 ERROR；
- 启动进程仍在而预期节点连续 3 次检查缺失：记录 STALE；
- 正常用户停止和 GUI 退出期间通过 `shutdown_in_progress` 抑制“意外退出”；
- 进程或节点恢复后发布本地 OK，结束对应活动异常。

### 7.4 定位数据

GUI 监听 `/Odometry`：

- 定位刚启动且从未收到数据：`ANAV-LOC-001`；
- 曾经收到数据，超过 2 秒未更新：`ANAV-LOC-002`；
- 收到新数据：恢复定位状态；
- 定位停止：顶部状态同步为“定位未启动”，不能继续显示“定位已就绪”。

## 8. 日志与导出

### 8.1 自动 JSONL 日志

新激活异常和恢复事件按天追加写入：

```text
~/anav_logs/diagnostics_YYYYMMDD.jsonl
```

| 字段 | 说明 |
| --- | --- |
| `timestamp` | 首次发生的 Unix 时间 |
| `last_seen` | 最后一次收到相同活动异常的时间 |
| `level` | 0/1/2/3 |
| `code` | 错误码 |
| `module` | `DiagnosticStatus.name` |
| `message` | 简短描述 |
| `detail` | 详细原因 |
| `action` | 处理建议 |
| `kind` | `FAULT`、`STATE` 或 `RECOVERY` |
| `active` | 写入时是否未恢复 |
| `occurrences` | 本次 GUI 运行内的激活次数 |
| `values` | ROS 消息的全部扩展字段 |
| `resolved_at` | 恢复时间；未恢复时为 `null` |

相同活动错误的周期心跳不会重复写盘，只更新 GUI 内存中的 `last_seen`。恢复事件
单独写一行。

当前版本启动 GUI 时不会回读以前的 JSONL，因此窗口只展示本次运行收到的记录；
磁盘历史仍可直接查看。日志尚未实现自动保留天数和容量清理。

### 8.2 CSV 导出

“导出记录”导出当前内存中的全部事件，默认路径为
`~/anav_diagnostics.csv`。CSV 使用 UTF-8 BOM，包含时间、最后更新、级别、错误码、
模块、描述、原因、建议、状态和发生次数。

“清除已恢复记录”不删除 JSONL 文件。历史日志的删除必须由维护人员明确执行，
不能由普通窗口清理按钮隐式完成。

## 9. 当前覆盖情况与待补充项

### 9.1 已实现

- 主窗口状态中心入口、未恢复计数和颜色提示；
- ERROR/STALE 新事件自动打开独立窗口；
- `/diagnostics` 订阅、去重、恢复和详情展示；
- ROS Master、CAN、受管进程、预期节点和定位数据监控；
- AutoNAV 启动前的地图、点位、拓扑检查；
- 导航任务失败；
- 标签短时无效、持续超时、控制忙和停车回正失败；
- 速度仲裁非法值、限幅、来源超时、安全接管和回正失败；
- 碰撞安全层地图、里程计、TF、障碍和运动模式异常；
- Ranger 底盘错误码、非法速度、速度超时和回正异常；
- JSONL 自动留档与 CSV 手动导出。

### 9.2 尚未完整接入

以下错误目前可能只出现在主窗口状态栏或终端：

- 建图进程、三维转二维和 map_saver 保存失败；
- `build_topo.py` 启动失败、超时、返回非 0、输出不存在或校验失败；
- 点位新增、继续添加、修改、撤销时的文件读写和格式校验失败；
- RViz 启动失败、记录点位工具失联、等待 RViz 选点超时；
- 手柄驱动启动失败和 `/joy` 发布不可用；
- `/plan_path_and_go` 命令调用失败但 AutoNAV 未返回任务状态的情况；
- 循环任务线程、取消导航服务和参数服务调用失败；
- TCP Server 客户端连接、协议解析和业务异常；
- 磁盘空间不足、JSONL 写入失败和 CSV 导出失败的统一错误码；
- 诊断发布者自身失联；
- GUI 重启后历史回读、异常确认、静音、搜索、模块筛选和日志保留策略。

建议下一阶段优先补充：

| 建议错误码 | 用途 | 推荐级别 |
| --- | --- | --- |
| `ANAV-MAP-010` | 拓扑构建命令启动失败 | ERROR |
| `ANAV-MAP-011` | 拓扑构建进程返回非 0 | ERROR |
| `ANAV-MAP-012` | 拓扑输出不存在或校验失败 | ERROR |
| `ANAV-MAP-013` | 点位文件读取/解析失败 | ERROR |
| `ANAV-MAP-014` | 点位文件写入或原子替换失败 | ERROR |
| `ANAV-MAP-015` | RViz 选点等待超时或工具失联 | WARN |
| `ANAV-MAP-016` | 地图保存失败 | ERROR |
| `ANAV-NAV-013` | 导航服务不可用或调用失败 | ERROR |
| `ANAV-NAV-014` | 取消任务服务失败 | WARN |
| `ANAV-NAV-015` | 循环任务线程异常退出 | ERROR |
| `ANAV-SYS-002` | 诊断日志写入失败或磁盘空间不足 | ERROR |
| `ANAV-SYS-003` | RViz/外部工具启动失败 | ERROR |

这些编号在代码接入前属于方案预留；正式接入时必须同步更新发布条件、恢复条件和
测试用例。

## 10. 新模块接入要求

1. 在 `CMakeLists.txt` 和 `package.xml` 中声明 `diagnostic_msgs`；
2. 发布到 `/diagnostics`，队列大小建议 10；
3. 为每个独立状态机选择稳定的 `name`；
4. 在本文登记错误码，禁止与已有语义冲突；
5. 同时实现故障激活和恢复，不允许只报错不恢复；
6. 高频状态使用节流或周期心跳，不允许高频创建新事件；
7. `message` 面向现场人员，`detail` 保留研发上下文；
8. `action` 必须给出可执行检查项，不能只写“请联系管理员”；
9. 为激活、重复、恢复和再次发生四种情况编写测试；
10. 验证节点退出或 ROS Master 失联时 GUI 不阻塞。

不要把正常控制循环调试、每帧传感器数据、普通任务进度、无操作意义的内部状态
切换，以及风险等级未变化的采样数值全部转为故障。

## 11. 验收测试

### 11.1 GUI 行为

1. 正常启动 GUI：状态中心按钮不带数量；
2. 发布一个 WARN：按钮变橙并显示 1，窗口不自动抢焦点；
3. 重复发布同一 WARN：列表不新增行，最后更新时间更新；
4. 发布 ERROR：窗口自动打开，按钮变红；
5. 发布同模块 OK：原异常变为已恢复，未恢复计数归零；
6. 取消勾选恢复/OK：恢复记录隐藏但未删除；
7. 清除已恢复记录：当前列表清理，磁盘 JSONL 保留；
8. 导出 CSV：中文、时间和详情可正常打开。

### 11.2 现场故障注入

| 场景 | 预期错误码 |
| --- | --- |
| 停止 roscore 或断开 ROS Master 网络 | `ANAV-SYS-001` |
| 定位运行后停止 `/Odometry` 超过 2 秒 | `ANAV-LOC-002` |
| 移走 `map.yaml` 后启动 AutoNAV | `ANAV-MAP-002` |
| 破坏 `robot_positions.txt` 格式 | `ANAV-MAP-004` |
| 删除或破坏 `topology.yaml` | `ANAV-MAP-009` |
| 靠站时遮挡标签直至超过超时 | 先 `ANAV-TAG-006`，后 `ANAV-TAG-007` |
| 向仲裁输入发布 NaN | `ANAV-ARB-005` |
| 停止 Ranger 回正 action server | `ANAV-ARB-011` 或 `ANAV-TAG-010` |
| 停止 `/odom` | `ANAV-SAF-004` |
| 制造定位 TF 跳变 | `ANAV-SAF-006` |
| Ranger 控制器上报非零 error_code | `ANAV-BASE-005` |
| 停止 `/cmd_vel` 更新 | `ANAV-BASE-008` |

每项都要确认：错误码正确、原因可读、建议可执行、恢复后计数归零。

### 11.3 构建与静态检查

```bash
catkin_make --source . -DCATKIN_WHITELIST_PACKAGES="cmd_vel_arbiter;collision_monitor;x2bot_teleop;ranger_base"
python3 -m py_compile src/script/aNAV_ranger.py src/script/fault_center.py
```

实际命令按当前工作空间结构调整。构建通过只证明接口和语法正确，仍需执行 GUI
行为与故障注入测试。

## 12. 已知限制与演进顺序

当前状态中心是单机 GUI 内存模型，适合现场操作和第一阶段交付，但不是集中式告警
平台。后续如需多机器人、远程运维或长期统计，可以保持本消息协议不变，增加独立
诊断采集节点，将 `/diagnostics` 转存到 SQLite、HTTP 或集中日志系统。

建议演进顺序：

1. 完成第 9.2 节的 GUI 操作错误接入；
2. 增加诊断源心跳与源失联错误，避免“发布者死了所以没有错误”的盲区；
3. 增加历史回读、搜索、模块筛选和异常确认；
4. 增加日志容量上限、保留天数和磁盘空间告警；
5. 稳定运行后再考虑远程汇总，不改变现场 GUI 的基本操作方式。
