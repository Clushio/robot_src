# cmd_vel_arbiter

`cmd_vel_arbiter` 在多个速度来源之间做确定性选择，并把带来源信息的候选命令交给
`collision_monitor`。它不负责地图碰撞判断，也不是最终 `/cmd_vel` 发布者。

## 数据流

```text
/cmd_vel/safety  priority 100 ┐
/cmd_vel/teleop  priority  80 ├─> cmd_vel_arbiter
/cmd_vel/tag     priority  60 ┤       │
/cmd_vel/nav     priority  20 ┘       └─> /cmd_vel/candidate
                                              │
                                              v
                                      collision_monitor
                                              │
                                           /cmd_vel
```

节点由 Ranger 底盘 launch 自动包含：

```bash
ros2 launch ranger_bringup ranger_mini_v2.launch.py
```

单独调试：

```bash
ros2 launch cmd_vel_arbiter cmd_vel_arbiter.launch.py
```

单独启动仲裁器不会使实车链路完整；还需要碰撞监控和底盘驱动。

## 仲裁规则

每个输入包含：

- Topic；
- 数值越大的优先级；
- 接收超时；
- `linear.x`、`linear.y`、`angular.z` 绝对上限；
- 是否仅允许零速度（安全通道）。

每个周期选择“仍新鲜且优先级最高”的来源。切换来源时先发布配置的零速度周期，避免
两个控制器命令直接跳变。超出上限、非有限数值或安全通道非零命令不会被当成正常
业务命令放行。

参数文件：`config/cmd_vel_arbiter.yaml`。

当前默认值：

| 来源 | Topic | 优先级 | 超时 | 最大速度 `(x, y, yaw)` |
|---|---|---:|---:|---|
| safety | `/cmd_vel/safety` | 100 | 0.25 s | 只允许零速度 |
| teleop | `/cmd_vel/teleop` | 80 | 0.25 s | `(0.35, 0.10, 1.10)` |
| tag | `/cmd_vel/tag` | 60 | 0.25 s | `(0.15, 0.15, 0.25)` |
| nav | `/cmd_vel/nav` | 20 | 0.25 s | `(0.30, 0.10, 4.10)` |

表中的单位分别为 m/s、m/s、rad/s。它们是仲裁层硬上限，不代表实际允许速度；
碰撞监控还会施加更保守的来源 profile 上限和制动预测。

## 候选输出

`/cmd_vel/candidate` 类型为 `cmd_vel_arbiter/msg/ArbitratedCommand`，包含：

- Header；
- 被选择的 `source`；
- `geometry_msgs/Twist command`。

来源字段让碰撞监控能够选择导航、Tag 或遥控 profile。不要用普通 Twist 替换该 Topic，
也不要让业务节点绕过它发布最终 `/cmd_vel`。

## 任务结束和停车回正

服务：

```text
/cmd_vel_arbiter/finish_motion
```

类型：`cmd_vel_arbiter/srv/FinishMotion`。调用方提供来源和结束原因，仲裁器会：

1. 暂时抑制该来源的排队/残留命令；
2. 调用 `/stop_and_center` action；
3. 等待底盘停车和轮组回正结果；
4. 返回是否确认回正及说明。

导航和 Tag 节点在任务成功、失败或取消时都会调用此服务。普通发布一个零 Twist 只表示
当前周期不运动，不保证轮组回正。

### 调用方来源

`source` 不是任意描述字符串，必须使用仲裁器已经注册的固定来源名：

| `source` | 对应输入 | 当前结束服务调用情况 |
|---|---|---|
| `nav` | `/cmd_vel/nav` | 正常完成、主动取消、任务失败 |
| `tag` | `/cmd_vel/tag` | 正常完成、任务失败 |
| `safety` | `/cmd_vel/safety` | 软件急停 |
| `teleop` | `/cmd_vel/teleop` | 仲裁器接受该来源，但当前没有调用方主动结束遥控 |

Topic 可以通过参数修改，但上述来源名目前固定在仲裁器代码中。传入其他字符串时，
服务返回 `accepted=false`，不会发起停车回正。

### 结束原因

调用方只能使用 `FinishMotion.srv` 中定义的常量：

| 数值 | 常量 | 含义 |
|---:|---|---|
| `0` | `TASK_FINISHED` | 任务正常完成 |
| `1` | `TASK_CANCELED` | 用户或 GUI 主动取消任务 |
| `2` | `TASK_FAILED` | 导航失败、Tag 数据持续失效等任务失败 |
| `4` | `SOFTWARE_ESTOP` | 软件急停；应与 `source=safety` 配合使用 |

当前代码中的实际调用组合为：

```text
nav    + TASK_FINISHED
nav    + TASK_CANCELED
nav    + TASK_FAILED
tag    + TASK_FINISHED
tag    + TASK_FAILED
safety + SOFTWARE_ESTOP
```

底盘的 `StopAndCenter.action` 另外定义了 `CMD_TIMEOUT=3` 和 `CHASSIS_FAULT=5`。它们
用于仲裁器和底盘驱动内部触发，不属于 `FinishMotion` 业务调用方应提交的结束原因。
当前服务回调没有额外校验 `reason` 数值范围，因此调用方必须使用服务定义的常量，
不能自行传入其他 `uint8` 数值。

服务响应字段含义：

- `accepted`：来源名是否被仲裁器识别；
- `centered`：本次停车和轮组回正是否得到成功确认；
- `message`：未执行或失败的具体说明。

除 `SOFTWARE_ESTOP` 外，如果该来源已经不再拥有运动控制权，服务可能返回
`accepted=true`、`centered=false`，表示请求格式有效，但没有再次发起回正。

相关参数：

- `stop_and_center_action`；
- `stop_and_center_wait_timeout`；
- `finish_source_suppression`；
- `center_on_nav_loss`；
- `center_on_tag_loss`。

## 软件急停

GUI 激活软件急停后持续向 `/cmd_vel/safety` 发布零 Twist。由于 safety 优先级最高，
其他业务来源不会被选择。

限制：软件急停依赖 ROS 2 通信图、GUI、仲裁器、碰撞监控、底盘节点和通信链路正常，
因此不能替代物理急停。

## 排障

```bash
ros2 topic hz /cmd_vel/candidate
ros2 topic echo /cmd_vel/candidate
ros2 topic echo /cmd_vel/safety
ros2 service type /cmd_vel_arbiter/finish_motion
ros2 action info /stop_and_center
```

### 候选速度一直为零

- 检查是否有新鲜的高优先级零速度来源；
- 检查业务输入是否超时；
- 检查命令是否超出该来源上限；
- 检查来源切换零速度周期；
- 检查结束任务后的临时抑制窗口。

### `/cmd_vel/candidate` 正常但底盘不动

继续检查 `/collision_monitor/status` 和最终 `/cmd_vel`。这通常已经不是仲裁选择问题。

### 任务结束后不回正

检查 `/stop_and_center` action 是否存在、底盘 `/motion_state`、驱动日志和
`stop_and_center_wait_timeout`。不要通过不断重复 finish 请求掩盖底盘状态问题。

## 修改约束

- 新速度来源必须配置 Topic、优先级、超时和三轴上限。
- 同步更新碰撞监控的来源 profile，否则新来源会被视为不支持。
- 安全通道必须保持“只允许零命令”。
- 修改优先级后验证同时到达的命令、超时切换和零周期。
- 修改速度上限后同时核对底盘能力、碰撞监控制动模型和现场制动测试。
