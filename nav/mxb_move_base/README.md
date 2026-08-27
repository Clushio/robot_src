# mxb_move_base（ROS2 Humble）

ROS1 包是 `move_base` 的项目分支。ROS2 版本使用 Nav2 的 PlannerServer、
ControllerServer、BT Navigator 和 lifecycle manager 负责同样的编排，并继续加载项目的
`myglobal_planner/MyGlobalPlanner` 与
`jgl_dwa_local_planner/DWAPlannerROS`，没有用通用算法替换核心导航逻辑。

```bash
ros2 launch mxb_move_base navigation.launch.py
```

## 接口映射

| ROS1 接口 | ROS2 Humble 接口 |
|---|---|
| `/move_base` (`move_base_msgs/MoveBase`) | `/navigate_to_pose` (`nav2_msgs/action/NavigateToPose`) |
| `/move_base_simple/goal` | 保留，由兼容节点转发到 `/navigate_to_pose` |
| `/mxb_move_base/current_goal` | 保留 |
| `/mxb_move_base/make_plan` | 保留，转发到 MyGlobalPlanner 的服务 |
| `/mxb_move_base/clear_costmaps` | 保留，同时调用 Nav2 两个 costmap 清理服务 |
| `/cmd_vel` | ControllerServer 输出重映射为 `/cmd_vel/nav`，最终 `/cmd_vel` 仍只由安全仲裁链发布 |

ROS2 没有 `move_base_msgs` 的 Humble 等价包，因此旧 action 的 wire type 无法原样保留；
项目调用端应使用标准 NavigateToPose。服务和 simple-goal 入口不依赖该消息包，故保持原名。

## 行为兼容

- 普通导航行为树以 5 Hz 重算路径，对应 ROS1 `planner_frequency: 5.0`。
- 固定路线行为树只计算一次路径；兼容节点在收到 transient-local
  `/anav/fixed_route_mode=true` 后，为 simple goal 选择该行为树。
- ROS1 配置禁用了 recovery behavior，ROS2 行为树也不主动清图、旋转或改道。
- 局部 costmap 仍为 `map` frame、3 m 滚动窗口，frame 名保持 `map`、`base_link`、
  `livox_frame`。
- 机器人 footprint 与 `collision_monitor` 使用相同四边形。
- 原 ROS1 `dynamic_reconfigure` 不再生成；Nav2 与两个项目 planner 均使用 ROS2 参数及各
  planner 已实现的在线参数回调。

固定路线直接调用 `/navigate_to_pose` 时，调用者应把
`NavigateToPose.Goal.behavior_tree` 设置为安装目录中的
`behavior_trees/navigate_to_pose_fixed_route.xml`。`x2bot_teleop` 的集成配置会传入该路径。

`cfg/MoveBase.cfg`、`include/move_base/move_base.h`、`src/move_base.cpp` 和
`src/move_base_node.cpp` 仅保留为 ROS1 分支实现参考，不参与 ROS2 构建或安装。
