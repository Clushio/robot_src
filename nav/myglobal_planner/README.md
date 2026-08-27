# myglobal_planner（ROS2 Humble）

本包保留项目原有的全局规划算法，并作为
`nav2_core::GlobalPlanner` 插件导出：

```yaml
planner_server:
  ros__parameters:
    planner_plugins: [GridBased]
    GridBased:
      plugin: myglobal_planner/MyGlobalPlanner
```

算法仍先生成直线路径，并使用路径点 `z` 字段向项目局部规划器传递状态：

- `z=1`：纯跟踪直线；
- `z=2`：障碍物存在，停车等待；
- `z=-1`：允许倒车的绕行路径。

发布的可视化路径会把 `z` 归零，避免 RViz 显示偏离地面；返回给控制器的路径标记
保持不变。

## ROS1 到 ROS2 的非等价映射

- `nav_core::BaseGlobalPlanner` 改为 Nav2 lifecycle
  `nav2_core::GlobalPlanner`，插件名称保持
  `myglobal_planner/MyGlobalPlanner`。
- ROS1 `dynamic_reconfigure` 没有 Humble 的直接等价接口。原
  `GlobalPlanner.cfg` 参数改为 `<plugin_id>.<parameter>` ROS2 参数，并通过
  on-set callback 支持运行时更新和范围拒绝。
- ROS1 `mxb_move_base` 会先用 controller costmap、再用 planner costmap 初始化同一
  插件。Nav2 标准接口只传入 planner costmap，而且 PlannerServer 与 ControllerServer
  是独立对象，不能跨进程调用插件扩展。ROS2 插件因此订阅
  `GridBased.local_costmap_topic`（默认 `/local_costmap/costmap_raw`）并在每次规划开始时
  原子切换到最新局部地图；首帧到达前安全回退到全局 costmap。公开的
  `setLocalCostmap()` 仍保留给同进程集成和测试。
- TF1 `TransformListener` 改为 PlannerServer 提供的 TF2 buffer；原 frame 名称取自
  costmap，项目配置仍使用 `map`。
- 原插件私有 `make_plan` 服务保留为
  `~/<plugin_id>/make_plan`（`nav_msgs/srv/GetPlan`）；Nav2 的主要入口为
  `/compute_path_to_pose` action。
- ROS1 定制代码曾关闭局部窗口外目标检查，但后续仍读取未赋值的栅格坐标，结果是
  未定义内存访问而不是有效规划。ROS2 端对这种目标明确返回失败；正常绕障流程仍由
  直线检查先选出局部窗口内的临时目标，不改变有效输入下的算法路径。

参数示例：

```yaml
GridBased:
  plugin: myglobal_planner/MyGlobalPlanner
  lethal_cost: 253
  neutral_cost: 10
  cost_factor: 3.0
  publish_potential: true
  orientation_mode: 1
  orientation_window_size: 1
  distance_convert_line: 0.5
  point_per_meter: 0.1
  distance_check_obstacle: 1.0
  distance_behind_obstacle: 1.0
  wait_time: 3.0
  enable_dwa_obstacle_avoidance: false
```

`src/plan_node.cpp`、`src/astar_modify.cpp` 在 ROS1 的 CMake 中也未构建，属于历史
实验源码，不是插件或运行节点；ROS2 构建仍不安装它们。
