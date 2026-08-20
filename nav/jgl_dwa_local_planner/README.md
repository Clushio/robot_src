# jgl_dwa_local_planner 项目定制说明

本包源自 ROS DWA Local Planner，但当前版本包含大量项目定制，不能按上游默认 DWA
理解。主要新增内容包括 B 样条参考路径、双阿克曼跟踪、失败段回退，以及两个代价
地图插件。

## 主要组件

| 组件 | 文件 | 作用 |
|---|---|---|
| `DWAPlannerROS` | `src/dwa_planner_ros.cpp` | MoveBase 局部规划插件、模式切换和导航状态 |
| `TrajectoryGenerator` | `src/trajectory_generator.cpp` | cubic/B 样条生成、优化、碰撞和曲率约束 |
| `ReferencePathManager` | `src/reference_path_manager.cpp` | 参考路径生命周期、进度和局部段管理 |
| `PathFollower` | `src/path_follower.cpp` | 参考路径跟踪、曲率到双阿克曼控制 |
| `DecayObstacleLayer` | `src/decay_obstacle_layer.cpp` | 障碍物随时间衰减的代价地图层 |
| `BlindClearLayer` | `src/blind_clear_layer.cpp` | 清除雷达盲区造成的历史障碍残留 |

## 参考路径处理

当前默认参数：

```yaml
enable_bspline_reference_path: true
reference_curve_type: bspline
```

生成器从全局/拓扑路径构造控制点，并综合以下代价优化：

- `bspline_weight_smooth`：平滑度；
- `bspline_weight_obstacle`：障碍距离；
- `bspline_weight_topo`：偏离原拓扑路径；
- `bspline_weight_curvature`：曲率；
- `max_curvature`、`min_turn_radius`：几何可行性；
- `safe_distance`：参考路径障碍安全距离。

完整路径不能安全优化时，可以根据失败样本定位问题段，对局部段使用旧跟踪/DWA 回退，
而不是接受一条总代价更差的曲线。

## 跟踪平滑

`PathFollower` 使用前视距离和参考曲率生成控制命令。为降低双阿克曼轮组抖动，当前
包含：

- `curvature_filter_tau`：一阶低通时间常数；
- `max_curvature_rate`：每秒最大曲率变化；
- `curvature_deadband`：小变化死区；
- `tracking_max_curvature`：跟踪阶段上限，与生成阶段上限分开；
- `control_period`：用于滤波和变化率计算的控制周期。

这组参数存在联动。增大平滑会降低抖动，但也可能造成转弯响应滞后和路径偏差。修改后
需要同时看曲率、轮组转角、路径误差和障碍净空。

B 样条中间拓扑点和末端使用独立到达语义：中间点按
`reference_middle_pass_distance` 的路径进度连续通过；倒数第二个拓扑点是 B 样条
末端，只有 `map->base_link` 的真实距离进入 `reference_terminal_xy_tolerance` 后才
发布通过状态。到达最后一个路径采样索引但尚未进入末端容差时，控制器继续跟踪最后
一段，不把“索引走完”误认为“车体到点”。该末端是无停车交接点，因此不应用
`end_slow_distance` 的路径末端减速；曲率限速、障碍物分级减速和 Collision Monitor
安全缩放仍然有效。

## 堵塞和模式

局部规划器发布 `/bspline_status` 给 AutoNAV。AutoNAV 决定是否封锁拓扑边和重新选路。
`/anav/fixed_route_mode` 为 true 时，局部层遵守固定路线语义，堵塞后停车等待，不由
全局规划器自行生成偏离拓扑的 DWA 绕行。

运行时在 B 样条上发现障碍后，控制器按照到首个阻塞采样点的路径弧长分级限制候选
速度：`obstacle_slowdown_distance` 外保持全速，进入该距离后依次限制为 75%、50%、
25%，到 `obstacle_stop_distance` 时请求停车。线速度和角速度同比例缩放以保持曲率。
该逻辑用于提前减速、等待和超时重规划；最终速度仍由 `collision_monitor` 按完整
footprint 和制动轨迹裁决。

全局规划配置中的 `enable_dwa_obstacle_avoidance: false` 与这一所有权划分有关：
AutoNAV 负责拓扑重规划，局部层不应静默改变业务路线。

## 代价地图插件

插件在 `costmap_plugin.xml` 注册，参数位于 `param/costmap_common_params.yaml`。

### DecayObstacleLayer

从 `/livox_pcl0` 接收点云，把障碍写入代价地图并按 `decay_time` 衰减。它用于减少移动
障碍离开后长时间残留，但不能替代正确的传感器 clearing 和最终碰撞监控。

### BlindClearLayer

按雷达有效角度范围和清理距离清除盲区中的历史栅格，同时保留机器人近身半径。
角度必须与 MID360s 的安装朝向和 LIO FOV 保持一致。

## 配置文件

| 文件 | 内容 |
|---|---|
| `param/dwa_local_planner_params.yaml` | B 样条、跟踪、DWA 和到达容差 |
| `param/costmap_common_params.yaml` | 障碍层、衰减层、盲区层、膨胀层 |
| `param/local_costmap_params.yaml` | 局部代价地图窗口和插件顺序 |
| `param/global_costmap_params.yaml` | 全局代价地图 |
| `param/global_planner_params.yaml` | 定制全局规划和堵塞行为 |
| `param/move_base_params.yaml` | MoveBase 频率和恢复配置 |

物理 footprint 不在本目录定义；统一从
`collision_monitor/config/robot_footprint.yaml` 加载。

## 测试

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make run_tests_jgl_dwa_local_planner
catkin_test_results build
```

`test/trajectory_generator_test.cpp` 覆盖曲线生成和部分跟踪边界。单元测试之外仍需要：

- 静态障碍和窄通道检查；
- S 弯、急弯和终点角度；
- 路径生成失败后的回退段；
- 堵塞、恢复和固定路线模式；
- 双阿克曼轮组转角/转角速度；
- 与 `collision_monitor` 最终停车行为的联调。

## 修改约束

- 不要把生成曲率上限和跟踪曲率上限混为一个参数。
- 不要让局部回退绕过 AutoNAV 的固定路线语义。
- 不要在此包直接发布最终 `/cmd_vel`；MoveBase 输出应进入 `/cmd_vel/nav`。
- 修改障碍高度/FOV 时同步核对 LIO、点云转换、costmap 和碰撞监控。
- 新增参数时同时更新 YAML、加载代码、测试和本文。
